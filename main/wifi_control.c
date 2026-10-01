#include "wifi_control.h"

#include "sdkconfig.h"

#if CONFIG_RC_WIFI_CONTROL_ENABLED

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dragy_sensor.h"
#include "drivetrain_control.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "powertrain_controller.h"
#include "remote_command.h"
#include "telemetry_log.h"

#define TELEMETRY_BUFFER_SIZE 8192
#define CONFIG_RESPONSE_BUFFER_SIZE 2048
#define COMMAND_ERROR_SIZE 192
#define HTTP_COMMAND_BODY_SIZE 512
#define LOG_EXPORT_CHUNK_SIZE 8192
#define LOG_FREE_RESERVE_BYTES 16384ULL
#define AP_IPV4_ADDRESS "192.168.4.1"

static const char *TAG = "wifi_control";
static httpd_handle_t http_server;
/* Keep the expanded JSON frame and file chunks out of the HTTP task's stack. */
static char telemetry_buffer[TELEMETRY_BUFFER_SIZE];
static char config_response_buffer[CONFIG_RESPONSE_BUFFER_SIZE];
static uint8_t log_export_chunk[LOG_EXPORT_CHUNK_SIZE];

extern const uint8_t web_index_start[]
    asm("_binary_web_index_start");
extern const uint8_t web_index_end[]
    asm("_binary_web_index_end");
extern const uint8_t web_app_start[]
    asm("_binary_web_app_start");
extern const uint8_t web_app_end[]
    asm("_binary_web_app_end");
extern const uint8_t web_style_start[]
    asm("_binary_web_style_start");
extern const uint8_t web_style_end[]
    asm("_binary_web_style_end");

static const char *bool_json(bool value)
{
    return value ? "true" : "false";
}

static double finite_number(float value)
{
    return isfinite(value) ? (double)value : 0.0;
}

static double finite_double(double value)
{
    return isfinite(value) ? value : 0.0;
}

static void format_i2c_addresses(const dragy_sensor_snapshot_t *dragy,
                                 char *buffer,
                                 size_t buffer_size)
{
    if (buffer_size == 0) return;
    size_t used = 0;
    buffer[0] = '\0';
    for (uint8_t index = 0; index < dragy->i2c_device_count; index++) {
        int written = snprintf(buffer + used, buffer_size - used,
                               "%s%u", index == 0 ? "" : ",",
                               dragy->i2c_addresses[index]);
        if (written < 0 || (size_t)written >= buffer_size - used) return;
        used += (size_t)written;
    }
}

static void json_escape_string(const char *source, char *destination, size_t size)
{
    if (size == 0) return;
    size_t output = 0;
    for (size_t input = 0; source[input] != '\0' && output + 1 < size; input++) {
        unsigned char value = (unsigned char)source[input];
        if ((value == '"' || value == '\\') && output + 2 < size) {
            destination[output++] = '\\';
            destination[output++] = (char)value;
        } else if (value >= 0x20) {
            destination[output++] = (char)value;
        } else {
            destination[output++] = ' ';
        }
    }
    destination[output] = '\0';
}

static const char *system_state_name(system_state_t state)
{
    switch (state) {
    case SYSTEM_DISARMED: return "DISARMED";
    case SYSTEM_DRIVE_ARMED: return "DRIVE_ARMED";
    case SYSTEM_ESC_CAL_ARMED: return "ESC_CAL_ARMED";
    case SYSTEM_ESC_CAL_MAX: return "ESC_CAL_MAX";
    case SYSTEM_ESC_CAL_MIN: return "ESC_CAL_MIN";
    case SYSTEM_ESC_CAL_MANUAL: return "ESC_CAL_MANUAL";
    default: return "UNKNOWN";
    }
}

static const char *tv_mode_name(tv_mode_t mode)
{
    switch (mode) {
    case TV_MODE_OFF: return "OFF";
    case TV_MODE_STRAIGHT: return "STRAIGHT";
    case TV_MODE_FULL: return "FULL";
    default: return "UNKNOWN";
    }
}

static const char *calibration_kind_name(powertrain_calibration_kind_t kind)
{
    switch (kind) {
    case POWERTRAIN_CALIBRATION_THROTTLE: return "receiver";
    case POWERTRAIN_CALIBRATION_STEERING: return "steering";
    case POWERTRAIN_CALIBRATION_ARM: return "arm";
    case POWERTRAIN_CALIBRATION_TV: return "tv";
    case POWERTRAIN_CALIBRATION_IMU: return "imu";
    case POWERTRAIN_CALIBRATION_ESC: return "esc";
    default: return "none";
    }
}

static int64_t sample_age_ms(const rc_channel_sample_t *sample, int64_t now_us)
{
    if (!sample->valid || sample->updated_at_us <= 0 || now_us < sample->updated_at_us) {
        return -1;
    }
    return (now_us - sample->updated_at_us) / 1000;
}

static int format_telemetry(char *buffer, size_t buffer_size)
{
    powertrain_remote_snapshot_t snapshot;
    if (!powertrain_get_remote_snapshot(&snapshot)) {
        return -1;
    }
    char vectoring_reason[POWERTRAIN_REMOTE_REASON_MAX * 2];
    json_escape_string(snapshot.vectoring_reason, vectoring_reason,
                       sizeof(vectoring_reason));
    char calibration_prompt[POWERTRAIN_CALIBRATION_PROMPT_MAX * 2];
    char calibration_message[POWERTRAIN_CALIBRATION_MESSAGE_MAX * 2];
    json_escape_string(snapshot.calibration_prompt, calibration_prompt,
                       sizeof(calibration_prompt));
    json_escape_string(snapshot.calibration_message, calibration_message,
                       sizeof(calibration_message));
    dragy_sensor_snapshot_t dragy;
    dragy_sensor_get_snapshot(&dragy);
    telemetry_log_status_t logging;
    telemetry_log_get_status(&logging);
    char dragy_i2c_addresses[DRAGY_I2C_MAX_DEVICES * 5];
    format_i2c_addresses(&dragy, dragy_i2c_addresses,
                         sizeof(dragy_i2c_addresses));
    int64_t dragy_age_ms = dragy.updated_at_us > 0 &&
                           snapshot.captured_at_us >= dragy.updated_at_us
                               ? (snapshot.captured_at_us - dragy.updated_at_us) / 1000
                               : -1;

    int written = snprintf(
        buffer, buffer_size,
        "{\"protocol\":1,\"uptime_ms\":%lld,"
        "\"state\":\"%s\",\"armed\":%s,\"inhibited\":%s,"
        "\"inhibit_flags\":%lu,\"inhibit_count\":%lu,"
        "\"initial_arm_cycle_ready\":%s,\"drive_mode\":\"%s\","
        "\"calibration\":{\"active\":%s,\"sampling\":%s,"
          "\"kind\":\"%s\",\"step\":%u,\"total_steps\":%u,"
          "\"values_us\":[%u,%u,%u],\"prompt\":\"%s\","
          "\"message\":\"%s\"},"
        "\"rc\":{"
          "\"throttle\":{\"valid\":%s,\"us\":%u,\"age_ms\":%lld},"
          "\"steering\":{\"valid\":%s,\"us\":%u,\"age_ms\":%lld},"
          "\"arm\":{\"valid\":%s,\"us\":%u,\"age_ms\":%lld},"
          "\"tv\":{\"valid\":%s,\"us\":%u,\"age_ms\":%lld}},"
        "\"steering\":{\"servo_us\":%u,\"filter_us\":%u,"
          "\"filter_active\":%s,\"rejected_spikes\":%lu,"
          "\"speed_valid\":%s,\"speed_kmh\":%.3f,\"limited\":%s,"
          "\"requested_deg\":%.3f,\"maximum_deg\":%.3f,"
          "\"servo_command_deg\":%.3f,\"left_wheel_deg\":%.3f,"
          "\"right_wheel_deg\":%.3f,\"average_wheel_deg\":%.3f},"
        "\"rpm\":{\"ppr\":%u,\"wheels\":["
          "{\"valid\":%s,\"rpm\":%.2f,\"hz\":%.2f,\"edges\":%lu,\"window_us\":%lu},"
          "{\"valid\":%s,\"rpm\":%.2f,\"hz\":%.2f,\"edges\":%lu,\"window_us\":%lu},"
          "{\"valid\":%s,\"rpm\":%.2f,\"hz\":%.2f,\"edges\":%lu,\"window_us\":%lu},"
          "{\"valid\":%s,\"rpm\":%.2f,\"hz\":%.2f,\"edges\":%lu,\"window_us\":%lu}]},"
        "\"imu\":{\"valid\":%s,\"bias_calibrated\":%s,\"yaw_dps\":%.3f,"
          "\"accel_mps2\":[%.3f,%.3f,%.3f],\"gyro_dps\":[%.3f,%.3f,%.3f]},"
        "\"tv\":{\"configured\":%s,\"requested\":\"%s\","
          "\"active\":%s,\"active_mode\":\"%s\",\"reason\":\"%s\","
          "\"target_yaw_dps\":%.3f,\"yaw_error_dps\":%.3f,"
          "\"side_rpm_error\":%.5f,\"predicted_lateral_mps2\":%.3f,"
          "\"lateral_demand\":%.3f,\"front_relief\":%.4f,"
          "\"correction\":[%.5f,%.5f,%.5f,%.5f]},"
        "\"outputs\":["
          "{\"throttle_us\":%u,\"reverse_us\":%u},"
          "{\"throttle_us\":%u,\"reverse_us\":%u},"
          "{\"throttle_us\":%u,\"reverse_us\":%u},"
          "{\"throttle_us\":%u,\"reverse_us\":%u}],"
        "\"dragy\":{\"initialized\":%s,\"nmea_recent\":%s,\"fix_valid\":%s,"
          "\"age_ms\":%lld,\"baud\":%lu,\"uart_bytes\":%lu,"
          "\"valid_sentences\":%lu,\"checksum_errors\":%lu,\"parse_errors\":%lu,"
          "\"position_valid\":%s,\"latitude_deg\":%.8f,\"longitude_deg\":%.8f,"
          "\"speed_valid\":%s,\"speed_kmh\":%.3f,\"course_valid\":%s,"
          "\"course_deg\":%.3f,\"altitude_valid\":%s,\"altitude_m\":%.3f,"
          "\"satellites\":%u,\"hdop\":%.3f,\"utc_time_ms\":%lu,"
          "\"date_ddmmyy\":%lu,\"i2c_scan_complete\":%s,"
          "\"i2c_addresses\":[%s],\"compass_candidate_present\":%s,"
          "\"compass_data_supported\":false},"
        "\"logging\":{\"initialized\":%s,\"mounted\":%s,"
          "\"manual_control\":true,\"recording\":%s,\"full\":%s,\"faulted\":%s,\"last_errno\":%d,"
          "\"rate_hz\":%lu,\"interval_ms\":%lu,"
          "\"boot_id\":%lu,\"samples\":%lu,\"write_errors\":%lu,"
          "\"bytes\":%llu,\"capacity_bytes\":%llu,\"free_bytes\":%llu},"
        "\"config\":{\"loaded_from_nvs\":%s,\"reverse_limit_percent\":%u,"
          "\"drive_smoothing_percent\":%u,"
          "\"failsafe_enabled\":%s,\"failsafe_us\":%u,\"failsafe_window_us\":%u,"
          "\"motor_poles\":%u,\"rpm_ppr\":%u,\"steering_trim_deg\":%.1f,"
          "\"steering_smoothing_ms\":%u,\"steering_speed_limit\":%s,"
          "\"steering_lateral_g\":%.3f,\"tv_authority_percent\":%u,"
          "\"tv_front_relief_percent\":%u,\"tv_turn_yaw_gain_dps\":%.5f,"
          "\"tv_turn_rpm_gain\":%.6f,\"tv_yaw_kp\":%.7f,"
          "\"tv_yaw_ki\":%.7f,\"tv_rpm_kp\":%.6f,\"imu_yaw_sign\":%d,"
          "\"permanent_arm_latch\":%s,"
          "\"logging_rate_hz\":%u}}",
        (long long)(snapshot.captured_at_us / 1000),
        system_state_name(snapshot.state),
        bool_json(snapshot.state == SYSTEM_DRIVE_ARMED),
        bool_json(snapshot.drive_inhibit_flags != 0),
        (unsigned long)snapshot.drive_inhibit_flags,
        (unsigned long)snapshot.drive_inhibit_count,
        bool_json(snapshot.arm_cycle_ready),
        drivetrain_mode_name(snapshot.config.drivetrain_mode),
        bool_json(snapshot.calibration_active),
        bool_json(snapshot.calibration_sampling),
        calibration_kind_name(snapshot.calibration_kind),
        snapshot.calibration_step, snapshot.calibration_total_steps,
        snapshot.calibration_values_us[0], snapshot.calibration_values_us[1],
        snapshot.calibration_values_us[2], calibration_prompt,
        calibration_message,
        bool_json(snapshot.throttle_input.valid), snapshot.throttle_input.pulse_us,
        (long long)sample_age_ms(&snapshot.throttle_input, snapshot.captured_at_us),
        bool_json(snapshot.steering_input.valid), snapshot.steering_input.pulse_us,
        (long long)sample_age_ms(&snapshot.steering_input, snapshot.captured_at_us),
        bool_json(snapshot.arm_input.valid), snapshot.arm_input.pulse_us,
        (long long)sample_age_ms(&snapshot.arm_input, snapshot.captured_at_us),
        bool_json(snapshot.tv_input.valid), snapshot.tv_input.pulse_us,
        (long long)sample_age_ms(&snapshot.tv_input, snapshot.captured_at_us),
        snapshot.steering_servo_us, snapshot.steering_filter_us,
        bool_json(snapshot.steering_filter_active),
        (unsigned long)snapshot.steering_rejected_spikes,
        bool_json(snapshot.vehicle_speed_valid),
        finite_number(snapshot.vehicle_speed_mps * 3.6f),
        bool_json(snapshot.steering_limited),
        finite_number(snapshot.steering_requested_deg),
        finite_number(snapshot.steering_maximum_deg),
        finite_number(snapshot.steering_servo_command_deg),
        finite_number(snapshot.steering_left_wheel_deg),
        finite_number(snapshot.steering_right_wheel_deg),
        finite_number(snapshot.steering_average_wheel_deg),
        snapshot.rpm.pulses_per_revolution,
        bool_json(snapshot.rpm.valid[0]), finite_number(snapshot.rpm.rpm[0]),
        finite_number(snapshot.rpm.frequency_hz[0]),
        (unsigned long)snapshot.rpm.edge_count[0], (unsigned long)snapshot.rpm.window_us[0],
        bool_json(snapshot.rpm.valid[1]), finite_number(snapshot.rpm.rpm[1]),
        finite_number(snapshot.rpm.frequency_hz[1]),
        (unsigned long)snapshot.rpm.edge_count[1], (unsigned long)snapshot.rpm.window_us[1],
        bool_json(snapshot.rpm.valid[2]), finite_number(snapshot.rpm.rpm[2]),
        finite_number(snapshot.rpm.frequency_hz[2]),
        (unsigned long)snapshot.rpm.edge_count[2], (unsigned long)snapshot.rpm.window_us[2],
        bool_json(snapshot.rpm.valid[3]), finite_number(snapshot.rpm.rpm[3]),
        finite_number(snapshot.rpm.frequency_hz[3]),
        (unsigned long)snapshot.rpm.edge_count[3], (unsigned long)snapshot.rpm.window_us[3],
        bool_json(snapshot.imu.valid), bool_json(snapshot.imu.bias_calibrated),
        finite_number(snapshot.imu.yaw_rate_dps),
        finite_number(snapshot.imu.accel_mps2[0]), finite_number(snapshot.imu.accel_mps2[1]),
        finite_number(snapshot.imu.accel_mps2[2]), finite_number(snapshot.imu.gyro_dps[0]),
        finite_number(snapshot.imu.gyro_dps[1]), finite_number(snapshot.imu.gyro_dps[2]),
        bool_json(snapshot.config.torque_vectoring_enabled),
        tv_mode_name(snapshot.requested_tv_mode), bool_json(snapshot.vectoring_active),
        tv_mode_name(snapshot.active_tv_mode), vectoring_reason,
        finite_number(snapshot.target_yaw_rate_dps), finite_number(snapshot.yaw_error_dps),
        finite_number(snapshot.side_rpm_error),
        finite_number(snapshot.predicted_lateral_accel_mps2),
        finite_number(snapshot.lateral_demand), finite_number(snapshot.front_relief),
        finite_number(snapshot.wheel_correction[0]), finite_number(snapshot.wheel_correction[1]),
        finite_number(snapshot.wheel_correction[2]), finite_number(snapshot.wheel_correction[3]),
        snapshot.outputs.throttle_us[0], snapshot.outputs.reverse_us[0],
        snapshot.outputs.throttle_us[1], snapshot.outputs.reverse_us[1],
        snapshot.outputs.throttle_us[2], snapshot.outputs.reverse_us[2],
        snapshot.outputs.throttle_us[3], snapshot.outputs.reverse_us[3],
        bool_json(dragy.initialized), bool_json(dragy.nmea_recent),
        bool_json(dragy.fix_valid), (long long)dragy_age_ms,
        (unsigned long)dragy.baud_rate, (unsigned long)dragy.uart_bytes,
        (unsigned long)dragy.valid_sentence_count,
        (unsigned long)dragy.checksum_error_count,
        (unsigned long)dragy.parse_error_count,
        bool_json(dragy.position_valid), finite_double(dragy.latitude_deg),
        finite_double(dragy.longitude_deg), bool_json(dragy.speed_valid),
        finite_number(dragy.speed_mps * 3.6f), bool_json(dragy.course_valid),
        finite_number(dragy.course_deg), bool_json(dragy.altitude_valid),
        finite_number(dragy.altitude_m), dragy.satellites,
        finite_number(dragy.hdop), (unsigned long)dragy.utc_time_ms,
        (unsigned long)dragy.date_ddmmyy, bool_json(dragy.i2c_scan_complete),
        dragy_i2c_addresses, bool_json(dragy.compass_candidate_present),
        bool_json(logging.initialized), bool_json(logging.mounted),
        bool_json(logging.recording), bool_json(logging.full),
        bool_json(logging.faulted), (int)logging.last_errno,
        (unsigned long)logging.rate_hz, (unsigned long)logging.interval_ms,
        (unsigned long)logging.boot_id,
        (unsigned long)logging.sample_count,
        (unsigned long)logging.write_error_count,
        (unsigned long long)logging.file_bytes,
        (unsigned long long)logging.capacity_bytes,
        (unsigned long long)logging.free_bytes,
        bool_json(snapshot.config.loaded_from_nvs),
        snapshot.config.reverse_limit_percent,
        snapshot.config.drive_smoothing_percent,
        bool_json(snapshot.config.receiver_failsafe_enabled),
        snapshot.config.receiver_failsafe_us, snapshot.config.receiver_failsafe_window_us,
        snapshot.config.motor_poles, snapshot.config.rpm_pulses_per_revolution,
        (double)snapshot.config.steering_trim_tenths_deg / 10.0,
        snapshot.config.steering_smoothing_ms,
        bool_json(snapshot.config.steering_speed_limit_enabled),
        finite_number(snapshot.config.steering_lateral_accel_g),
        snapshot.config.tv_authority_percent, snapshot.config.tv_front_relief_percent,
        finite_number(snapshot.config.tv_turn_yaw_gain_dps),
        finite_number(snapshot.config.tv_turn_rpm_gain),
        finite_number(snapshot.config.tv_yaw_kp), finite_number(snapshot.config.tv_yaw_ki),
        finite_number(snapshot.config.tv_rpm_kp), snapshot.config.imu_yaw_sign,
        bool_json(snapshot.config.permanent_arm_latch_enabled),
        snapshot.config.telemetry_log_rate_hz);

    if (written <= 0) {
        ESP_LOGE(TAG, "Could not format telemetry JSON");
        return -1;
    }
    if ((size_t)written >= buffer_size) {
        ESP_LOGE(TAG,
                 "Telemetry JSON requires %d bytes including terminator; buffer has %u",
                 written + 1, (unsigned int)buffer_size);
        return -1;
    }
    return written;
}

static bool apply_remote_command(const remote_command_t *command)
{
    switch (command->operation) {
    case REMOTE_DISARM_FOR_CONFIG:
        return powertrain_disarm_for_configuration();
    case REMOTE_CONFIG_DRIVETRAIN:
        return powertrain_set_drivetrain_mode(command->drivetrain_mode);
    case REMOTE_CONFIG_REVERSE_LIMIT:
        return powertrain_set_reverse_limit((uint8_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_DRIVE_SMOOTHING:
        return powertrain_set_drive_smoothing(
            (uint8_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_FAILSAFE:
        return powertrain_set_failsafe((uint16_t)command->unsigned_values[0],
                                       (uint16_t)command->unsigned_values[1]);
    case REMOTE_CONFIG_FAILSAFE_ENABLED:
        return powertrain_set_failsafe_enabled(command->enabled);
    case REMOTE_CONFIG_RPM_POLES:
        return powertrain_set_motor_poles((uint8_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_RPM_PPR:
        return powertrain_set_rpm_pulses_per_revolution(
            (uint16_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_STEERING_TRIM:
        return powertrain_set_steering_trim(command->float_values[0]);
    case REMOTE_CONFIG_STEERING_SMOOTHING:
        return powertrain_set_steering_smoothing(
            (uint16_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_STEERING_SPEED_LIMIT:
        return powertrain_set_steering_speed_limit_enabled(command->enabled);
    case REMOTE_CONFIG_STEERING_LATERAL_G:
        return powertrain_set_steering_lateral_accel(command->float_values[0]);
    case REMOTE_CONFIG_TV_ENABLED:
        return powertrain_set_tv_enabled(command->enabled);
    case REMOTE_CONFIG_TV_AUTHORITY:
        return powertrain_set_tv_authority((uint8_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_TV_FRONT_RELIEF:
        return powertrain_set_tv_front_relief((uint8_t)command->unsigned_values[0]);
    case REMOTE_CONFIG_TV_GAINS:
        return powertrain_set_tv_gains(command->float_values[0], command->float_values[1],
                                       command->float_values[2], command->float_values[3],
                                       command->float_values[4]);
    case REMOTE_CONFIG_IMU_YAW_SIGN:
        return powertrain_set_imu_yaw_sign((int8_t)command->signed_value);
    case REMOTE_CONFIG_ARM_LATCH:
        return powertrain_set_permanent_arm_latch_enabled(command->enabled);
    case REMOTE_CONFIG_LOGGING_RATE:
        return powertrain_set_logging_rate_hz((uint8_t)command->unsigned_values[0]);
    case REMOTE_CALIBRATE_THROTTLE:
        return powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_THROTTLE);
    case REMOTE_CALIBRATE_STEERING:
        return powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_STEERING);
    case REMOTE_CALIBRATE_ARM:
        return powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_ARM);
    case REMOTE_CALIBRATE_TV:
        return powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_TV);
    case REMOTE_CALIBRATE_IMU:
        return powertrain_remote_calibration_start(POWERTRAIN_CALIBRATION_IMU);
    case REMOTE_CALIBRATE_CAPTURE:
        return powertrain_remote_calibration_capture();
    case REMOTE_CALIBRATE_ESC_ARM:
        return powertrain_cal_esc_arm();
    case REMOTE_CALIBRATE_ESC_MAX:
        return powertrain_cal_esc_max();
    case REMOTE_CALIBRATE_ESC_MIN:
        return powertrain_cal_esc_min();
    case REMOTE_CALIBRATE_ESC_MANUAL:
        return powertrain_cal_manual();
    case REMOTE_CALIBRATE_CANCEL:
        return powertrain_cal_cancel();
    default:
        return false;
    }
}

static esp_err_t set_http_common_headers(httpd_req_t *request)
{
    esp_err_t err = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (err == ESP_OK) {
        err = httpd_resp_set_hdr(request, "X-Content-Type-Options", "nosniff");
    }
    return err;
}

static esp_err_t send_json(httpd_req_t *request,
                           const char *status,
                           const char *payload,
                           size_t payload_size)
{
    httpd_resp_set_status(request, status);
    httpd_resp_set_type(request, "application/json; charset=utf-8");
    set_http_common_headers(request);
    return httpd_resp_send(request, payload, (ssize_t)payload_size);
}

static esp_err_t send_json_message(httpd_req_t *request,
                                   const char *status,
                                   bool ok,
                                   const char *message)
{
    char escaped[COMMAND_ERROR_SIZE * 2];
    char response[sizeof(escaped) + 48];
    json_escape_string(message, escaped, sizeof(escaped));
    int written = snprintf(response, sizeof(response),
                           "{\"ok\":%s,\"message\":\"%s\"}",
                           bool_json(ok), escaped);
    if (written < 0 || (size_t)written >= sizeof(response)) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Could not format response");
    }
    return send_json(request, status, response, (size_t)written);
}

static esp_err_t send_static_asset(httpd_req_t *request,
                                   const char *content_type,
                                   const uint8_t *start,
                                   const uint8_t *end)
{
    size_t size = (size_t)(end - start);
    if (size > 0 && start[size - 1] == 0) size--;
    httpd_resp_set_type(request, content_type);
    set_http_common_headers(request);
    return httpd_resp_send(request, (const char *)start, (ssize_t)size);
}

static esp_err_t root_get_handler(httpd_req_t *request)
{
    return send_static_asset(request, "text/html; charset=utf-8",
                             web_index_start, web_index_end);
}

static esp_err_t app_get_handler(httpd_req_t *request)
{
    return send_static_asset(request, "text/javascript; charset=utf-8",
                             web_app_start, web_app_end);
}

static esp_err_t style_get_handler(httpd_req_t *request)
{
    return send_static_asset(request, "text/css; charset=utf-8",
                             web_style_start, web_style_end);
}

static esp_err_t favicon_get_handler(httpd_req_t *request)
{
    httpd_resp_set_status(request, "204 No Content");
    return httpd_resp_send(request, NULL, 0);
}

static esp_err_t state_get_handler(httpd_req_t *request)
{
    int length = format_telemetry(telemetry_buffer, sizeof(telemetry_buffer));
    if (length < 0) {
        return send_json_message(request, "503 Service Unavailable", false,
                                 "Controller telemetry is unavailable");
    }
    return send_json(request, "200 OK", telemetry_buffer, (size_t)length);
}

static int format_config_response(char *buffer, size_t buffer_size)
{
    powertrain_remote_snapshot_t snapshot;
    if (!powertrain_get_remote_snapshot(&snapshot)) return -1;
    telemetry_log_status_t logging;
    telemetry_log_get_status(&logging);
    const drive_config_t *config = &snapshot.config;
    int written = snprintf(
        buffer, buffer_size,
        "{\"ok\":true,\"source\":\"%s\",\"config\":{"
        "\"loaded_from_nvs\":%s,\"reverse_limit_percent\":%u,"
        "\"drive_smoothing_percent\":%u,"
        "\"failsafe_enabled\":%s,\"failsafe_us\":%u,"
        "\"failsafe_window_us\":%u,\"motor_poles\":%u,\"rpm_ppr\":%u,"
        "\"steering_trim_deg\":%.1f,\"steering_smoothing_ms\":%u,"
        "\"steering_speed_limit\":%s,\"steering_lateral_g\":%.3f,"
        "\"tv_authority_percent\":%u,\"tv_front_relief_percent\":%u,"
        "\"tv_turn_yaw_gain_dps\":%.5f,\"tv_turn_rpm_gain\":%.6f,"
        "\"tv_yaw_kp\":%.7f,\"tv_yaw_ki\":%.7f,\"tv_rpm_kp\":%.6f,"
        "\"imu_yaw_sign\":%d,\"permanent_arm_latch\":%s,"
        "\"logging_rate_hz\":%u},\"drive_mode\":\"%s\","
        "\"tv_configured\":%s,\"logging\":{"
        "\"rate_hz\":%lu,\"interval_ms\":%lu,\"samples\":%lu,"
        "\"bytes\":%llu,\"capacity_bytes\":%llu,\"free_bytes\":%llu}}",
        config->loaded_from_nvs ? "nvs" : "compiled_defaults",
        bool_json(config->loaded_from_nvs), config->reverse_limit_percent,
        config->drive_smoothing_percent,
        bool_json(config->receiver_failsafe_enabled), config->receiver_failsafe_us,
        config->receiver_failsafe_window_us, config->motor_poles,
        config->rpm_pulses_per_revolution,
        (double)config->steering_trim_tenths_deg / 10.0,
        config->steering_smoothing_ms,
        bool_json(config->steering_speed_limit_enabled),
        finite_number(config->steering_lateral_accel_g),
        config->tv_authority_percent, config->tv_front_relief_percent,
        finite_number(config->tv_turn_yaw_gain_dps),
        finite_number(config->tv_turn_rpm_gain), finite_number(config->tv_yaw_kp),
        finite_number(config->tv_yaw_ki), finite_number(config->tv_rpm_kp),
        config->imu_yaw_sign, bool_json(config->permanent_arm_latch_enabled),
        config->telemetry_log_rate_hz, drivetrain_mode_name(config->drivetrain_mode),
        bool_json(config->torque_vectoring_enabled),
        (unsigned long)logging.rate_hz, (unsigned long)logging.interval_ms,
        (unsigned long)logging.sample_count,
        (unsigned long long)logging.file_bytes,
        (unsigned long long)logging.capacity_bytes,
        (unsigned long long)logging.free_bytes);
    if (written <= 0 || (size_t)written >= buffer_size) return -1;
    return written;
}

static esp_err_t config_get_handler(httpd_req_t *request)
{
    int length = format_config_response(config_response_buffer,
                                        sizeof(config_response_buffer));
    if (length < 0) {
        return send_json_message(request, "503 Service Unavailable", false,
                                 "Controller configuration is unavailable");
    }
    return send_json(request, "200 OK", config_response_buffer, (size_t)length);
}

static const char *command_result_message(const remote_command_t *command,
                                          bool applied)
{
    bool disarm_request = command->operation == REMOTE_DISARM_FOR_CONFIG;
    bool calibration_request = command->operation >= REMOTE_CALIBRATE_THROTTLE;
    if (applied) {
        if (disarm_request) {
            return "drive disarmed for configuration; physical STOP-to-RUN required";
        }
        if (calibration_request) {
            return "calibration command accepted; follow live calibration status";
        }
        return "configuration saved";
    }
    if (disarm_request) return "configuration disarm rejected during calibration";
    if (calibration_request) {
        return "calibration command rejected; check DISARMED, neutral, CH4 STOP, and current step";
    }
    return "controller rejected command; it must be DISARMED and prerequisites must be valid";
}

static esp_err_t apply_http_command(httpd_req_t *request, const char *command_text)
{
    remote_command_t command;
    char detail[COMMAND_ERROR_SIZE];
    if (!remote_command_parse(command_text, &command, detail, sizeof(detail))) {
        return send_json_message(request, "400 Bad Request", false, detail);
    }
    bool applied = apply_remote_command(&command);
    return send_json_message(request, applied ? "200 OK" : "409 Conflict",
                             applied, command_result_message(&command, applied));
}

static bool parse_command_body(const char *body,
                               size_t body_length,
                               char *command,
                               size_t command_size)
{
    const char *cursor = body;
    const char *limit = body + body_length;
    while (cursor < limit && isspace((unsigned char)*cursor)) cursor++;
    if (cursor >= limit || *cursor++ != '{') return false;
    while (cursor < limit && isspace((unsigned char)*cursor)) cursor++;
    static const char key[] = "\"command\"";
    if ((size_t)(limit - cursor) < sizeof(key) - 1 ||
        memcmp(cursor, key, sizeof(key) - 1) != 0) {
        return false;
    }
    cursor += sizeof(key) - 1;
    while (cursor < limit && isspace((unsigned char)*cursor)) cursor++;
    if (cursor >= limit || *cursor++ != ':') return false;
    while (cursor < limit && isspace((unsigned char)*cursor)) cursor++;
    if (cursor >= limit || *cursor++ != '\"') return false;

    size_t output = 0;
    while (cursor < limit && *cursor != '\"') {
        unsigned char value = (unsigned char)*cursor++;
        if (value == '\\' || value < 0x20 || output + 1 >= command_size) {
            return false;
        }
        command[output++] = (char)value;
    }
    if (cursor >= limit || *cursor++ != '\"') return false;
    command[output] = '\0';
    while (cursor < limit && isspace((unsigned char)*cursor)) cursor++;
    if (cursor >= limit || *cursor++ != '}') return false;
    while (cursor < limit && isspace((unsigned char)*cursor)) cursor++;
    return cursor == limit && output > 0;
}

static esp_err_t command_post_handler(httpd_req_t *request)
{
    if (request->content_len <= 0 || request->content_len >= HTTP_COMMAND_BODY_SIZE) {
        return send_json_message(request, "413 Payload Too Large", false,
                                 "Expected a small JSON command object");
    }
    char body[HTTP_COMMAND_BODY_SIZE];
    size_t received = 0;
    while (received < (size_t)request->content_len) {
        int result = httpd_req_recv(request, body + received,
                                    request->content_len - received);
        if (result == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (result <= 0) {
            return send_json_message(request, "400 Bad Request", false,
                                     "Could not read JSON request body");
        }
        received += (size_t)result;
    }
    body[received] = '\0';

    char command[256];
    if (!parse_command_body(body, received, command, sizeof(command))) {
        return send_json_message(request, "400 Bad Request", false,
                                 "Expected JSON object containing a plain command string");
    }

    char *start = command;
    while (*start != '\0' && isspace((unsigned char)*start)) start++;
    char *end = start + strlen(start);
    while (end > start && isspace((unsigned char)end[-1])) *--end = '\0';
    for (char *cursor = start; *cursor != '\0'; cursor++) {
        *cursor = (char)tolower((unsigned char)*cursor);
    }
    return apply_http_command(request, start);
}

static esp_err_t disarm_post_handler(httpd_req_t *request)
{
    return apply_http_command(request, "disarm config");
}

static esp_err_t log_get_handler(httpd_req_t *request)
{
    telemetry_log_status_t status;
    telemetry_log_get_status(&status);
    if (status.sample_count == 0) {
        const char *message = status.full && status.capacity_bytes > 0 &&
                                      status.free_bytes <= LOG_FREE_RESERVE_BYTES
                                  ? "The device log has no telemetry samples. The dedicated log storage reports no usable free space; while DISARMED, use Recover log storage to rebuild it."
                                  : "The device log has not recorded a telemetry sample yet.";
        return send_json_message(request, "409 Conflict", false, message);
    }

    size_t bytes_read = 0;
    uint32_t total = 0;
    esp_err_t err = telemetry_log_read(0, NULL, 0, &bytes_read, &total);
    if (err != ESP_OK) {
        return send_json_message(request, "503 Service Unavailable", false,
                                 "Device log is unavailable");
    }
    httpd_resp_set_type(request, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(request, "Content-Disposition",
                       "attachment; filename=rc_car_telemetry.csv");
    set_http_common_headers(request);
    uint32_t offset = 0;
    while (offset < total) {
        size_t requested = total - offset;
        if (requested > sizeof(log_export_chunk)) requested = sizeof(log_export_chunk);
        uint32_t current_size = 0;
        err = telemetry_log_read(offset, log_export_chunk, requested,
                                 &bytes_read, &current_size);
        if (err != ESP_OK || bytes_read == 0) {
            ESP_LOGE(TAG, "CSV HTTP export failed at byte %lu: %s",
                     (unsigned long)offset, esp_err_to_name(err));
            return ESP_FAIL;
        }
        if (httpd_resp_send_chunk(request, (const char *)log_export_chunk,
                                  (ssize_t)bytes_read) != ESP_OK) {
            return ESP_FAIL;
        }
        offset += (uint32_t)bytes_read;
    }
    return httpd_resp_send_chunk(request, NULL, 0);
}

static esp_err_t log_clear_post_handler(httpd_req_t *request)
{
    powertrain_remote_snapshot_t snapshot;
    bool disarmed = powertrain_get_remote_snapshot(&snapshot) &&
                    snapshot.state == SYSTEM_DISARMED;
    esp_err_t err = disarmed ? telemetry_log_clear() : ESP_ERR_INVALID_STATE;
    return send_json_message(
        request, err == ESP_OK ? "200 OK" : "409 Conflict", err == ESP_OK,
        err == ESP_OK ? "CSV log cleared. Recording is stopped; use Start recording when ready."
                      : (disarmed ? "Log storage could not be cleared; check storage status."
                                  : "log clear requires the car to be DISARMED"));
}

static esp_err_t log_recording_post_handler(httpd_req_t *request)
{
    bool recording = strcmp(request->uri, "/api/log/start") == 0;
    esp_err_t err = telemetry_log_set_recording(recording);
    return send_json_message(
        request, err == ESP_OK ? "200 OK" : "409 Conflict", err == ESP_OK,
        err == ESP_OK ? (recording ? "Recording started; appending to the device CSV."
                                  : "Recording stopped. Recorded samples are saved.")
                      : (recording ? "Cannot start recording: storage is unavailable, full, or faulted. Export before clearing."
                                   : "Recording could not be confirmed saved; check storage status and export readable data."));
}

static esp_err_t register_http_handlers(httpd_handle_t server)
{
    const httpd_uri_t handlers[] = {
        {.uri = "/", .method = HTTP_GET, .handler = root_get_handler},
        {.uri = "/index.html", .method = HTTP_GET, .handler = root_get_handler},
        {.uri = "/app.js", .method = HTTP_GET, .handler = app_get_handler},
        {.uri = "/style.css", .method = HTTP_GET, .handler = style_get_handler},
        {.uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get_handler},
        {.uri = "/api/state", .method = HTTP_GET, .handler = state_get_handler},
        {.uri = "/api/config", .method = HTTP_GET, .handler = config_get_handler},
        {.uri = "/api/config", .method = HTTP_POST, .handler = command_post_handler},
        {.uri = "/api/calibration", .method = HTTP_POST, .handler = command_post_handler},
        {.uri = "/api/disarm", .method = HTTP_POST, .handler = disarm_post_handler},
        {.uri = "/api/log.csv", .method = HTTP_GET, .handler = log_get_handler},
        {.uri = "/api/log/clear", .method = HTTP_POST, .handler = log_clear_post_handler},
        {.uri = "/api/log/start", .method = HTTP_POST, .handler = log_recording_post_handler},
        {.uri = "/api/log/stop", .method = HTTP_POST, .handler = log_recording_post_handler},
    };
    for (size_t index = 0; index < sizeof(handlers) / sizeof(handlers[0]); index++) {
        esp_err_t err = httpd_register_uri_handler(server, &handlers[index]);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_RC_WIFI_HTTP_PORT;
    config.task_priority = 2;
    config.stack_size = 8192;
    config.max_uri_handlers = 16;
    config.lru_purge_enable = true;
    esp_err_t err = httpd_start(&http_server, &config);
    if (err != ESP_OK) return err;
    err = register_http_handlers(http_server);
    if (err != ESP_OK) {
        httpd_stop(http_server);
        http_server = NULL;
    }
    return err;
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    (void)arg;
    (void)event_data;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Viewer device connected to the RC-car access point");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "Viewer device disconnected from the RC-car access point");
    }
}

esp_err_t wifi_control_start(void)
{
    size_t ssid_length = strlen(CONFIG_RC_WIFI_AP_SSID);
    size_t password_length = strlen(CONFIG_RC_WIFI_AP_PASSWORD);
    if (ssid_length == 0 || ssid_length > 32 ||
        password_length < 8 || password_length > 63) {
        ESP_LOGE(TAG, "SoftAP SSID must be 1-32 bytes and WPA2 password 8-63 bytes");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK) return err;
    if (esp_netif_create_default_wifi_ap() == NULL) return ESP_ERR_NO_MEM;

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_config);
    if (err == ESP_OK) {
        err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                         wifi_event_handler, NULL);
    }
    wifi_config_t wifi_config = {0};
    memcpy(wifi_config.ap.ssid, CONFIG_RC_WIFI_AP_SSID, ssid_length);
    wifi_config.ap.ssid_len = (uint8_t)ssid_length;
    memcpy(wifi_config.ap.password, CONFIG_RC_WIFI_AP_PASSWORD, password_length);
    wifi_config.ap.channel = CONFIG_RC_WIFI_AP_CHANNEL;
    wifi_config.ap.max_connection = CONFIG_RC_WIFI_AP_MAX_CLIENTS;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    if (err == ESP_OK) err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err == ESP_OK) err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (err == ESP_OK) err = esp_wifi_start();
    if (err == ESP_OK) err = start_http_server();
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Access point %s ready on channel %d",
             CONFIG_RC_WIFI_AP_SSID, CONFIG_RC_WIFI_AP_CHANNEL);
    if (CONFIG_RC_WIFI_HTTP_PORT == 80) {
        ESP_LOGI(TAG, "Viewer hosted at http://%s", AP_IPV4_ADDRESS);
    } else {
        ESP_LOGI(TAG, "Viewer hosted at http://%s:%d", AP_IPV4_ADDRESS,
                 CONFIG_RC_WIFI_HTTP_PORT);
    }
    return ESP_OK;
}

#else

#include "esp_log.h"

esp_err_t wifi_control_start(void)
{
    ESP_LOGI("wifi_control", "Wi-Fi access point disabled in project configuration");
    return ESP_OK;
}

#endif
