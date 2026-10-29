#include "wifi_control.h"

#include "sdkconfig.h"

#if CONFIG_RC_WIFI_CONTROL_ENABLED

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "drivetrain_control.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "independent_steering.h"
#include "powertrain_controller.h"
#include "remote_command.h"

#define TELEMETRY_BUFFER_SIZE 8192
#define CONFIG_RESPONSE_BUFFER_SIZE 3072
#define COMMAND_ERROR_SIZE 192
#define HTTP_COMMAND_BODY_SIZE 512
#define AP_IPV4_ADDRESS "192.168.4.1"

static const char *TAG = "wifi_control";
static httpd_handle_t http_server;
/* Keep the JSON response buffers out of the HTTP task's stack. */
static char telemetry_buffer[TELEMETRY_BUFFER_SIZE];
static char config_response_buffer[CONFIG_RESPONSE_BUFFER_SIZE];

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

static const char *calibration_kind_name(powertrain_calibration_kind_t kind)
{
    switch (kind) {
    case POWERTRAIN_CALIBRATION_THROTTLE: return "receiver";
    case POWERTRAIN_CALIBRATION_STEERING: return "steering";
    case POWERTRAIN_CALIBRATION_ARM: return "arm";
    case POWERTRAIN_CALIBRATION_ESC: return "esc";
    case POWERTRAIN_CALIBRATION_WHEEL_LEFT: return "wheel-left";
    case POWERTRAIN_CALIBRATION_WHEEL_RIGHT: return "wheel-right";
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

typedef struct {
    char *data;
    size_t size;
    size_t used;
    bool failed;
} json_builder_t;

static void json_append(json_builder_t *json, const char *format, ...)
    __attribute__((format(printf, 2, 3)));

static void json_append(json_builder_t *json, const char *format, ...)
{
    if (json->failed || json->used >= json->size) { json->failed = true; return; }
    va_list args;
    va_start(args, format);
    int length = vsnprintf(json->data + json->used, json->size - json->used, format, args);
    va_end(args);
    if (length < 0 || (size_t)length >= json->size - json->used) json->failed = true;
    else json->used += (size_t)length;
}

static void append_config(json_builder_t *json, const powertrain_remote_snapshot_t *snapshot)
{
    const drive_config_t *c = &snapshot->config;
    int16_t toe_min = 0, toe_max = 0;
    independent_steering_toe_range(&c->steering, &toe_min, &toe_max);
    json_append(json, "{\"loaded_from_nvs\":%s,\"reverse_limit_percent\":%u,"
                "\"tank_limit_percent\":%u,\"steering_toe_deg\":%.1f,"
                "\"steering_toe_range\":{\"min_deg\":%.1f,\"max_deg\":%.1f},"
                "\"drive_smoothing_percent\":%u,\"failsafe_enabled\":%s,\"failsafe_us\":%u,"
                "\"failsafe_window_us\":%u,\"motor_poles\":%u,\"rpm_ppr\":%u,"
                "\"steering_smoothing_ms\":%u,\"permanent_arm_latch\":%s,"
                "\"servos\":[",
                bool_json(c->loaded_from_nvs), c->reverse_limit_percent,
                c->tank_limit_percent, c->steering.toe_tenths_deg / 10.0,
                toe_min / 10.0, toe_max / 10.0, c->drive_smoothing_percent,
                bool_json(c->receiver_failsafe_enabled), c->receiver_failsafe_us,
                c->receiver_failsafe_window_us, c->motor_poles, c->rpm_pulses_per_revolution,
                c->steering_smoothing_ms, bool_json(c->permanent_arm_latch_enabled));
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        const steering_servo_config_t *servo = &c->steering.servo[i];
        json_append(json, "%s{\"left_us\":%u,\"center_us\":%u,\"right_us\":%u,\"trim_us\":%d,\"left_angle_deg\":%.1f,\"right_angle_deg\":%.1f,\"wheel_points\":[",
                    i ? "," : "", servo->left_us, servo->center_us, servo->right_us,
                    (int)c->steering.trim_us[i],
                    servo->left_tenths_deg / 10.0, servo->right_tenths_deg / 10.0);
        const wheel_angle_calibration_t *cal = &c->steering.wheel_calibration[i];
        for (uint16_t p = 0; p < cal->count && p < WHEEL_CALIBRATION_MAX_POINTS; p++) {
            json_append(json, "%s{\"pwm_us\":%u,\"angle_deg\":%.1f}", p ? "," : "",
                        cal->points[p].pulse_us, cal->points[p].angle_tenths_deg / 10.0);
        }
        json_append(json, "]}");
    }
    const steering_geometry_t *g = &c->steering.geometry;
    const steering_linkage_limits_t *linkage = &c->steering.linkage;
    const steering_calibration_t *input = &snapshot->steering_calibration;
    json_append(json, "],\"steering_geometry\":{\"wheelbase_mm\":%.1f,\"track_width_mm\":%.1f,"
                "\"wheel_diameter_mm\":%.1f,\"max_left_deg\":%.1f,\"max_right_deg\":%.1f},"
                "\"steering_linkage\":{\"inside_deg\":%.1f,\"outside_deg\":%.1f},"
                "\"steering_input\":{\"left_us\":%u,\"center_us\":%u,\"right_us\":%u,\"deadband_us\":%u},"
                "\"ackermann_strength_points\":[",
                g->wheelbase_tenths_mm / 10.0, g->track_tenths_mm / 10.0,
                g->wheel_diameter_tenths_mm / 10.0, g->max_left_tenths_deg / 10.0,
                g->max_right_tenths_deg / 10.0,
                linkage->inside_tenths_deg / 10.0, linkage->outside_tenths_deg / 10.0,
                input->left_us, input->center_us, input->right_us, input->deadband_us);
    for (uint16_t i = 0; i < c->steering.point_count && i < ACKERMANN_MAX_POINTS; i++) {
        json_append(json, "%s{\"speed_kmh\":%.1f,\"percent\":%.1f}", i ? "," : "",
                    c->steering.points[i].speed_tenths_kmh / 10.0,
                    c->steering.points[i].strength_tenths_percent / 10.0);
    }
    json_append(json, "],\"steering_limit_points\":[");
    for (uint16_t i = 0; i < c->steering_limit.point_count && i < STEERING_LIMIT_MAX_POINTS; i++) {
        json_append(json, "%s{\"speed_kmh\":%.1f,\"percent\":%.1f}", i ? "," : "",
                    c->steering_limit.points[i].speed_tenths_kmh / 10.0,
                    c->steering_limit.points[i].limit_tenths_percent / 10.0);
    }
    json_append(json, "]}");
}

static int format_telemetry(char *buffer, size_t buffer_size)
{
    powertrain_remote_snapshot_t snapshot;
    if (!powertrain_get_remote_snapshot(&snapshot)) return -1;
    char prompt[POWERTRAIN_CALIBRATION_PROMPT_MAX * 2];
    char message[POWERTRAIN_CALIBRATION_MESSAGE_MAX * 2];
    json_escape_string(snapshot.calibration_prompt, prompt, sizeof(prompt));
    json_escape_string(snapshot.calibration_message, message, sizeof(message));
    json_builder_t json = {.data = buffer, .size = buffer_size};
    json_append(&json, "{\"protocol\":3,\"uptime_ms\":%lld,\"state\":\"%s\",\"armed\":%s,"
                "\"inhibited\":%s,\"inhibit_flags\":%lu,\"inhibit_count\":%lu,"
                "\"initial_arm_cycle_ready\":%s,\"drive_mode\":\"%s\","
                "\"calibration\":{\"active\":%s,\"sampling\":%s,\"kind\":\"%s\","
                "\"step\":%u,\"total_steps\":%u,\"values_us\":[%u,%u,%u],\"wheel_capture_sequence\":%lu,\"prompt\":\"%s\",\"message\":\"%s\"},\"rc\":{",
                (long long)(snapshot.captured_at_us / 1000), system_state_name(snapshot.state),
                bool_json(snapshot.state == SYSTEM_DRIVE_ARMED), bool_json(snapshot.drive_inhibit_flags != 0),
                (unsigned long)snapshot.drive_inhibit_flags, (unsigned long)snapshot.drive_inhibit_count,
                bool_json(snapshot.arm_cycle_ready), drivetrain_mode_name(snapshot.config.drivetrain_mode),
                bool_json(snapshot.calibration_active), bool_json(snapshot.calibration_sampling),
                calibration_kind_name(snapshot.calibration_kind), snapshot.calibration_step,
                snapshot.calibration_total_steps, snapshot.calibration_values_us[0],
                snapshot.calibration_values_us[1], snapshot.calibration_values_us[2],
                (unsigned long)snapshot.wheel_capture_sequence, prompt, message);
    const rc_channel_sample_t *channels[] = {&snapshot.throttle_input, &snapshot.steering_input, &snapshot.arm_input};
    const char *names[] = {"throttle", "steering", "arm"};
    for (size_t i = 0; i < 3; i++) {
        json_append(&json, "%s\"%s\":{\"valid\":%s,\"us\":%u,\"age_ms\":%lld}", i ? "," : "",
                    names[i], bool_json(channels[i]->valid), channels[i]->pulse_us,
                    (long long)sample_age_ms(channels[i], snapshot.captured_at_us));
    }
    json_append(&json, "},\"steering\":{\"left_us\":%u,\"right_us\":%u,\"filter_us\":%u,"
                "\"filter_active\":%s,\"rejected_spikes\":%lu,\"speed_valid\":%s,\"speed_held\":%s,"
                "\"speed_kmh\":%.3f,\"command\":%.4f,\"limited_command\":%.4f,\"limit_percent\":%.2f,"
                "\"ackermann_strength_percent\":%.2f,\"left_angle_deg\":%.3f,\"right_angle_deg\":%.3f},"
                "\"rpm\":{\"ppr\":%u,\"wheels\":[",
                snapshot.steering_servo_us[0], snapshot.steering_servo_us[1], snapshot.steering_filter_us,
                bool_json(snapshot.steering_filter_active), (unsigned long)snapshot.steering_rejected_spikes,
                bool_json(snapshot.vehicle_speed_valid), bool_json(snapshot.vehicle_speed_held),
                finite_number(snapshot.vehicle_speed_mps * 3.6f), finite_number(snapshot.steering_command),
                finite_number(snapshot.steering_limited_command), finite_number(snapshot.steering_limit_percent),
                finite_number(snapshot.ackermann_strength_percent), finite_number(snapshot.steering_angle_deg[0]),
                finite_number(snapshot.steering_angle_deg[1]), snapshot.rpm.pulses_per_revolution);
    for (size_t i = 0; i < REAR_RPM_COUNT; i++) {
        json_append(&json, "%s{\"wheel\":\"%s\",\"valid\":%s,\"rpm\":%.2f,\"hz\":%.2f,\"edges\":%lu,\"window_us\":%lu}",
                    i ? "," : "", i == RPM_REAR_LEFT ? "RL" : "RR", bool_json(snapshot.rpm.valid[i]),
                    finite_number(snapshot.rpm.rpm[i]), finite_number(snapshot.rpm.frequency_hz[i]),
                    (unsigned long)snapshot.rpm.edge_count[i], (unsigned long)snapshot.rpm.window_us[i]);
    }
    json_append(&json, "]},\"outputs\":[");
    for (size_t i = 0; i < POWERTRAIN_WHEEL_COUNT; i++) {
        json_append(&json, "%s{\"throttle_us\":%u,\"reverse_us\":%u}", i ? "," : "",
                    snapshot.outputs.throttle_us[i], snapshot.outputs.reverse_us[i]);
    }
    json_append(&json, "],\"config\":");
    append_config(&json, &snapshot);
    json_append(&json, "}");
    if (json.failed) ESP_LOGE(TAG, "Telemetry JSON exceeded buffer or formatting failed");
    return json.failed ? -1 : (int)json.used;
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
    json_builder_t json = {.data = buffer, .size = buffer_size};
    json_append(&json, "{\"ok\":true,\"source\":\"%s\",\"drive_mode\":\"%s\",\"config\":",
                snapshot.config.loaded_from_nvs ? "nvs" : "compiled_defaults",
                drivetrain_mode_name(snapshot.config.drivetrain_mode));
    append_config(&json, &snapshot);
    json_append(&json, "}");
    return json.failed ? -1 : (int)json.used;
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
    if (!applied && command->operation == REMOTE_CONFIG_WHEEL_CALIBRATION)
        return "wheel calibration rejected: finish measurement mode, verify monotonic points, 0 degrees at trimmed neutral, steering maxima and toe travel";
    if (!applied && (command->operation == REMOTE_CONFIG_STEERING_TRIM || command->operation == REMOTE_CONFIG_STEERING_SERVO))
        return "servo change rejected: check DISARMED, endpoints, trim and toe; clear this wheel's measured calibration before changing its neutral or endpoints";
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
    bool applied = powertrain_apply_command(&command);
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

static esp_err_t backup_get_handler(httpd_req_t *request)
{
    uint8_t data[CONFIG_BACKUP_SIZE];
    if (!powertrain_export_configuration(data, sizeof(data))) {
        return send_json_message(request, "409 Conflict", false,
            "Disarm and finish calibration before exporting saved configuration");
    }
    httpd_resp_set_type(request, "application/octet-stream");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    httpd_resp_set_hdr(request, "Content-Disposition", "attachment; filename=rc-car-configuration.rcconfig");
    return httpd_resp_send(request, (const char *)data, sizeof(data));
}

static bool receive_backup(httpd_req_t *request, uint8_t data[CONFIG_BACKUP_SIZE])
{
    if (request->content_len != CONFIG_BACKUP_SIZE) return false;
    size_t received = 0;
    unsigned timeouts = 0;
    while (received < CONFIG_BACKUP_SIZE) {
        int result = httpd_req_recv(request, (char *)data + received, CONFIG_BACKUP_SIZE - received);
        if (result == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts <= 2) continue;
        if (result <= 0) return false;
        received += (size_t)result;
    }
    return true;
}

static esp_err_t backup_preview_handler(httpd_req_t *request)
{
    uint8_t data[CONFIG_BACKUP_SIZE];
    configuration_bundle_t bundle;
    if (!receive_backup(request, data) || !config_backup_decode(data, sizeof(data), &bundle)) {
        return send_json_message(request, "400 Bad Request", false,
            "Invalid, damaged or unsupported configuration backup; nothing changed");
    }
    char response[448];
    int length = snprintf(response, sizeof(response),
        "{\"ok\":true,\"drive_mode\":\"%s\",\"throttle_calibrated\":%s,\"steering_calibrated\":%s,"
        "\"arm_calibrated\":%s,\"left_wheel_points\":%u,\"right_wheel_points\":%u,"
        "\"ackermann_points\":%u,\"steering_limit_points\":%u}",
        drivetrain_mode_name(bundle.drive.drivetrain_mode), bool_json(bundle.throttle.loaded_from_nvs),
        bool_json(bundle.input.loaded_from_nvs), bool_json(bundle.arm.loaded_from_nvs),
        bundle.drive.steering.wheel_calibration[0].count, bundle.drive.steering.wheel_calibration[1].count,
        bundle.drive.steering.point_count, bundle.drive.steering_limit.point_count);
    if (length < 0 || (size_t)length >= sizeof(response)) return ESP_FAIL;
    return send_json(request, "200 OK", response, length);
}

static esp_err_t backup_import_handler(httpd_req_t *request)
{
    uint8_t data[CONFIG_BACKUP_SIZE];
    configuration_bundle_t bundle;
    if (!receive_backup(request, data) || !config_backup_decode(data, sizeof(data), &bundle)) {
        return send_json_message(request, "400 Bad Request", false,
            "Invalid, damaged or unsupported configuration backup; nothing changed");
    }
    bool imported = powertrain_import_configuration(data, sizeof(data));
    return send_json_message(request, imported ? "200 OK" : "409 Conflict", imported,
        imported ? "Complete configuration saved. Physical CH4 STOP-to-RUN cycle required."
                 : "Import not applied: stay DISARMED outside calibration and check NVS storage");
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
        {.uri = "/api/config/backup", .method = HTTP_GET, .handler = backup_get_handler},
        {.uri = "/api/config/preview", .method = HTTP_POST, .handler = backup_preview_handler},
        {.uri = "/api/config/import", .method = HTTP_POST, .handler = backup_import_handler},
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
