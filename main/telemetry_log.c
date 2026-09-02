#include "telemetry_log.h"

#include "sdkconfig.h"

#if CONFIG_RC_TELEMETRY_LOG_ENABLED

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "dragy_sensor.h"
#include "default_config.h"
#include "drivetrain_control.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "powertrain_controller.h"
#include "wear_levelling.h"

#define LOG_BASE_PATH "/telemetry"
#define LOG_PARTITION_LABEL "logdata"
#define LOG_FILE_PATH LOG_BASE_PATH "/telemetry.csv"
#define LOG_TASK_STACK_BYTES 8192
#define LOG_TASK_PRIORITY 1
#define LOG_ROW_BUFFER_SIZE 3072
#define LOG_STDIO_BUFFER_SIZE 4096
#define LOG_FLUSH_INTERVAL_MS 1000
#define LOG_FREE_RESERVE_BYTES (16U * 1024U)

static const char *TAG = "telemetry_log";
static const char CSV_HEADER[] =
    "schema,boot_id,sample_seq,uptime_ms,state,armed,inhibit_flags,inhibit_count,arm_cycle_ready,drive_mode,"
    "rc_throttle_valid,rc_throttle_us,rc_throttle_age_ms,rc_steering_valid,rc_steering_us,rc_steering_age_ms,"
    "rc_arm_valid,rc_arm_us,rc_arm_age_ms,rc_tv_valid,rc_tv_us,rc_tv_age_ms,"
    "steering_servo_us,steering_filter_us,steering_filter_active,steering_rejected_spikes,speed_valid,vehicle_speed_kmh,"
    "steering_limited,steering_requested_deg,steering_max_deg,steering_servo_deg,wheel_left_deg,wheel_right_deg,wheel_average_deg,"
    "rpm_ppr,rpm_fl_valid,rpm_fl,rpm_fl_hz,rpm_fl_edges,rpm_fl_window_us,rpm_fr_valid,rpm_fr,rpm_fr_hz,rpm_fr_edges,rpm_fr_window_us,"
    "rpm_rl_valid,rpm_rl,rpm_rl_hz,rpm_rl_edges,rpm_rl_window_us,rpm_rr_valid,rpm_rr,rpm_rr_hz,rpm_rr_edges,rpm_rr_window_us,"
    "imu_valid,imu_bias_calibrated,imu_yaw_dps,imu_ax_mps2,imu_ay_mps2,imu_az_mps2,imu_gx_dps,imu_gy_dps,imu_gz_dps,"
    "tv_configured,tv_requested,tv_active,tv_active_mode,tv_target_yaw_dps,tv_yaw_error_dps,tv_side_rpm_error,"
    "tv_predicted_lateral_mps2,tv_lateral_demand,tv_front_relief,tv_corr_fl,tv_corr_fr,tv_corr_rl,tv_corr_rr,tv_reason,"
    "out_fl_throttle_us,out_fl_reverse_us,out_fr_throttle_us,out_fr_reverse_us,out_rl_throttle_us,out_rl_reverse_us,out_rr_throttle_us,out_rr_reverse_us,"
    "gps_initialized,gps_nmea_recent,gps_fix_valid,gps_age_ms,gps_baud,gps_uart_bytes,gps_valid_sentences,gps_checksum_errors,gps_parse_errors,"
    "gps_position_valid,gps_latitude_deg,gps_longitude_deg,gps_speed_valid,gps_speed_kmh,gps_course_valid,gps_course_deg,"
    "gps_altitude_valid,gps_altitude_m,gps_satellites,gps_hdop,gps_utc_time_ms,gps_date_ddmmyy,"
    "cfg_reverse_limit_percent,cfg_failsafe_enabled,cfg_failsafe_us,cfg_failsafe_window_us,cfg_motor_poles,cfg_rpm_ppr,"
    "cfg_steering_trim_deg,cfg_steering_smoothing_ms,cfg_steering_speed_limit,cfg_steering_lateral_g,cfg_tv_authority_percent,"
    "cfg_tv_front_relief_percent,cfg_tv_turn_yaw_gain_dps,cfg_tv_turn_rpm_gain,cfg_tv_yaw_kp,cfg_tv_yaw_ki,cfg_tv_rpm_kp,cfg_imu_yaw_sign\n";

typedef struct {
    char *data;
    size_t size;
    size_t used;
    bool overflowed;
} csv_builder_t;

static SemaphoreHandle_t log_mutex;
static FILE *log_file;
static wl_handle_t wl_handle = WL_INVALID_HANDLE;
static TaskHandle_t log_task_handle;
static char log_stdio_buffer[LOG_STDIO_BUFFER_SIZE];
static telemetry_log_status_t log_status;

static const char *state_name(system_state_t state)
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

static const char *tv_name(tv_mode_t mode)
{
    switch (mode) {
    case TV_MODE_OFF: return "OFF";
    case TV_MODE_STRAIGHT: return "STRAIGHT";
    case TV_MODE_FULL: return "FULL";
    default: return "UNKNOWN";
    }
}

static int64_t sample_age_ms(const rc_channel_sample_t *sample, int64_t now_us)
{
    if (!sample->valid || sample->updated_at_us <= 0 || now_us < sample->updated_at_us) {
        return -1;
    }
    return (now_us - sample->updated_at_us) / 1000;
}

static double finite_float(float value)
{
    return isfinite(value) ? (double)value : 0.0;
}

static double finite_f64(double value)
{
    return isfinite(value) ? value : 0.0;
}

static void sanitize_csv_text(const char *source, char *destination, size_t size)
{
    if (size == 0) return;
    size_t output = 0;
    for (size_t input = 0; source[input] != '\0' && output + 1 < size; input++) {
        char value = source[input];
        destination[output++] = value == ',' || value == '\r' || value == '\n' || value == '"'
                                    ? ';'
                                    : value;
    }
    destination[output] = '\0';
}

static void appendf(csv_builder_t *builder, const char *format, ...)
{
    if (builder->overflowed || builder->used >= builder->size) return;
    va_list args;
    va_start(args, format);
    int written = vsnprintf(builder->data + builder->used,
                            builder->size - builder->used, format, args);
    va_end(args);
    if (written < 0 || (size_t)written >= builder->size - builder->used) {
        builder->overflowed = true;
        return;
    }
    builder->used += (size_t)written;
}

static bool format_row(char *buffer,
                       size_t size,
                       uint32_t boot_id,
                       uint32_t sample_sequence)
{
    powertrain_remote_snapshot_t snapshot;
    if (!powertrain_get_remote_snapshot(&snapshot)) return false;
    dragy_sensor_snapshot_t dragy = {0};
    dragy_sensor_get_snapshot(&dragy);
    char reason[POWERTRAIN_REMOTE_REASON_MAX];
    sanitize_csv_text(snapshot.vectoring_reason, reason, sizeof(reason));
    int64_t gps_age_ms = dragy.updated_at_us > 0 &&
                         snapshot.captured_at_us >= dragy.updated_at_us
                             ? (snapshot.captured_at_us - dragy.updated_at_us) / 1000
                             : -1;

    csv_builder_t row = {.data = buffer, .size = size};
    appendf(&row, "1,%lu,%lu,%lld,%s,%u,%lu,%lu,%u,%s,",
            (unsigned long)boot_id, (unsigned long)sample_sequence,
            (long long)(snapshot.captured_at_us / 1000), state_name(snapshot.state),
            snapshot.state == SYSTEM_DRIVE_ARMED, (unsigned long)snapshot.drive_inhibit_flags,
            (unsigned long)snapshot.drive_inhibit_count, snapshot.arm_cycle_ready,
            drivetrain_mode_name(snapshot.config.drivetrain_mode));
    const rc_channel_sample_t *channels[] = {
        &snapshot.throttle_input, &snapshot.steering_input,
        &snapshot.arm_input, &snapshot.tv_input,
    };
    for (size_t index = 0; index < 4; index++) {
        appendf(&row, "%u,%u,%lld,", channels[index]->valid,
                channels[index]->pulse_us,
                (long long)sample_age_ms(channels[index], snapshot.captured_at_us));
    }
    appendf(&row, "%u,%u,%u,%lu,%u,%.3f,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
            snapshot.steering_servo_us, snapshot.steering_filter_us,
            snapshot.steering_filter_active,
            (unsigned long)snapshot.steering_rejected_spikes,
            snapshot.vehicle_speed_valid, finite_float(snapshot.vehicle_speed_mps * 3.6f),
            snapshot.steering_limited, finite_float(snapshot.steering_requested_deg),
            finite_float(snapshot.steering_maximum_deg),
            finite_float(snapshot.steering_servo_command_deg),
            finite_float(snapshot.steering_left_wheel_deg),
            finite_float(snapshot.steering_right_wheel_deg),
            finite_float(snapshot.steering_average_wheel_deg));
    appendf(&row, "%u,", snapshot.rpm.pulses_per_revolution);
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        appendf(&row, "%u,%.2f,%.2f,%lu,%lu,", snapshot.rpm.valid[wheel],
                finite_float(snapshot.rpm.rpm[wheel]),
                finite_float(snapshot.rpm.frequency_hz[wheel]),
                (unsigned long)snapshot.rpm.edge_count[wheel],
                (unsigned long)snapshot.rpm.window_us[wheel]);
    }
    appendf(&row, "%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
            snapshot.imu.valid, snapshot.imu.bias_calibrated,
            finite_float(snapshot.imu.yaw_rate_dps),
            finite_float(snapshot.imu.accel_mps2[0]),
            finite_float(snapshot.imu.accel_mps2[1]),
            finite_float(snapshot.imu.accel_mps2[2]),
            finite_float(snapshot.imu.gyro_dps[0]),
            finite_float(snapshot.imu.gyro_dps[1]),
            finite_float(snapshot.imu.gyro_dps[2]));
    appendf(&row, "%u,%s,%u,%s,%.3f,%.3f,%.5f,%.3f,%.4f,%.4f,",
            snapshot.config.torque_vectoring_enabled,
            tv_name(snapshot.requested_tv_mode), snapshot.vectoring_active,
            tv_name(snapshot.active_tv_mode), finite_float(snapshot.target_yaw_rate_dps),
            finite_float(snapshot.yaw_error_dps), finite_float(snapshot.side_rpm_error),
            finite_float(snapshot.predicted_lateral_accel_mps2),
            finite_float(snapshot.lateral_demand), finite_float(snapshot.front_relief));
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        appendf(&row, "%.5f,", finite_float(snapshot.wheel_correction[wheel]));
    }
    appendf(&row, "%s,", reason);
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        appendf(&row, "%u,%u,", snapshot.outputs.throttle_us[wheel],
                snapshot.outputs.reverse_us[wheel]);
    }
    appendf(&row, "%u,%u,%u,%lld,%lu,%lu,%lu,%lu,%lu,%u,%.8f,%.8f,%u,%.3f,%u,%.3f,%u,%.3f,%u,%.3f,%lu,%lu,",
            dragy.initialized, dragy.nmea_recent, dragy.fix_valid,
            (long long)gps_age_ms, (unsigned long)dragy.baud_rate,
            (unsigned long)dragy.uart_bytes,
            (unsigned long)dragy.valid_sentence_count,
            (unsigned long)dragy.checksum_error_count,
            (unsigned long)dragy.parse_error_count, dragy.position_valid,
            finite_f64(dragy.latitude_deg), finite_f64(dragy.longitude_deg),
            dragy.speed_valid, finite_float(dragy.speed_mps * 3.6f),
            dragy.course_valid, finite_float(dragy.course_deg),
            dragy.altitude_valid, finite_float(dragy.altitude_m), dragy.satellites,
            finite_float(dragy.hdop), (unsigned long)dragy.utc_time_ms,
            (unsigned long)dragy.date_ddmmyy);
    appendf(&row, "%u,%u,%u,%u,%u,%u,%.1f,%u,%u,%.3f,%u,%u,%.5f,%.6f,%.7f,%.7f,%.6f,%d\n",
            snapshot.config.reverse_limit_percent,
            snapshot.config.receiver_failsafe_enabled,
            snapshot.config.receiver_failsafe_us,
            snapshot.config.receiver_failsafe_window_us,
            snapshot.config.motor_poles,
            snapshot.config.rpm_pulses_per_revolution,
            (double)snapshot.config.steering_trim_tenths_deg / 10.0,
            snapshot.config.steering_smoothing_ms,
            snapshot.config.steering_speed_limit_enabled,
            finite_float(snapshot.config.steering_lateral_accel_g),
            snapshot.config.tv_authority_percent,
            snapshot.config.tv_front_relief_percent,
            finite_float(snapshot.config.tv_turn_yaw_gain_dps),
            finite_float(snapshot.config.tv_turn_rpm_gain),
            finite_float(snapshot.config.tv_yaw_kp),
            finite_float(snapshot.config.tv_yaw_ki),
            finite_float(snapshot.config.tv_rpm_kp), snapshot.config.imu_yaw_sign);
    return !row.overflowed;
}

static esp_err_t open_log_file_locked(void)
{
    struct stat file_stat;
    bool exists = stat(LOG_FILE_PATH, &file_stat) == 0;
    log_file = fopen(LOG_FILE_PATH, "a+");
    if (log_file == NULL) return ESP_FAIL;
    setvbuf(log_file, log_stdio_buffer, _IOFBF, sizeof(log_stdio_buffer));
    if (!exists || file_stat.st_size == 0) {
        size_t header_size = strlen(CSV_HEADER);
        if (fwrite(CSV_HEADER, 1, header_size, log_file) != header_size ||
            fflush(log_file) != 0 || fsync(fileno(log_file)) != 0) {
            fclose(log_file);
            log_file = NULL;
            return ESP_FAIL;
        }
        log_status.file_bytes = header_size;
    } else {
        log_status.file_bytes = (uint64_t)file_stat.st_size;
        FILE *reader = fopen(LOG_FILE_PATH, "rb");
        if (reader != NULL) {
            uint32_t line_count = 0;
            char count_buffer[512];
            size_t count_read;
            while ((count_read = fread(count_buffer, 1, sizeof(count_buffer), reader)) > 0) {
                for (size_t index = 0; index < count_read; index++) {
                    if (count_buffer[index] == '\n') line_count++;
                }
            }
            fclose(reader);
            log_status.sample_count = line_count > 0 ? line_count - 1 : 0;
        }
    }
    log_status.faulted = false;
    log_status.last_errno = 0;
    log_status.recording = true;
    return ESP_OK;
}

static void refresh_space_locked(void)
{
    uint64_t total = 0;
    uint64_t free = 0;
    if (esp_vfs_fat_info(LOG_BASE_PATH, &total, &free) == ESP_OK) {
        log_status.capacity_bytes = total;
        log_status.free_bytes = free;
    }
}

static void mark_storage_fault_locked(const char *operation, int error_number)
{
    int reported_errno = error_number != 0 ? error_number : EIO;
    if (!log_status.faulted) {
        ESP_LOGE(TAG, "CSV storage fault during %s: errno=%d (%s)",
                 operation, reported_errno, strerror(reported_errno));
    }
    log_status.faulted = true;
    log_status.recording = false;
    log_status.last_errno = reported_errno;
}

static uint8_t configured_rate_hz(void)
{
    powertrain_remote_snapshot_t snapshot;
    if (powertrain_get_remote_snapshot(&snapshot) &&
        snapshot.config.telemetry_log_rate_hz >= 1 &&
        snapshot.config.telemetry_log_rate_hz <= 50) {
        return snapshot.config.telemetry_log_rate_hz;
    }
    return DEFAULT_TELEMETRY_LOG_RATE_HZ;
}

static void log_task(void *arg)
{
    (void)arg;
    char row[LOG_ROW_BUFFER_SIZE];
    int64_t next_wake_us = esp_timer_get_time();
    int64_t last_flush_us = next_wake_us;
    uint8_t previous_rate_hz = 0;
    while (true) {
        uint8_t rate_hz = configured_rate_hz();
        uint32_t interval_us = 1000000U / rate_hz;
        int64_t now_us = esp_timer_get_time();
        if (rate_hz != previous_rate_hz) {
            next_wake_us = now_us;
            previous_rate_hz = rate_hz;
            xSemaphoreTake(log_mutex, portMAX_DELAY);
            log_status.rate_hz = rate_hz;
            log_status.interval_ms = (interval_us + 500U) / 1000U;
            xSemaphoreGive(log_mutex);
        }
        next_wake_us += interval_us;
        int64_t delay_us = next_wake_us - now_us;
        if (delay_us > 0) {
            vTaskDelay(pdMS_TO_TICKS((delay_us + 999) / 1000));
        } else {
            next_wake_us = esp_timer_get_time();
        }

        xSemaphoreTake(log_mutex, portMAX_DELAY);
        uint32_t boot_id = log_status.boot_id;
        uint32_t sample_sequence = log_status.sample_count + 1;
        xSemaphoreGive(log_mutex);
        if (!format_row(row, sizeof(row), boot_id, sample_sequence)) {
            xSemaphoreTake(log_mutex, portMAX_DELAY);
            log_status.write_error_count++;
            xSemaphoreGive(log_mutex);
            continue;
        }
        size_t row_size = strlen(row);
        xSemaphoreTake(log_mutex, portMAX_DELAY);
        if (sample_sequence != log_status.sample_count + 1) {
            xSemaphoreGive(log_mutex);
            continue;
        }
        if (log_file != NULL && !log_status.full && !log_status.faulted) {
            if ((log_status.capacity_bytes > 0 &&
                 log_status.free_bytes <= LOG_FREE_RESERVE_BYTES + row_size) ||
                (log_status.capacity_bytes > LOG_FREE_RESERVE_BYTES &&
                 log_status.file_bytes + row_size >=
                    log_status.capacity_bytes - LOG_FREE_RESERVE_BYTES)) {
                log_status.full = true;
                log_status.recording = false;
                if (fflush(log_file) != 0 || fsync(fileno(log_file)) != 0) {
                    int flush_errno = errno;
                    log_status.write_error_count++;
                    log_status.last_errno = flush_errno != 0 ? flush_errno : EIO;
                    clearerr(log_file);
                }
                ESP_LOGW(TAG, "CSV log is full; export and clear it to resume recording");
            } else if (fwrite(row, 1, row_size, log_file) == row_size) {
                log_status.sample_count = sample_sequence;
                log_status.file_bytes += row_size;
            } else {
                int write_errno = errno;
                log_status.write_error_count++;
                if (write_errno == ENOSPC) {
                    log_status.full = true;
                    log_status.recording = false;
                    log_status.last_errno = write_errno;
                    ESP_LOGW(TAG, "CSV log reached filesystem capacity");
                } else {
                    mark_storage_fault_locked("write", write_errno);
                }
                clearerr(log_file);
            }
        }
        now_us = esp_timer_get_time();
        if (log_file != NULL && !log_status.full && !log_status.faulted &&
            now_us - last_flush_us >= LOG_FLUSH_INTERVAL_MS * 1000) {
            if (fflush(log_file) != 0 || fsync(fileno(log_file)) != 0) {
                int flush_errno = errno;
                log_status.write_error_count++;
                mark_storage_fault_locked("flush", flush_errno);
                clearerr(log_file);
            }
            refresh_space_locked();
            last_flush_us = now_us;
        }
        xSemaphoreGive(log_mutex);
    }
}

esp_err_t telemetry_log_start(void)
{
    if (log_mutex != NULL) return ESP_ERR_INVALID_STATE;
    log_mutex = xSemaphoreCreateMutex();
    if (log_mutex == NULL) return ESP_ERR_NO_MEM;

    esp_vfs_fat_mount_config_t mount_config = VFS_FAT_MOUNT_DEFAULT_CONFIG();
    mount_config.format_if_mount_failed = true;
    mount_config.max_files = 3;
    mount_config.allocation_unit_size = 4096;
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(
        LOG_BASE_PATH, LOG_PARTITION_LABEL, &mount_config, &wl_handle);
    if (err != ESP_OK) {
        log_status.initialized = true;
        ESP_LOGE(TAG, "Could not mount CSV log partition: %s", esp_err_to_name(err));
        return err;
    }

    log_status.initialized = true;
    log_status.mounted = true;
    log_status.rate_hz = configured_rate_hz();
    log_status.interval_ms = (1000U + log_status.rate_hz / 2U) /
                             log_status.rate_hz;
    log_status.boot_id = esp_random();
    refresh_space_locked();
    err = open_log_file_locked();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open CSV log file");
        return err;
    }

    BaseType_t created = xTaskCreatePinnedToCore(log_task, "telemetry_log",
                                                 LOG_TASK_STACK_BYTES, NULL,
                                                 LOG_TASK_PRIORITY,
                                                 &log_task_handle, 0);
    if (created != pdPASS) {
        fclose(log_file);
        log_file = NULL;
        log_status.recording = false;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Offline CSV recording at %u Hz (%u ms); %llu usable bytes",
             log_status.rate_hz, log_status.interval_ms,
             (unsigned long long)log_status.capacity_bytes);
    return ESP_OK;
}

void telemetry_log_get_status(telemetry_log_status_t *status)
{
    if (status == NULL) return;
    if (log_mutex == NULL) {
        *status = log_status;
        return;
    }
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    *status = log_status;
    xSemaphoreGive(log_mutex);
}

esp_err_t telemetry_log_read(uint32_t offset,
                             uint8_t *buffer,
                             size_t buffer_size,
                             size_t *bytes_read,
                             uint32_t *file_size)
{
    if (bytes_read == NULL || file_size == NULL ||
        (buffer_size > 0 && buffer == NULL) || log_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *bytes_read = 0;
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    if (!log_status.mounted || log_file == NULL) {
        xSemaphoreGive(log_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (!log_status.faulted && fflush(log_file) != 0) {
        int flush_errno = errno;
        log_status.write_error_count++;
        mark_storage_fault_locked("export flush", flush_errno);
        clearerr(log_file);
    }
    struct stat file_stat;
    if (stat(LOG_FILE_PATH, &file_stat) != 0 || file_stat.st_size < 0 ||
        (uint64_t)file_stat.st_size > UINT32_MAX) {
        int stat_errno = errno;
        ESP_LOGE(TAG, "Could not stat CSV for export: errno=%d (%s)",
                 stat_errno, strerror(stat_errno));
        xSemaphoreGive(log_mutex);
        return ESP_FAIL;
    }
    *file_size = (uint32_t)file_stat.st_size;
    log_status.file_bytes = *file_size;
    if (buffer_size == 0 || offset >= *file_size) {
        xSemaphoreGive(log_mutex);
        return ESP_OK;
    }
    FILE *reader = fopen(LOG_FILE_PATH, "rb");
    if (reader == NULL || fseek(reader, (long)offset, SEEK_SET) != 0) {
        int read_errno = errno;
        if (reader != NULL) fclose(reader);
        ESP_LOGE(TAG, "Could not position CSV export at %lu: errno=%d (%s)",
                 (unsigned long)offset, read_errno, strerror(read_errno));
        xSemaphoreGive(log_mutex);
        return ESP_FAIL;
    }
    size_t available = *file_size - offset;
    size_t requested = buffer_size < available ? buffer_size : available;
    *bytes_read = fread(buffer, 1, requested, reader);
    bool failed = ferror(reader) != 0;
    if (failed) {
        int read_errno = errno;
        ESP_LOGE(TAG, "CSV export read failed at %lu: errno=%d (%s)",
                 (unsigned long)offset, read_errno, strerror(read_errno));
    }
    fclose(reader);
    xSemaphoreGive(log_mutex);
    return failed ? ESP_FAIL : ESP_OK;
}

esp_err_t telemetry_log_clear(void)
{
    if (log_mutex == NULL) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    if (log_file != NULL) {
        fflush(log_file);
        fsync(fileno(log_file));
        fclose(log_file);
        log_file = NULL;
    }
    errno = 0;
    bool removed = unlink(LOG_FILE_PATH) == 0 || errno == ENOENT;
    int clear_errno = errno;
    esp_err_t result = removed ? ESP_OK : ESP_FAIL;
    if (removed) {
        log_status.file_bytes = 0;
        log_status.sample_count = 0;
        log_status.full = false;
        log_status.faulted = false;
        log_status.last_errno = 0;
        log_status.recording = false;
        refresh_space_locked();
        bool allocation_map_exhausted =
            log_status.capacity_bytes > LOG_FREE_RESERVE_BYTES &&
            log_status.free_bytes <= LOG_FREE_RESERVE_BYTES;
        if (allocation_map_exhausted) {
            ESP_LOGW(TAG,
                     "No usable FAT space remained after clear; formatting the "
                     "dedicated telemetry partition");
            result = esp_vfs_fat_spiflash_format_rw_wl(
                LOG_BASE_PATH, LOG_PARTITION_LABEL);
            if (result == ESP_OK) {
                refresh_space_locked();
            }
        }
        if (result == ESP_OK) {
            result = open_log_file_locked();
        }
        if (result != ESP_OK) {
            mark_storage_fault_locked("reopen after clear", errno);
        }
    } else {
        mark_storage_fault_locked("clear", clear_errno);
        struct stat file_stat;
        if (stat(LOG_FILE_PATH, &file_stat) == 0 && file_stat.st_size >= 0) {
            log_status.file_bytes = (uint64_t)file_stat.st_size;
        }
    }
    refresh_space_locked();
    xSemaphoreGive(log_mutex);
    return result;
}

#else

esp_err_t telemetry_log_start(void)
{
    return ESP_OK;
}

void telemetry_log_get_status(telemetry_log_status_t *status)
{
    if (status != NULL) *status = (telemetry_log_status_t){0};
}

esp_err_t telemetry_log_read(uint32_t offset,
                             uint8_t *buffer,
                             size_t buffer_size,
                             size_t *bytes_read,
                             uint32_t *file_size)
{
    (void)offset;
    (void)buffer;
    (void)buffer_size;
    if (bytes_read != NULL) *bytes_read = 0;
    if (file_size != NULL) *file_size = 0;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t telemetry_log_clear(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

#endif
