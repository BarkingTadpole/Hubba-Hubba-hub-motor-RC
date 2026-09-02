#include "config_store.h"

#include <math.h>
#include <stdlib.h>

#include "default_config.h"
#include "nvs.h"
#include "nvs_flash.h"

static bool pulse_valid(uint16_t pulse_us)
{
    return pulse_us >= 800 && pulse_us <= 2200;
}

static esp_err_t open_namespace(const char *name, nvs_open_mode_t mode, nvs_handle_t *handle)
{
    return nvs_open(name, mode, handle);
}

esp_err_t config_store_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    return err;
}

void config_store_load_throttle(throttle_calibration_t *calibration)
{
    *calibration = (throttle_calibration_t){
        .full_throttle_us = DEFAULT_RC_THROTTLE_FULL_US,
        .neutral_us = DEFAULT_RC_THROTTLE_NEUTRAL_US,
        .full_reverse_us = DEFAULT_RC_THROTTLE_REVERSE_US,
        .deadband_us = DEFAULT_RC_THROTTLE_DEADBAND_US,
    };

    nvs_handle_t nvs;
    if (open_namespace("cal", NVS_READONLY, &nvs) != ESP_OK) {
        return;
    }

    uint16_t throttle = 0;
    uint16_t neutral = 0;
    uint16_t reverse = 0;
    uint16_t deadband = 0;
    esp_err_t err = nvs_get_u16(nvs, "rx_throttle", &throttle);
    if (err == ESP_OK) {
        err = nvs_get_u16(nvs, "rx_neutral", &neutral);
    }
    if (err == ESP_OK) {
        err = nvs_get_u16(nvs, "rx_reverse", &reverse);
    }
    if (err == ESP_OK) {
        err = nvs_get_u16(nvs, "rx_deadband", &deadband);
    }
    nvs_close(nvs);

    int32_t throttle_delta = (int32_t)throttle - neutral;
    int32_t reverse_delta = (int32_t)reverse - neutral;
    if (err == ESP_OK && pulse_valid(throttle) && pulse_valid(neutral) &&
        pulse_valid(reverse) && abs(throttle_delta) >= 150 &&
        abs(reverse_delta) >= 150 && ((throttle_delta > 0) != (reverse_delta > 0)) &&
        deadband >= 10 && deadband <= 150) {
        calibration->full_throttle_us = throttle;
        calibration->neutral_us = neutral;
        calibration->full_reverse_us = reverse;
        calibration->deadband_us = deadband;
        calibration->loaded_from_nvs = true;
    }
}

esp_err_t config_store_save_throttle(const throttle_calibration_t *calibration)
{
    nvs_handle_t nvs;
    esp_err_t err = open_namespace("cal", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u16(nvs, "rx_throttle", calibration->full_throttle_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "rx_neutral", calibration->neutral_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "rx_reverse", calibration->full_reverse_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "rx_deadband", calibration->deadband_us);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

void config_store_load_steering(steering_calibration_t *calibration)
{
    *calibration = (steering_calibration_t){
        .left_us = DEFAULT_RC_STEERING_LEFT_US,
        .center_us = DEFAULT_RC_STEERING_CENTER_US,
        .right_us = DEFAULT_RC_STEERING_RIGHT_US,
        .deadband_us = DEFAULT_RC_STEERING_DEADBAND_US,
    };

    nvs_handle_t nvs;
    if (open_namespace("cal", NVS_READONLY, &nvs) != ESP_OK) return;
    uint16_t left = 0, center = 0, right = 0, deadband = 0;
    esp_err_t err = nvs_get_u16(nvs, "st_left", &left);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "st_center", &center);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "st_right", &right);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "st_dead", &deadband);
    nvs_close(nvs);

    int32_t left_delta = (int32_t)left - center;
    int32_t right_delta = (int32_t)right - center;
    if (err == ESP_OK && pulse_valid(left) && pulse_valid(center) && pulse_valid(right) &&
        abs(left_delta) >= 150 && abs(right_delta) >= 150 &&
        ((left_delta > 0) != (right_delta > 0)) &&
        deadband >= 5 && deadband <= 150) {
        calibration->left_us = left;
        calibration->center_us = center;
        calibration->right_us = right;
        calibration->deadband_us = deadband;
        calibration->loaded_from_nvs = true;
    }
}

esp_err_t config_store_save_steering(const steering_calibration_t *calibration)
{
    nvs_handle_t nvs;
    esp_err_t err = open_namespace("cal", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_u16(nvs, "st_left", calibration->left_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "st_center", calibration->center_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "st_right", calibration->right_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "st_dead", calibration->deadband_us);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

void config_store_load_arm(arm_calibration_t *calibration)
{
    *calibration = (arm_calibration_t){
        .run_us = DEFAULT_RC_ARM_RUN_US,
        .stop_1_us = DEFAULT_RC_ARM_STOP_1_US,
        .stop_2_us = DEFAULT_RC_ARM_STOP_2_US,
    };

    nvs_handle_t nvs;
    if (open_namespace("cal", NVS_READONLY, &nvs) != ESP_OK) return;
    uint16_t run = 0, stop_1 = 0, stop_2 = 0;
    esp_err_t err = nvs_get_u16(nvs, "arm_run", &run);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "arm_stop", &stop_1);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "arm_stop2", &stop_2);
    nvs_close(nvs);
    if (err == ESP_OK && pulse_valid(run) && pulse_valid(stop_1) &&
        pulse_valid(stop_2) &&
        abs((int32_t)run - (int32_t)stop_1) >= 250 &&
        abs((int32_t)run - (int32_t)stop_2) >= 250 &&
        abs((int32_t)stop_1 - (int32_t)stop_2) >= 250) {
        calibration->run_us = run;
        calibration->stop_1_us = stop_1;
        calibration->stop_2_us = stop_2;
        calibration->loaded_from_nvs = true;
    }
}

esp_err_t config_store_save_arm(const arm_calibration_t *calibration)
{
    nvs_handle_t nvs;
    esp_err_t err = open_namespace("cal", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_u16(nvs, "arm_run", calibration->run_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "arm_stop", calibration->stop_1_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "arm_stop2", calibration->stop_2_us);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

void config_store_load_tv_mode(tv_mode_calibration_t *calibration)
{
    *calibration = (tv_mode_calibration_t){
        .off_us = DEFAULT_RC_TV_OFF_US,
        .straight_us = DEFAULT_RC_TV_STRAIGHT_US,
        .full_us = DEFAULT_RC_TV_FULL_US,
    };

    nvs_handle_t nvs;
    if (open_namespace("cal", NVS_READONLY, &nvs) != ESP_OK) return;
    uint16_t off = 0, straight = 0, full = 0;
    esp_err_t err = nvs_get_u16(nvs, "tv_off", &off);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "tv_straight", &straight);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "tv_full", &full);
    nvs_close(nvs);
    if (err == ESP_OK && pulse_valid(off) && pulse_valid(straight) && pulse_valid(full) &&
        abs((int32_t)off - (int32_t)straight) >= 150 &&
        abs((int32_t)straight - (int32_t)full) >= 150 &&
        abs((int32_t)off - (int32_t)full) >= 300) {
        calibration->off_us = off;
        calibration->straight_us = straight;
        calibration->full_us = full;
        calibration->loaded_from_nvs = true;
    }
}

esp_err_t config_store_save_tv_mode(const tv_mode_calibration_t *calibration)
{
    nvs_handle_t nvs;
    esp_err_t err = open_namespace("cal", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_u16(nvs, "tv_off", calibration->off_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "tv_straight", calibration->straight_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "tv_full", calibration->full_us);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

static uint32_t float_to_scaled(float value)
{
    return (uint32_t)lroundf(value * 1000000.0f);
}

static float scaled_to_float(uint32_t value)
{
    return (float)value / 1000000.0f;
}

void config_store_load_drive(drive_config_t *config)
{
    *config = (drive_config_t){
        .drivetrain_mode = DRIVETRAIN_AWD,
        .reverse_limit_percent = DEFAULT_REVERSE_LIMIT_PERCENT,
        .drive_smoothing_percent = DEFAULT_DRIVE_SMOOTHING_PERCENT,
        .receiver_failsafe_enabled = DEFAULT_RC_THROTTLE_FAILSAFE_ENABLED != 0,
        .receiver_failsafe_us = DEFAULT_RC_THROTTLE_FAILSAFE_US,
        .receiver_failsafe_window_us = DEFAULT_RC_THROTTLE_FAILSAFE_WINDOW_US,
        .motor_poles = 14,
        .rpm_pulses_per_revolution = 7,
        .steering_trim_tenths_deg = 0,
        .steering_smoothing_ms = DEFAULT_STEERING_SMOOTHING_MS,
        .steering_speed_limit_enabled = DEFAULT_STEERING_SPEED_LIMIT_ENABLED != 0,
        .steering_lateral_accel_g = DEFAULT_STEERING_LATERAL_ACCEL_G,
        .torque_vectoring_enabled = DEFAULT_TV_ENABLED != 0,
        .tv_authority_percent = DEFAULT_TV_AUTHORITY_PERCENT,
        .tv_front_relief_percent = DEFAULT_TV_FRONT_RELIEF_PERCENT,
        .tv_turn_yaw_gain_dps = 180.0f,
        .tv_turn_rpm_gain = 0.20f,
        .tv_yaw_kp = 0.00025f,
        .tv_yaw_ki = 0.00004f,
        .tv_rpm_kp = 0.20f,
        .imu_yaw_sign = DEFAULT_IMU_YAW_SIGN,
        .permanent_arm_latch_enabled =
            DEFAULT_PERMANENT_ARM_LATCH_ENABLED != 0,
        .telemetry_log_rate_hz = DEFAULT_TELEMETRY_LOG_RATE_HZ,
    };

    nvs_handle_t nvs;
    if (open_namespace("cfg", NVS_READONLY, &nvs) != ESP_OK) return;

    bool found = false;
    uint8_t u8 = 0;
    uint16_t u16 = 0;
    uint32_t u32 = 0;
    int16_t i16 = 0;
    int8_t i8 = 0;

    if (nvs_get_u8(nvs, "drive_mode", &u8) == ESP_OK && u8 <= DRIVETRAIN_RWD) {
        config->drivetrain_mode = (drivetrain_mode_t)u8; found = true;
    }
    if (nvs_get_u8(nvs, "rev_limit", &u8) == ESP_OK && u8 <= 100) {
        config->reverse_limit_percent = u8; found = true;
    }
    if (nvs_get_u8(nvs, "drive_smooth", &u8) == ESP_OK && u8 <= 100) {
        config->drive_smoothing_percent = u8; found = true;
    }
    if (nvs_get_u8(nvs, "fs_en", &u8) == ESP_OK && u8 <= 1) {
        config->receiver_failsafe_enabled = u8 != 0; found = true;
    }
    if (nvs_get_u16(nvs, "fs_us", &u16) == ESP_OK && pulse_valid(u16)) {
        config->receiver_failsafe_us = u16; found = true;
    }
    if (nvs_get_u16(nvs, "fs_window", &u16) == ESP_OK && u16 >= 1 && u16 <= 100) {
        config->receiver_failsafe_window_us = u16; found = true;
    }
    if (nvs_get_u8(nvs, "mtr_poles", &u8) == ESP_OK &&
        u8 >= 2 && u8 <= 60 && (u8 % 2) == 0) {
        config->motor_poles = u8;
        config->rpm_pulses_per_revolution = u8 / 2;
        found = true;
    }
    if (nvs_get_u16(nvs, "rpm_ppr", &u16) == ESP_OK && u16 >= 1 && u16 <= 120) {
        config->rpm_pulses_per_revolution = u16; found = true;
    }
    if (nvs_get_i16(nvs, "st_trim10", &i16) == ESP_OK && i16 >= -150 && i16 <= 150) {
        config->steering_trim_tenths_deg = i16; found = true;
    }
    if (nvs_get_u16(nvs, "st_smooth", &u16) == ESP_OK && u16 <= 500) {
        config->steering_smoothing_ms = u16; found = true;
    }
    if (nvs_get_u8(nvs, "st_spdlim", &u8) == ESP_OK && u8 <= 1) {
        config->steering_speed_limit_enabled = u8 != 0; found = true;
    }
    if (nvs_get_u32(nvs, "st_latg", &u32) == ESP_OK) {
        float lateral_accel_g = scaled_to_float(u32);
        if (isfinite(lateral_accel_g) && lateral_accel_g >= 0.2f &&
            lateral_accel_g <= 3.0f) {
            config->steering_lateral_accel_g = lateral_accel_g; found = true;
        }
    }
    if (nvs_get_u8(nvs, "tv_en", &u8) == ESP_OK && u8 <= 1) {
        config->torque_vectoring_enabled = u8 != 0; found = true;
    }
    if (nvs_get_u8(nvs, "tv_auth", &u8) == ESP_OK && u8 <= 25) {
        config->tv_authority_percent = u8; found = true;
    }
    if (nvs_get_u8(nvs, "tv_frelief", &u8) == ESP_OK && u8 <= 50) {
        config->tv_front_relief_percent = u8; found = true;
    }
    if (nvs_get_u32(nvs, "yaw_gain", &u32) == ESP_OK && u32 <= 500000000) {
        config->tv_turn_yaw_gain_dps = scaled_to_float(u32); found = true;
    }
    if (nvs_get_u32(nvs, "turn_rpm", &u32) == ESP_OK && u32 <= 1000000) {
        config->tv_turn_rpm_gain = scaled_to_float(u32); found = true;
    }
    if (nvs_get_u32(nvs, "yaw_kp", &u32) == ESP_OK && u32 <= 1000000) {
        config->tv_yaw_kp = scaled_to_float(u32); found = true;
    }
    if (nvs_get_u32(nvs, "yaw_ki", &u32) == ESP_OK && u32 <= 1000000) {
        config->tv_yaw_ki = scaled_to_float(u32); found = true;
    }
    if (nvs_get_u32(nvs, "rpm_kp", &u32) == ESP_OK && u32 <= 5000000) {
        config->tv_rpm_kp = scaled_to_float(u32); found = true;
    }
    if (nvs_get_i8(nvs, "imu_sign", &i8) == ESP_OK && (i8 == -1 || i8 == 1)) {
        config->imu_yaw_sign = i8; found = true;
    }
    if (nvs_get_u8(nvs, "arm_latch", &u8) == ESP_OK && u8 <= 1) {
        config->permanent_arm_latch_enabled = u8 != 0; found = true;
    }
    if (nvs_get_u8(nvs, "log_hz", &u8) == ESP_OK && u8 >= 1 && u8 <= 50) {
        config->telemetry_log_rate_hz = u8; found = true;
    }
    nvs_close(nvs);
    config->loaded_from_nvs = found;
}

esp_err_t config_store_save_drive(const drive_config_t *config)
{
    nvs_handle_t nvs;
    esp_err_t err = open_namespace("cfg", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(nvs, "drive_mode", (uint8_t)config->drivetrain_mode);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "rev_limit", config->reverse_limit_percent);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "drive_smooth", config->drive_smoothing_percent);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "fs_en", config->receiver_failsafe_enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "fs_us", config->receiver_failsafe_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "fs_window", config->receiver_failsafe_window_us);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "mtr_poles", config->motor_poles);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "rpm_ppr", config->rpm_pulses_per_revolution);
    if (err == ESP_OK) err = nvs_set_i16(nvs, "st_trim10", config->steering_trim_tenths_deg);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "st_smooth", config->steering_smoothing_ms);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "st_spdlim", config->steering_speed_limit_enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_set_u32(nvs, "st_latg", float_to_scaled(config->steering_lateral_accel_g));
    if (err == ESP_OK) err = nvs_set_u8(nvs, "tv_en", config->torque_vectoring_enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "tv_auth", config->tv_authority_percent);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "tv_frelief", config->tv_front_relief_percent);
    if (err == ESP_OK) err = nvs_set_u32(nvs, "yaw_gain", float_to_scaled(config->tv_turn_yaw_gain_dps));
    if (err == ESP_OK) err = nvs_set_u32(nvs, "turn_rpm", float_to_scaled(config->tv_turn_rpm_gain));
    if (err == ESP_OK) err = nvs_set_u32(nvs, "yaw_kp", float_to_scaled(config->tv_yaw_kp));
    if (err == ESP_OK) err = nvs_set_u32(nvs, "yaw_ki", float_to_scaled(config->tv_yaw_ki));
    if (err == ESP_OK) err = nvs_set_u32(nvs, "rpm_kp", float_to_scaled(config->tv_rpm_kp));
    if (err == ESP_OK) err = nvs_set_i8(nvs, "imu_sign", config->imu_yaw_sign);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "arm_latch", config->permanent_arm_latch_enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "log_hz", config->telemetry_log_rate_hz);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}
