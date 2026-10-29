#include "config_store.h"

#include <stddef.h>
#include <stdio.h>

#include "calibration_validation.h"

#include "default_config.h"
#include "independent_steering.h"
#include "steering_limit.h"
#include "nvs.h"
#include "nvs_flash.h"

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

static void load_legacy_throttle(throttle_calibration_t *calibration, bool read_nvs)
{
    *calibration = (throttle_calibration_t){
        .full_throttle_us = DEFAULT_RC_THROTTLE_FULL_US,
        .neutral_us = DEFAULT_RC_THROTTLE_NEUTRAL_US,
        .full_reverse_us = DEFAULT_RC_THROTTLE_REVERSE_US,
        .deadband_us = DEFAULT_RC_THROTTLE_DEADBAND_US,
    };

    if (!read_nvs) return;
    nvs_handle_t nvs;
    if (nvs_open("cal", NVS_READONLY, &nvs) != ESP_OK) {
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

    if (err == ESP_OK && calibration_endpoints_valid(throttle, neutral, reverse) &&
        deadband >= 10 && deadband <= 150) {
        calibration->full_throttle_us = throttle;
        calibration->neutral_us = neutral;
        calibration->full_reverse_us = reverse;
        calibration->deadband_us = deadband;
        calibration->loaded_from_nvs = true;
    }
}

static esp_err_t save_legacy_throttle(const throttle_calibration_t *calibration)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("cal", NVS_READWRITE, &nvs);
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

static void load_legacy_steering(steering_calibration_t *calibration, bool read_nvs)
{
    *calibration = (steering_calibration_t){
        .left_us = DEFAULT_RC_STEERING_LEFT_US,
        .center_us = DEFAULT_RC_STEERING_CENTER_US,
        .right_us = DEFAULT_RC_STEERING_RIGHT_US,
        .deadband_us = DEFAULT_RC_STEERING_DEADBAND_US,
    };
    if (!read_nvs) return;

    nvs_handle_t nvs;
    if (nvs_open("cal", NVS_READONLY, &nvs) != ESP_OK) return;
    uint16_t left = 0, center = 0, right = 0, deadband = 0;
    esp_err_t err = nvs_get_u16(nvs, "st_left", &left);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "st_center", &center);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "st_right", &right);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "st_dead", &deadband);
    nvs_close(nvs);

    if (err == ESP_OK && calibration_steering_valid(left, center, right, deadband)) {
        calibration->left_us = left;
        calibration->center_us = center;
        calibration->right_us = right;
        calibration->deadband_us = deadband;
        calibration->loaded_from_nvs = true;
    }
}

static esp_err_t save_legacy_steering(const steering_calibration_t *calibration)
{
    if (calibration == NULL || !calibration_steering_valid(calibration->left_us, calibration->center_us,
            calibration->right_us, calibration->deadband_us)) return ESP_ERR_INVALID_ARG;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("cal", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_u16(nvs, "st_left", calibration->left_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "st_center", calibration->center_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "st_right", calibration->right_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "st_dead", calibration->deadband_us);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

static void load_legacy_arm(arm_calibration_t *calibration, bool read_nvs)
{
    *calibration = (arm_calibration_t){
        .run_us = DEFAULT_RC_ARM_RUN_US,
        .stop_1_us = DEFAULT_RC_ARM_STOP_1_US,
        .stop_2_us = DEFAULT_RC_ARM_STOP_2_US,
    };
    if (!read_nvs) return;

    nvs_handle_t nvs;
    if (nvs_open("cal", NVS_READONLY, &nvs) != ESP_OK) return;
    uint16_t run = 0, stop_1 = 0, stop_2 = 0;
    esp_err_t err = nvs_get_u16(nvs, "arm_run", &run);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "arm_stop", &stop_1);
    if (err == ESP_OK) err = nvs_get_u16(nvs, "arm_stop2", &stop_2);
    nvs_close(nvs);
    if (err == ESP_OK && calibration_arm_positions_valid(run, stop_1, stop_2)) {
        calibration->run_us = run;
        calibration->stop_1_us = stop_1;
        calibration->stop_2_us = stop_2;
        calibration->loaded_from_nvs = true;
    }
}

static esp_err_t save_legacy_arm(const arm_calibration_t *calibration)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("cal", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_u16(nvs, "arm_run", calibration->run_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "arm_stop", calibration->stop_1_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "arm_stop2", calibration->stop_2_us);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

static void load_legacy_drive(drive_config_t *config, bool read_nvs)
{
    *config = (drive_config_t){
        .drivetrain_mode = DRIVETRAIN_AWD,
        .reverse_limit_percent = DEFAULT_REVERSE_LIMIT_PERCENT,
        .tank_limit_percent = DEFAULT_TANK_LIMIT_PERCENT,
        .drive_smoothing_percent = DEFAULT_DRIVE_SMOOTHING_PERCENT,
        .receiver_failsafe_enabled = DEFAULT_RC_THROTTLE_FAILSAFE_ENABLED != 0,
        .receiver_failsafe_us = DEFAULT_RC_THROTTLE_FAILSAFE_US,
        .receiver_failsafe_window_us = DEFAULT_RC_THROTTLE_FAILSAFE_WINDOW_US,
        .motor_poles = 14,
        .rpm_pulses_per_revolution = 7,
        .steering_smoothing_ms = DEFAULT_STEERING_SMOOTHING_MS,
        .permanent_arm_latch_enabled =
            DEFAULT_PERMANENT_ARM_LATCH_ENABLED != 0,
    };

    independent_steering_defaults(&config->steering);
    steering_limit_defaults(&config->steering_limit);
    if (!read_nvs) return;

    nvs_handle_t nvs;
    if (nvs_open("cfg", NVS_READONLY, &nvs) != ESP_OK) return;

    bool found = false;
    uint8_t u8 = 0;
    uint16_t u16 = 0;

    if (nvs_get_u8(nvs, "drive_mode", &u8) == ESP_OK && u8 <= DRIVETRAIN_TANK) {
        config->drivetrain_mode = (drivetrain_mode_t)u8; found = true;
    }
    if (nvs_get_u8(nvs, "rev_limit", &u8) == ESP_OK && u8 <= 100) {
        config->reverse_limit_percent = u8; found = true;
    }
    if (nvs_get_u8(nvs, "tank_limit", &u8) == ESP_OK && u8 <= 100) {
        config->tank_limit_percent = u8; found = true;
    }
    if (nvs_get_u8(nvs, "drive_smooth", &u8) == ESP_OK && u8 <= 100) {
        config->drive_smoothing_percent = u8; found = true;
    }
    if (nvs_get_u8(nvs, "fs_en", &u8) == ESP_OK && u8 <= 1) {
        config->receiver_failsafe_enabled = u8 != 0; found = true;
    }
    if (nvs_get_u16(nvs, "fs_us", &u16) == ESP_OK && calibration_pulse_valid(u16)) {
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
    if (nvs_get_u16(nvs, "st_smooth", &u16) == ESP_OK && u16 <= 500) {
        config->steering_smoothing_ms = u16; found = true;
    }
    if (nvs_get_u8(nvs, "arm_latch", &u8) == ESP_OK && u8 <= 1) {
        config->permanent_arm_latch_enabled = u8 != 0; found = true;
    }
    steering_limit_config_t limit = {0};
    size_t limit_size = sizeof(limit);
    if (nvs_get_blob(nvs, "steer_limit_v1", &limit, &limit_size) == ESP_OK &&
        limit_size == sizeof(limit) && steering_limit_config_valid(&limit)) {
        config->steering_limit = limit;
        found = true;
    }
    independent_steering_config_t steering = {0};
    size_t steering_size = sizeof(steering);
    esp_err_t steering_err = nvs_get_blob(nvs, "dual_steer_v6", &steering, &steering_size);
    if (steering_err == ESP_ERR_NVS_NOT_FOUND) {
        /* Older endpoint angles are unmeasured; never invent measured points. */
        steering = config->steering;
        steering_size = offsetof(independent_steering_config_t, wheel_calibration);
        steering_err = nvs_get_blob(nvs, "dual_steer_v5", &steering, &steering_size);
        if (steering_err == ESP_OK && steering_size == offsetof(independent_steering_config_t, wheel_calibration) &&
            steering.version == 5) {
            steering.version = INDEPENDENT_STEERING_VERSION;
            steering_size = sizeof(steering);
        } else if (steering_err == ESP_OK) steering_err = ESP_ERR_INVALID_ARG;
    }
    if (steering_err == ESP_ERR_NVS_NOT_FOUND) {
        /* V4 retains calibration, trims and linkage limits; append zero toe. */
        steering = config->steering;
        steering_size = offsetof(independent_steering_config_t, toe_tenths_deg);
        steering_err = nvs_get_blob(nvs, "dual_steer_v4", &steering, &steering_size);
        if (steering_err == ESP_OK && steering_size == offsetof(independent_steering_config_t, toe_tenths_deg) &&
            steering.version == 4) {
            steering.version = INDEPENDENT_STEERING_VERSION;
            steering_size = sizeof(steering);
        } else if (steering_err == ESP_OK) {
            steering_err = ESP_ERR_INVALID_ARG;
        }
    }
    if (steering_err == ESP_ERR_NVS_NOT_FOUND) {
        /* Preserve v3 trims/calibration; append default linkage limits.
         * Migration is read-only. A present but invalid blob never falls back. */
        steering = config->steering;
        steering_size = offsetof(independent_steering_config_t, linkage);
        steering_err = nvs_get_blob(nvs, "dual_steer_v3", &steering, &steering_size);
        if (steering_err == ESP_OK && steering_size == offsetof(independent_steering_config_t, linkage) &&
            steering.version == 3) {
            steering.version = INDEPENDENT_STEERING_VERSION;
            steering_size = sizeof(steering);
        } else if (steering_err == ESP_OK) {
            steering_err = ESP_ERR_INVALID_ARG;
        }
    }
    if (steering_err == ESP_ERR_NVS_NOT_FOUND) {
        /* V2 has neither trims nor linkage limits. */
        steering = config->steering;
        steering_size = offsetof(independent_steering_config_t, trim_us);
        steering_err = nvs_get_blob(nvs, "dual_steer_v2", &steering, &steering_size);
        if (steering_err == ESP_OK && steering_size == offsetof(independent_steering_config_t, trim_us) &&
            steering.version == 2) {
            steering.version = INDEPENDENT_STEERING_VERSION;
            steering_size = sizeof(steering);
        } else if (steering_err == ESP_OK) {
            steering_err = ESP_ERR_INVALID_ARG;
        }
    }
    if (steering_err == ESP_OK &&
        steering_size == sizeof(steering) && independent_steering_config_valid(&steering)) {
        config->steering = steering;
        found = true;
    } else if (steering_err == ESP_ERR_NVS_NOT_FOUND) {
        /* V1 was a 48-byte PWM-reduction model. Its percentages have a
         * different meaning, so retain endpoints only, never its curve. */
        uint16_t legacy[24] = {0};
        size_t legacy_size = sizeof(legacy);
        if (nvs_get_blob(nvs, "dual_steer_v1", legacy, &legacy_size) == ESP_OK &&
            legacy_size == sizeof(legacy) && legacy[0] == 1) {
            steering = config->steering;
            for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
                steering.servo[i].left_us = legacy[1 + i * 3];
                steering.servo[i].center_us = legacy[2 + i * 3];
                steering.servo[i].right_us = legacy[3 + i * 3];
            }
            if (independent_steering_config_valid(&steering)) {
                config->steering = steering;
                found = true;
            }
        }
    }
    nvs_close(nvs);
    config->loaded_from_nvs = found;
}

static esp_err_t save_legacy_drive(const drive_config_t *config)
{
    if (config == NULL || config->drivetrain_mode < DRIVETRAIN_AWD ||
        config->drivetrain_mode > DRIVETRAIN_TANK || config->tank_limit_percent > 100 ||
        !independent_steering_config_valid(&config->steering) ||
        !steering_limit_config_valid(&config->steering_limit)) return ESP_ERR_INVALID_ARG;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("cfg", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(nvs, "drive_mode", (uint8_t)config->drivetrain_mode);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "rev_limit", config->reverse_limit_percent);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "tank_limit", config->tank_limit_percent);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "drive_smooth", config->drive_smoothing_percent);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "fs_en", config->receiver_failsafe_enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "fs_us", config->receiver_failsafe_us);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "fs_window", config->receiver_failsafe_window_us);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "mtr_poles", config->motor_poles);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "rpm_ppr", config->rpm_pulses_per_revolution);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "st_smooth", config->steering_smoothing_ms);
    if (err == ESP_OK) err = nvs_set_blob(nvs, "dual_steer_v6", &config->steering, sizeof(config->steering));
    if (err == ESP_OK) err = nvs_set_blob(nvs, "steer_limit_v1", &config->steering_limit, sizeof(config->steering_limit));
    if (err == ESP_OK) err = nvs_set_u8(nvs, "arm_latch", config->permanent_arm_latch_enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

/* A complete imported profile is authoritative. A corrupt present profile uses
 * uncalibrated defaults instead of combining stale legacy keys. */
static int read_profile(configuration_bundle_t *bundle)
{
    nvs_handle_t nvs;
    if (nvs_open("cfg", NVS_READONLY, &nvs) != ESP_OK) return 0;
    uint8_t data[CONFIG_BACKUP_SIZE];
    size_t size = sizeof(data);
    esp_err_t err = nvs_get_blob(nvs, "profile_v1", data, &size);
    nvs_close(nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) return 0;
    return err == ESP_OK && config_backup_decode(data, size, bundle) ? 1 : -1;
}

static void load_legacy_bundle(configuration_bundle_t *bundle, bool read_nvs)
{
    load_legacy_throttle(&bundle->throttle, read_nvs);
    load_legacy_steering(&bundle->input, read_nvs);
    load_legacy_arm(&bundle->arm, read_nvs);
    load_legacy_drive(&bundle->drive, read_nvs);
}

void config_store_load_bundle(configuration_bundle_t *bundle)
{
    int profile = read_profile(bundle);
    if (profile != 1) load_legacy_bundle(bundle, profile == 0);
}

static esp_err_t erase_calibration_keys(const char *const *keys, size_t count)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("cal", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    for (size_t i = 0; i < count && err == ESP_OK; i++) {
        err = nvs_erase_key(nvs, keys[i]);
        if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;
    }
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

static esp_err_t save_profile(const configuration_bundle_t *bundle, unsigned mirror)
{
    uint8_t data[CONFIG_BACKUP_SIZE];
    if (!config_backup_encode(bundle, data, sizeof(data))) return ESP_ERR_INVALID_ARG;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("cfg", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(nvs, "profile_v1", data, sizeof(data));
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) return err;

    /* Preserve the legacy key formats after the complete record is durable.
     * A failed mirror cannot invalidate the authoritative imported profile. */
    const char *const throttle_keys[] = {"rx_throttle","rx_neutral","rx_reverse","rx_deadband"};
    const char *const input_keys[] = {"st_left","st_center","st_right","st_dead"};
    const char *const arm_keys[] = {"arm_run","arm_stop","arm_stop2"};
    bool failed = false;
    if (mirror & 1) failed |= (bundle->throttle.loaded_from_nvs ? save_legacy_throttle(&bundle->throttle)
        : erase_calibration_keys(throttle_keys, 4)) != ESP_OK;
    if (mirror & 2) failed |= (bundle->input.loaded_from_nvs ? save_legacy_steering(&bundle->input)
        : erase_calibration_keys(input_keys, 4)) != ESP_OK;
    if (mirror & 4) failed |= (bundle->arm.loaded_from_nvs ? save_legacy_arm(&bundle->arm)
        : erase_calibration_keys(arm_keys, 3)) != ESP_OK;
    if (mirror & 8) failed |= save_legacy_drive(&bundle->drive) != ESP_OK;
    if (failed) printf("WARN: complete profile saved; legacy compatibility mirror incomplete\n");
    return ESP_OK;
}

esp_err_t config_store_save_bundle(const configuration_bundle_t *bundle)
{
    return save_profile(bundle, 15);
}

void config_store_load_throttle(throttle_calibration_t *calibration)
{
    configuration_bundle_t bundle;
    int profile = read_profile(&bundle);
    if (profile == 1) *calibration = bundle.throttle;
    else load_legacy_throttle(calibration, profile == 0);
}

void config_store_load_steering(steering_calibration_t *calibration)
{
    configuration_bundle_t bundle;
    int profile = read_profile(&bundle);
    if (profile == 1) *calibration = bundle.input;
    else load_legacy_steering(calibration, profile == 0);
}

void config_store_load_arm(arm_calibration_t *calibration)
{
    configuration_bundle_t bundle;
    int profile = read_profile(&bundle);
    if (profile == 1) *calibration = bundle.arm;
    else load_legacy_arm(calibration, profile == 0);
}

void config_store_load_drive(drive_config_t *config)
{
    configuration_bundle_t bundle;
    int profile = read_profile(&bundle);
    if (profile == 1) *config = bundle.drive;
    else load_legacy_drive(config, profile == 0);
}

esp_err_t config_store_save_throttle(const throttle_calibration_t *calibration)
{
    if (calibration == NULL) return ESP_ERR_INVALID_ARG;
    configuration_bundle_t bundle;
    int profile = read_profile(&bundle);
    if (profile == 0) return save_legacy_throttle(calibration);
    if (profile < 0) load_legacy_bundle(&bundle, false);
    bundle.throttle = *calibration;
    return save_profile(&bundle, 1);
}

esp_err_t config_store_save_steering(const steering_calibration_t *calibration)
{
    if (calibration == NULL) return ESP_ERR_INVALID_ARG;
    configuration_bundle_t bundle;
    int profile = read_profile(&bundle);
    if (profile == 0) return save_legacy_steering(calibration);
    if (profile < 0) load_legacy_bundle(&bundle, false);
    bundle.input = *calibration;
    return save_profile(&bundle, 2);
}

esp_err_t config_store_save_arm(const arm_calibration_t *calibration)
{
    if (calibration == NULL) return ESP_ERR_INVALID_ARG;
    configuration_bundle_t bundle;
    int profile = read_profile(&bundle);
    if (profile == 0) return save_legacy_arm(calibration);
    if (profile < 0) load_legacy_bundle(&bundle, false);
    bundle.arm = *calibration;
    return save_profile(&bundle, 4);
}

esp_err_t config_store_save_drive(const drive_config_t *config)
{
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    configuration_bundle_t bundle;
    int profile = read_profile(&bundle);
    if (profile == 0) return save_legacy_drive(config);
    if (profile < 0) load_legacy_bundle(&bundle, false);
    bundle.drive = *config;
    return save_profile(&bundle, 8);
}
