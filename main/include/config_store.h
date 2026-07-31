#pragma once

#include "esp_err.h"
#include "powertrain_types.h"

esp_err_t config_store_init(void);

void config_store_load_throttle(throttle_calibration_t *calibration);
esp_err_t config_store_save_throttle(const throttle_calibration_t *calibration);

void config_store_load_steering(steering_calibration_t *calibration);
esp_err_t config_store_save_steering(const steering_calibration_t *calibration);

void config_store_load_arm(arm_calibration_t *calibration);
esp_err_t config_store_save_arm(const arm_calibration_t *calibration);

void config_store_load_tv_mode(tv_mode_calibration_t *calibration);
esp_err_t config_store_save_tv_mode(const tv_mode_calibration_t *calibration);

void config_store_load_drive(drive_config_t *config);
esp_err_t config_store_save_drive(const drive_config_t *config);
