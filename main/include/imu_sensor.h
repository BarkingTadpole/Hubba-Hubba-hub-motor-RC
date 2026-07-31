#pragma once

#include "esp_err.h"
#include "powertrain_types.h"

esp_err_t imu_sensor_init(int8_t yaw_sign);
void imu_sensor_set_yaw_sign(int8_t yaw_sign);
bool imu_sensor_update(void);
bool imu_sensor_calibrate_bias(uint32_t duration_ms);
void imu_sensor_get_snapshot(imu_snapshot_t *snapshot);
