#pragma once

#include "esp_err.h"
#include "powertrain_types.h"

esp_err_t rpm_sensor_init(uint8_t motor_poles);
void rpm_sensor_set_motor_poles(uint8_t motor_poles);
uint8_t rpm_sensor_motor_poles(void);
void rpm_sensor_update(void);
void rpm_sensor_get_snapshot(rpm_snapshot_t *snapshot);
