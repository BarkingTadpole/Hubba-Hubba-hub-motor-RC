#pragma once

#include "esp_err.h"
#include "powertrain_types.h"

esp_err_t rpm_sensor_init(uint16_t pulses_per_revolution);
void rpm_sensor_set_pulses_per_revolution(uint16_t pulses_per_revolution);
uint16_t rpm_sensor_pulses_per_revolution(void);
void rpm_sensor_update(void);
void rpm_sensor_get_snapshot(rpm_snapshot_t *snapshot);
