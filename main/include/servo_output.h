#pragma once

#include "esp_err.h"
#include "powertrain_types.h"

esp_err_t servo_output_init(const uint16_t centers[STEERING_SERVO_COUNT]);
void servo_output_set_pulses(const uint16_t pulses[STEERING_SERVO_COUNT]);
void servo_output_get_pulses(uint16_t pulses[STEERING_SERVO_COUNT]);
