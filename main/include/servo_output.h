#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t servo_output_init(uint16_t center_us);
void servo_output_set_pulse(uint16_t pulse_us);
void servo_output_set_neutral(void);
uint16_t servo_output_get_pulse(void);
