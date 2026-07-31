#pragma once

#include "esp_err.h"
#include "powertrain_types.h"

esp_err_t esc_output_init(const esc_limits_t *limits);
void esc_output_set_all(uint16_t throttle_us, uint16_t reverse_us);
void esc_output_set_wheels(const wheel_output_command_t *command);
void esc_output_set_safe(void);
void esc_output_get(wheel_output_command_t *command);
const esc_limits_t *esc_output_limits(void);
