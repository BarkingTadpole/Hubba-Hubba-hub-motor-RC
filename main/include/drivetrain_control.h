#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "powertrain_types.h"

bool drivetrain_mode_valid(drivetrain_mode_t mode);
const char *drivetrain_mode_name(drivetrain_mode_t mode);
bool drivetrain_wheel_is_driven(drivetrain_mode_t mode, wheel_id_t wheel);
bool drivetrain_apply_output_mask(drivetrain_mode_t mode,
                                  const esc_limits_t *limits,
                                  wheel_output_command_t *command);
uint16_t drivetrain_ramp_step_us(uint16_t smoothest_step_us,
                                 uint8_t smoothing_percent);
/* Tank left reverses FL/RL; tank right reverses FR/RR. Invalid mixes safe all. */
bool drivetrain_mix_outputs(drivetrain_mode_t mode, drive_direction_t direction,
                             uint16_t throttle_us, const esc_limits_t *limits,
                             wheel_output_command_t *command);
