#pragma once

#include "powertrain_types.h"

void independent_steering_defaults(independent_steering_config_t *config);
bool steering_servo_config_valid(const steering_servo_config_t *servo);
bool steering_geometry_valid(const steering_geometry_t *geometry);
bool ackermann_curve_valid(const ackermann_point_t *points, uint16_t count);
bool independent_steering_config_valid(const independent_steering_config_t *config);
float ackermann_strength_percent(const independent_steering_config_t *config,
                                  float speed_kmh);
/* Positive command/angles turn right. 0% parallel, 100% ideal Ackermann.
 * angles_deg may be NULL; reported angles are targets, not sensor feedback. */
void independent_steering_mix(const independent_steering_config_t *config,
                              float command, float strength_percent,
                              uint16_t pulses[STEERING_SERVO_COUNT],
                              float angles_deg[STEERING_SERVO_COUNT]);
