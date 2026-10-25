#pragma once

#include "powertrain_types.h"

void independent_steering_defaults(independent_steering_config_t *config);
bool steering_servo_config_valid(const steering_servo_config_t *servo);
bool steering_geometry_valid(const steering_geometry_t *geometry);
bool steering_linkage_limits_valid(const steering_linkage_limits_t *limits);
bool steering_wheel_calibration_valid(const independent_steering_config_t *config, uint8_t side);
/* Physical-CH1 measurement mode: PWM interpolation without the wheel-angle model. */
void independent_steering_measurement_pulses(const independent_steering_config_t *config,
                                            uint8_t side, float command, uint16_t pulses[STEERING_SERVO_COUNT]);
/* Shared toe range from both calibrated servos and linkage caps, rounded inward
 * to 0.1 degree. Does not depend on the current toe or requested turn limits. */
bool independent_steering_toe_range(const independent_steering_config_t *config,
                                    int16_t *minimum_tenths_deg, int16_t *maximum_tenths_deg);
bool ackermann_curve_valid(const ackermann_point_t *points, uint16_t count);
bool independent_steering_config_valid(const independent_steering_config_t *config);
float ackermann_strength_percent(const independent_steering_config_t *config,
                                  float speed_kmh);
/* Positive command/angles turn right. 0% parallel, 100% ideal Ackermann.
 * Toe is added after mixing; linkage limits include toe and reduce both
 * steering components while preserving their blend relationship.
 * angles_deg may be NULL; reported angles are targets, not sensor feedback. */
void independent_steering_mix(const independent_steering_config_t *config,
                              float command, float strength_percent,
                              uint16_t pulses[STEERING_SERVO_COUNT],
                              float angles_deg[STEERING_SERVO_COUNT]);
