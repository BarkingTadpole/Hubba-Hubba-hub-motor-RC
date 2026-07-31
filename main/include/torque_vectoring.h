#pragma once

#include "powertrain_types.h"

typedef struct {
    tv_mode_t requested_mode;
    drive_direction_t direction;
    float base_throttle;
    float steering;
    rpm_snapshot_t rpm;
    imu_snapshot_t imu;
    float dt_seconds;
} torque_vectoring_input_t;

typedef struct {
    float wheel_correction[POWERTRAIN_WHEEL_COUNT];
    float target_yaw_rate_dps;
    float yaw_error_dps;
    float side_rpm_error;
    tv_mode_t active_mode;
    bool active;
    const char *inactive_reason;
} torque_vectoring_output_t;

typedef struct {
    float yaw_integral;
    tv_mode_t previous_mode;
} torque_vectoring_state_t;

void torque_vectoring_reset(torque_vectoring_state_t *state);
void torque_vectoring_update(torque_vectoring_state_t *state,
                             const drive_config_t *config,
                             const torque_vectoring_input_t *input,
                             torque_vectoring_output_t *output);
