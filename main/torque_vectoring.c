#include "torque_vectoring.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RPM_MIN_CONTROL_FREQUENCY_HZ 16.7f

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static void set_inactive(torque_vectoring_state_t *state,
                         torque_vectoring_output_t *output,
                         const char *reason)
{
    state->yaw_integral = 0.0f;
    output->inactive_reason = reason;
}

static bool finite_config(const drive_config_t *config)
{
    return isfinite(config->tv_turn_yaw_gain_dps) &&
           isfinite(config->tv_turn_rpm_gain) &&
           isfinite(config->tv_yaw_kp) &&
           isfinite(config->tv_yaw_ki) &&
           isfinite(config->tv_rpm_kp);
}

void torque_vectoring_reset(torque_vectoring_state_t *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

void torque_vectoring_update(torque_vectoring_state_t *state,
                             const drive_config_t *config,
                             const torque_vectoring_input_t *input,
                             torque_vectoring_output_t *output)
{
    if (state == NULL || config == NULL || input == NULL || output == NULL) {
        return;
    }

    memset(output, 0, sizeof(*output));
    output->active_mode = TV_MODE_OFF;

    if (!finite_config(config) || !isfinite(input->base_throttle) ||
        !isfinite(input->steering) || !isfinite(input->dt_seconds) ||
        !isfinite(input->imu.yaw_rate_dps)) {
        set_inactive(state, output, "controller input or configuration is non-finite");
        return;
    }
    if (input->base_throttle < 0.0f || input->base_throttle > 1.0f ||
        input->steering < -1.0f || input->steering > 1.0f ||
        input->dt_seconds <= 0.0f || input->dt_seconds > 0.1f) {
        set_inactive(state, output, "controller input is outside its valid range");
        return;
    }
    if (!config->torque_vectoring_enabled) {
        set_inactive(state, output, "disabled in configuration");
        return;
    }
    if (input->requested_mode == TV_MODE_OFF) {
        set_inactive(state, output, "CH5 mode is off");
        return;
    }
    if (input->direction != DRIVE_DIRECTION_FORWARD) {
        set_inactive(state, output, "forward drive only");
        return;
    }
    if (input->base_throttle < 0.05f) {
        set_inactive(state, output, "below minimum throttle");
        return;
    }
    if (!input->imu.valid || !input->imu.bias_calibrated) {
        set_inactive(state, output, "IMU unavailable or uncalibrated");
        return;
    }
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        if (!input->rpm.valid[wheel] || !isfinite(input->rpm.rpm[wheel]) ||
            !isfinite(input->rpm.frequency_hz[wheel]) ||
            input->rpm.rpm[wheel] < 0.0f) {
            set_inactive(state, output, "one or more RPM signals are unavailable");
            return;
        }
        if (input->rpm.frequency_hz[wheel] < RPM_MIN_CONTROL_FREQUENCY_HZ) {
            set_inactive(state, output, "RPM frequency is below the sensor's documented range");
            return;
        }
    }
    if (input->requested_mode == TV_MODE_STRAIGHT && fabsf(input->steering) > 0.06f) {
        set_inactive(state, output, "straight assist waits for centered steering");
        return;
    }

    float left_rpm = (input->rpm.rpm[WHEEL_FRONT_LEFT] +
                      input->rpm.rpm[WHEEL_REAR_LEFT]) * 0.5f;
    float right_rpm = (input->rpm.rpm[WHEEL_FRONT_RIGHT] +
                       input->rpm.rpm[WHEEL_REAR_RIGHT]) * 0.5f;
    float average_rpm = (left_rpm + right_rpm) * 0.5f;
    float target_yaw_rate_dps = 0.0f;
    float desired_side_rpm_delta = 0.0f;
    if (input->requested_mode == TV_MODE_FULL) {
        target_yaw_rate_dps = input->steering *
                              config->tv_turn_yaw_gain_dps *
                              input->base_throttle;
        desired_side_rpm_delta = input->steering * config->tv_turn_rpm_gain;
    }

    float measured_side_rpm_delta = (left_rpm - right_rpm) / average_rpm;
    float yaw_error_dps = target_yaw_rate_dps - input->imu.yaw_rate_dps;
    float side_rpm_error = desired_side_rpm_delta - measured_side_rpm_delta;
    float authority = (float)config->tv_authority_percent / 100.0f;

    if (state->previous_mode != input->requested_mode) {
        state->yaw_integral = 0.0f;
    }
    state->previous_mode = input->requested_mode;
    float proportional_and_rpm = (config->tv_yaw_kp * yaw_error_dps) +
                                 (config->tv_rpm_kp * side_rpm_error);
    float candidate_integral = state->yaw_integral +
                               yaw_error_dps * clampf(input->dt_seconds, 0.0f, 0.1f);
    if (config->tv_yaw_ki > 0.0f) {
        float integral_limit = authority / config->tv_yaw_ki;
        candidate_integral = clampf(candidate_integral, -integral_limit, integral_limit);
    } else {
        state->yaw_integral = 0.0f;
        candidate_integral = 0.0f;
    }

    float balanced_limit = input->base_throttle;
    if ((1.0f - input->base_throttle) < balanced_limit) {
        balanced_limit = 1.0f - input->base_throttle;
    }
    if (authority < balanced_limit) {
        balanced_limit = authority;
    }
    float candidate_correction = proportional_and_rpm +
                                 (config->tv_yaw_ki * candidate_integral);
    bool candidate_saturated_high = candidate_correction > balanced_limit;
    bool candidate_saturated_low = candidate_correction < -balanced_limit;
    if ((!candidate_saturated_high && !candidate_saturated_low) ||
        (candidate_saturated_high && yaw_error_dps < 0.0f) ||
        (candidate_saturated_low && yaw_error_dps > 0.0f)) {
        state->yaw_integral = candidate_integral;
    }

    float side_correction = proportional_and_rpm +
                            (config->tv_yaw_ki * state->yaw_integral);
    side_correction = clampf(side_correction, -balanced_limit, balanced_limit);

    output->wheel_correction[WHEEL_FRONT_LEFT] = side_correction;
    output->wheel_correction[WHEEL_REAR_LEFT] = side_correction;
    output->wheel_correction[WHEEL_FRONT_RIGHT] = -side_correction;
    output->wheel_correction[WHEEL_REAR_RIGHT] = -side_correction;
    output->target_yaw_rate_dps = target_yaw_rate_dps;
    output->yaw_error_dps = yaw_error_dps;
    output->side_rpm_error = side_rpm_error;
    output->active_mode = input->requested_mode;
    output->active = true;
    output->inactive_reason = "active";
}
