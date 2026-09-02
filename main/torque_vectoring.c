#include "torque_vectoring.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "cornering_control.h"
#include "default_config.h"
#include "drivetrain_control.h"

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
           isfinite(config->tv_rpm_kp) &&
           isfinite(config->steering_lateral_accel_g);
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
        !isfinite(input->vehicle_speed_mps) ||
        !isfinite(input->average_wheel_angle_deg) ||
        !isfinite(input->imu.yaw_rate_dps)) {
        set_inactive(state, output, "controller input or configuration is non-finite");
        return;
    }
    if (input->base_throttle < 0.0f || input->base_throttle > 1.0f ||
        input->steering < -1.0f || input->steering > 1.0f ||
        input->vehicle_speed_mps < 0.0f ||
        config->steering_lateral_accel_g < 0.2f ||
        config->steering_lateral_accel_g > 3.0f ||
        config->tv_front_relief_percent > 50 ||
        input->dt_seconds <= 0.0f || input->dt_seconds > 0.1f) {
        set_inactive(state, output, "controller input is outside its valid range");
        return;
    }
    if (!drivetrain_mode_valid(config->drivetrain_mode)) {
        set_inactive(state, output, "invalid drivetrain mode");
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
        if (!drivetrain_wheel_is_driven(config->drivetrain_mode,
                                        (wheel_id_t)wheel)) {
            continue;
        }
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

    float left_rpm;
    float right_rpm;
    if (config->drivetrain_mode == DRIVETRAIN_FWD) {
        left_rpm = input->rpm.rpm[WHEEL_FRONT_LEFT];
        right_rpm = input->rpm.rpm[WHEEL_FRONT_RIGHT];
    } else if (config->drivetrain_mode == DRIVETRAIN_RWD) {
        left_rpm = input->rpm.rpm[WHEEL_REAR_LEFT];
        right_rpm = input->rpm.rpm[WHEEL_REAR_RIGHT];
    } else {
        left_rpm = (input->rpm.rpm[WHEEL_FRONT_LEFT] +
                    input->rpm.rpm[WHEEL_REAR_LEFT]) * 0.5f;
        right_rpm = (input->rpm.rpm[WHEEL_FRONT_RIGHT] +
                     input->rpm.rpm[WHEEL_REAR_RIGHT]) * 0.5f;
    }
    float average_rpm = (left_rpm + right_rpm) * 0.5f;
    if (average_rpm <= 0.0f) {
        set_inactive(state, output, "average wheel speed is zero");
        return;
    }
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

    float predicted_lateral_accel_mps2 = 0.0f;
    float lateral_demand = 0.0f;
    float front_relief = 0.0f;
    if (input->requested_mode == TV_MODE_FULL &&
        config->drivetrain_mode != DRIVETRAIN_RWD) {
        lateral_demand = cornering_lateral_demand(
            input->vehicle_speed_mps,
            input->average_wheel_angle_deg,
            DEFAULT_WHEELBASE_M,
            config->steering_lateral_accel_g,
            &predicted_lateral_accel_mps2);
        front_relief = ((float)config->tv_front_relief_percent / 100.0f) *
                       lateral_demand;
        if (front_relief > input->base_throttle) {
            front_relief = input->base_throttle;
        }
    }

    if (drivetrain_wheel_is_driven(config->drivetrain_mode, WHEEL_FRONT_LEFT)) {
        output->wheel_correction[WHEEL_FRONT_LEFT] =
            clampf(side_correction - front_relief,
                   -input->base_throttle, 1.0f - input->base_throttle);
        output->wheel_correction[WHEEL_FRONT_RIGHT] =
            clampf(-side_correction - front_relief,
                   -input->base_throttle, 1.0f - input->base_throttle);
    }
    if (drivetrain_wheel_is_driven(config->drivetrain_mode, WHEEL_REAR_LEFT)) {
        output->wheel_correction[WHEEL_REAR_LEFT] = side_correction;
        output->wheel_correction[WHEEL_REAR_RIGHT] = -side_correction;
    }
    output->target_yaw_rate_dps = target_yaw_rate_dps;
    output->yaw_error_dps = yaw_error_dps;
    output->side_rpm_error = side_rpm_error;
    output->predicted_lateral_accel_mps2 = predicted_lateral_accel_mps2;
    output->lateral_demand = lateral_demand;
    output->front_relief = front_relief;
    output->active_mode = input->requested_mode;
    output->active = true;
    output->inactive_reason = "active";
}
