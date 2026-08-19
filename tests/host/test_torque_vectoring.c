#include <stddef.h>

#include "torque_vectoring.h"

#define CHECK(condition)            \
    do {                            \
        if (!(condition)) {         \
            __builtin_trap();       \
        }                           \
    } while (0)

/* Keep the test executable independent of a host C runtime. */
void *memset(void *destination, int value, size_t length)
{
    unsigned char *bytes = destination;
    while (length-- > 0) {
        *bytes++ = (unsigned char)value;
    }
    return destination;
}

static drive_config_t valid_config(void)
{
    return (drive_config_t){
        .torque_vectoring_enabled = true,
        .tv_authority_percent = 10,
        .tv_front_relief_percent = 0,
        .steering_lateral_accel_g = 1.0f,
        .tv_turn_yaw_gain_dps = 180.0f,
        .tv_turn_rpm_gain = 0.20f,
        .tv_yaw_kp = 0.00025f,
        .tv_yaw_ki = 0.00004f,
        .tv_rpm_kp = 0.20f,
    };
}

static torque_vectoring_input_t valid_input(void)
{
    torque_vectoring_input_t input = {
        .requested_mode = TV_MODE_FULL,
        .direction = DRIVE_DIRECTION_FORWARD,
        .base_throttle = 0.5f,
        .steering = 0.5f,
        .dt_seconds = 0.02f,
        .imu = {
            .valid = true,
            .bias_calibrated = true,
        },
    };
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        input.rpm.valid[wheel] = true;
        input.rpm.rpm[wheel] = 1000.0f;
        input.rpm.frequency_hz[wheel] = 100.0f;
    }
    return input;
}

static void test_balanced_right_turn(void)
{
    drive_config_t config = valid_config();
    torque_vectoring_input_t input = valid_input();
    torque_vectoring_state_t state;
    torque_vectoring_output_t output;
    torque_vectoring_reset(&state);

    torque_vectoring_update(&state, &config, &input, &output);

    CHECK(output.active);
    CHECK(output.active_mode == TV_MODE_FULL);
    CHECK(output.target_yaw_rate_dps == 45.0f);
    CHECK(output.wheel_correction[WHEEL_FRONT_LEFT] > 0.0f);
    CHECK(output.wheel_correction[WHEEL_FRONT_LEFT] ==
          output.wheel_correction[WHEEL_REAR_LEFT]);
    CHECK(output.wheel_correction[WHEEL_FRONT_RIGHT] ==
          -output.wheel_correction[WHEEL_FRONT_LEFT]);
    CHECK(output.wheel_correction[WHEEL_FRONT_LEFT] <= 0.1f);
}

static void test_frequency_guard(void)
{
    drive_config_t config = valid_config();
    torque_vectoring_input_t input = valid_input();
    torque_vectoring_state_t state;
    torque_vectoring_output_t output;
    torque_vectoring_reset(&state);
    input.rpm.frequency_hz[WHEEL_REAR_RIGHT] = 10.0f;

    torque_vectoring_update(&state, &config, &input, &output);

    CHECK(!output.active);
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        CHECK(output.wheel_correction[wheel] == 0.0f);
    }
}

static void test_non_finite_config_falls_back(void)
{
    drive_config_t config = valid_config();
    torque_vectoring_input_t input = valid_input();
    torque_vectoring_state_t state;
    torque_vectoring_output_t output;
    torque_vectoring_reset(&state);
    config.tv_yaw_kp = __builtin_nanf("");

    torque_vectoring_update(&state, &config, &input, &output);

    CHECK(!output.active);
    CHECK(state.yaw_integral == 0.0f);
}

static void test_integrator_does_not_wind_further_into_saturation(void)
{
    drive_config_t config = valid_config();
    torque_vectoring_input_t input = valid_input();
    torque_vectoring_state_t state;
    torque_vectoring_output_t output;
    torque_vectoring_reset(&state);
    config.tv_yaw_kp = 0.01f;
    config.tv_yaw_ki = 0.01f;
    config.tv_rpm_kp = 0.0f;
    input.imu.yaw_rate_dps = -1000.0f;

    torque_vectoring_update(&state, &config, &input, &output);

    CHECK(output.active);
    CHECK(output.wheel_correction[WHEEL_FRONT_LEFT] == 0.1f);
    CHECK(state.yaw_integral == 0.0f);
}

static void test_full_mode_reduces_front_torque_at_lateral_limit(void)
{
    drive_config_t config = valid_config();
    torque_vectoring_input_t input = valid_input();
    torque_vectoring_state_t state;
    torque_vectoring_output_t output;
    torque_vectoring_reset(&state);
    config.tv_front_relief_percent = 20;
    input.vehicle_speed_mps = 10.0f;
    input.average_wheel_angle_deg = 5.0f;

    torque_vectoring_update(&state, &config, &input, &output);

    CHECK(output.active);
    CHECK(output.lateral_demand == 1.0f);
    CHECK(output.front_relief == 0.2f);
    CHECK(output.wheel_correction[WHEEL_FRONT_LEFT] <
          output.wheel_correction[WHEEL_REAR_LEFT]);
    CHECK(output.wheel_correction[WHEEL_FRONT_RIGHT] <
          output.wheel_correction[WHEEL_REAR_RIGHT]);
}

int main(void)
{
    test_balanced_right_turn();
    test_frequency_guard();
    test_non_finite_config_falls_back();
    test_integrator_does_not_wind_further_into_saturation();
    test_full_mode_reduces_front_torque_at_lateral_limit();
    return 0;
}
