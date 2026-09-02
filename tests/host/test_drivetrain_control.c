#include "drivetrain_control.h"

#define CHECK(condition)            \
    do {                            \
        if (!(condition)) {         \
            __builtin_trap();       \
        }                           \
    } while (0)

static const esc_limits_t limits = {
    .throttle_min_us = 1100,
    .throttle_max_us = 1940,
    .reverse_low_us = 1100,
    .reverse_high_us = 1940,
};

static wheel_output_command_t active_command(void)
{
    wheel_output_command_t command;
    for (unsigned int wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        command.throttle_us[wheel] = 1500 + wheel;
        command.reverse_us[wheel] = 1940;
    }
    return command;
}

static void test_awd_keeps_all_outputs(void)
{
    wheel_output_command_t command = active_command();
    CHECK(drivetrain_apply_output_mask(DRIVETRAIN_AWD, &limits, &command));
    for (unsigned int wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        CHECK(command.throttle_us[wheel] == 1500 + wheel);
        CHECK(command.reverse_us[wheel] == 1940);
    }
}

static void test_fwd_safes_rear_axle(void)
{
    wheel_output_command_t command = active_command();
    CHECK(drivetrain_apply_output_mask(DRIVETRAIN_FWD, &limits, &command));
    CHECK(command.throttle_us[WHEEL_FRONT_LEFT] == 1500);
    CHECK(command.throttle_us[WHEEL_FRONT_RIGHT] == 1501);
    CHECK(command.throttle_us[WHEEL_REAR_LEFT] == 1100);
    CHECK(command.throttle_us[WHEEL_REAR_RIGHT] == 1100);
    CHECK(command.reverse_us[WHEEL_REAR_LEFT] == 1100);
    CHECK(command.reverse_us[WHEEL_REAR_RIGHT] == 1100);
}

static void test_rwd_safes_front_axle(void)
{
    wheel_output_command_t command = active_command();
    CHECK(drivetrain_apply_output_mask(DRIVETRAIN_RWD, &limits, &command));
    CHECK(command.throttle_us[WHEEL_FRONT_LEFT] == 1100);
    CHECK(command.throttle_us[WHEEL_FRONT_RIGHT] == 1100);
    CHECK(command.reverse_us[WHEEL_FRONT_LEFT] == 1100);
    CHECK(command.reverse_us[WHEEL_FRONT_RIGHT] == 1100);
    CHECK(command.throttle_us[WHEEL_REAR_LEFT] == 1502);
    CHECK(command.throttle_us[WHEEL_REAR_RIGHT] == 1503);
}

static void test_invalid_mode_safes_every_output(void)
{
    wheel_output_command_t command = active_command();
    CHECK(!drivetrain_apply_output_mask((drivetrain_mode_t)99, &limits, &command));
    for (unsigned int wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        CHECK(command.throttle_us[wheel] == 1100);
        CHECK(command.reverse_us[wheel] == 1100);
    }
}

static void test_drive_smoothing_scales_both_ramps(void)
{
    CHECK(drivetrain_ramp_step_us(12, 100) == 12);
    CHECK(drivetrain_ramp_step_us(100, 100) == 100);
    CHECK(drivetrain_ramp_step_us(12, 50) == 24);
    CHECK(drivetrain_ramp_step_us(100, 50) == 200);
    CHECK(drivetrain_ramp_step_us(12, 25) == 48);
    CHECK(drivetrain_ramp_step_us(100, 25) == 400);
    CHECK(drivetrain_ramp_step_us(12, 0) == UINT16_MAX);
    CHECK(drivetrain_ramp_step_us(100, 0) == UINT16_MAX);
    CHECK(drivetrain_ramp_step_us(12, 101) == 12);
}

int main(void)
{
    test_awd_keeps_all_outputs();
    test_fwd_safes_rear_axle();
    test_rwd_safes_front_axle();
    test_invalid_mode_safes_every_output();
    test_drive_smoothing_scales_both_ramps();
    return 0;
}
