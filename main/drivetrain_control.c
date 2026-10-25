#include "drivetrain_control.h"

#include <stddef.h>
#include <stdint.h>

bool drivetrain_mode_valid(drivetrain_mode_t mode)
{
    return mode == DRIVETRAIN_AWD || mode == DRIVETRAIN_FWD ||
           mode == DRIVETRAIN_RWD || mode == DRIVETRAIN_TANK;
}

const char *drivetrain_mode_name(drivetrain_mode_t mode)
{
    switch (mode) {
    case DRIVETRAIN_AWD: return "AWD";
    case DRIVETRAIN_FWD: return "FWD";
    case DRIVETRAIN_RWD: return "RWD";
    case DRIVETRAIN_TANK: return "TANK";
    default: return "UNKNOWN";
    }
}

bool drivetrain_wheel_is_driven(drivetrain_mode_t mode, wheel_id_t wheel)
{
    if (!drivetrain_mode_valid(mode) || wheel < WHEEL_FRONT_LEFT ||
        wheel > WHEEL_REAR_RIGHT) {
        return false;
    }
    if (mode == DRIVETRAIN_AWD || mode == DRIVETRAIN_TANK) {
        return true;
    }
    bool front_wheel = wheel == WHEEL_FRONT_LEFT || wheel == WHEEL_FRONT_RIGHT;
    return mode == DRIVETRAIN_FWD ? front_wheel : !front_wheel;
}

bool drivetrain_apply_output_mask(drivetrain_mode_t mode,
                                  const esc_limits_t *limits,
                                  wheel_output_command_t *command)
{
    if (limits == NULL || command == NULL) {
        return false;
    }

    bool mode_valid = drivetrain_mode_valid(mode);
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        if (!mode_valid ||
            !drivetrain_wheel_is_driven(mode, (wheel_id_t)wheel)) {
            command->throttle_us[wheel] = limits->throttle_min_us;
            command->reverse_us[wheel] = limits->reverse_low_us;
        }
    }
    return mode_valid;
}

bool drivetrain_mix_outputs(drivetrain_mode_t mode, drive_direction_t direction,
                             uint16_t throttle_us, const esc_limits_t *limits,
                             wheel_output_command_t *command)
{
    if (limits == NULL || command == NULL) return false;
    bool tank = mode == DRIVETRAIN_TANK;
    bool pivot = direction == DRIVE_DIRECTION_TANK_LEFT || direction == DRIVE_DIRECTION_TANK_RIGHT;
    bool linear = direction == DRIVE_DIRECTION_FORWARD || direction == DRIVE_DIRECTION_REVERSE;
    bool valid = drivetrain_mode_valid(mode) && throttle_us >= limits->throttle_min_us &&
                 throttle_us <= limits->throttle_max_us &&
                 (tank ? (pivot || (linear && throttle_us == limits->throttle_min_us)) : linear);
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        bool left = wheel == WHEEL_FRONT_LEFT || wheel == WHEEL_REAR_LEFT;
        bool reverse = pivot ? (left == (direction == DRIVE_DIRECTION_TANK_LEFT))
                             : direction == DRIVE_DIRECTION_REVERSE;
        bool driven = valid && drivetrain_wheel_is_driven(mode, (wheel_id_t)wheel);
        command->throttle_us[wheel] = driven ? throttle_us : limits->throttle_min_us;
        command->reverse_us[wheel] = driven && reverse ? limits->reverse_high_us : limits->reverse_low_us;
    }
    return valid;
}

uint16_t drivetrain_ramp_step_us(uint16_t smoothest_step_us,
                                 uint8_t smoothing_percent)
{
    if (smoothest_step_us == 0) return 0;
    if (smoothing_percent == 0) return UINT16_MAX;
    if (smoothing_percent > 100) smoothing_percent = 100;

    uint32_t numerator = (uint32_t)smoothest_step_us * 100U;
    uint32_t step_us = (numerator + smoothing_percent - 1U) /
                       smoothing_percent;
    return step_us > UINT16_MAX ? UINT16_MAX : (uint16_t)step_us;
}
