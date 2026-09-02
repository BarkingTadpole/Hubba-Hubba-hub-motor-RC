#include "drivetrain_control.h"

#include <stddef.h>
#include <stdint.h>

bool drivetrain_mode_valid(drivetrain_mode_t mode)
{
    return mode == DRIVETRAIN_AWD || mode == DRIVETRAIN_FWD ||
           mode == DRIVETRAIN_RWD;
}

const char *drivetrain_mode_name(drivetrain_mode_t mode)
{
    switch (mode) {
    case DRIVETRAIN_AWD: return "AWD";
    case DRIVETRAIN_FWD: return "FWD";
    case DRIVETRAIN_RWD: return "RWD";
    default: return "UNKNOWN";
    }
}

bool drivetrain_wheel_is_driven(drivetrain_mode_t mode, wheel_id_t wheel)
{
    if (!drivetrain_mode_valid(mode) || wheel < WHEEL_FRONT_LEFT ||
        wheel > WHEEL_REAR_RIGHT) {
        return false;
    }
    if (mode == DRIVETRAIN_AWD) {
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
