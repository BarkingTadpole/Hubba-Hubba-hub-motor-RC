#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* Shared by captured calibrations and saved calibration validation. */
static inline bool calibration_pulse_valid(uint16_t pulse_us)
{
    return pulse_us >= 800 && pulse_us <= 2200;
}

static inline bool calibration_endpoints_valid(uint16_t first, uint16_t center,
                                               uint16_t second)
{
    int32_t first_delta = (int32_t)first - center;
    int32_t second_delta = (int32_t)second - center;
    return calibration_pulse_valid(first) && calibration_pulse_valid(center) &&
           calibration_pulse_valid(second) && abs(first_delta) >= 150 &&
           abs(second_delta) >= 150 && ((first_delta > 0) != (second_delta > 0));
}

static inline bool calibration_arm_positions_valid(uint16_t run, uint16_t stop_1,
                                                   uint16_t stop_2)
{
    return calibration_pulse_valid(run) && calibration_pulse_valid(stop_1) &&
           calibration_pulse_valid(stop_2) &&
           abs((int32_t)run - stop_1) >= 250 &&
           abs((int32_t)run - stop_2) >= 250 &&
           abs((int32_t)stop_1 - stop_2) >= 250;
}

static inline bool calibration_steering_valid(uint16_t left, uint16_t center,
                                             uint16_t right, uint16_t deadband)
{
    return calibration_endpoints_valid(left, center, right) && deadband <= 150 &&
           deadband < abs((int)left - center) && deadband < abs((int)right - center);
}
