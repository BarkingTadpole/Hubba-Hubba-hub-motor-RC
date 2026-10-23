#pragma once

#include <stdint.h>

#include "driver/ledc.h"

/* Both LEDC groups use the same 50 Hz, 16-bit pulse conversion. */
#define PWM_FRAME_HZ 50
#define PWM_FRAME_US 20000
#define PWM_DUTY_RES LEDC_TIMER_16_BIT
#define PWM_MAX_DUTY ((1U << 16) - 1U)

static inline uint32_t pwm_pulse_to_duty(uint16_t pulse_us)
{
    return ((uint32_t)pulse_us * PWM_MAX_DUTY) / PWM_FRAME_US;
}

static inline uint16_t pwm_clamp_pulse(uint16_t pulse_us, uint16_t minimum,
                                      uint16_t maximum)
{
    if (pulse_us < minimum) return minimum;
    if (pulse_us > maximum) return maximum;
    return pulse_us;
}
