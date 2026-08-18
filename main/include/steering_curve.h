#pragma once

#include <stdbool.h>

typedef struct {
    float servo_command_deg;
    float left_wheel_deg;
    float right_wheel_deg;
    float average_wheel_deg;
    float normalized_average;
    bool valid;
    bool saturated;
} steering_curve_sample_t;

bool steering_curve_sample(float servo_command_deg, steering_curve_sample_t *sample);
