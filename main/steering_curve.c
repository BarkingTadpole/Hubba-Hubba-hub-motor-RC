#include "steering_curve.h"

#include <math.h>
#include <stddef.h>

/*
 * Corrected steering data supplied 2026-08-10. Source servo-positive steers
 * the vehicle left. Wheel angles use wheel-local signs: positive points each
 * wheel outward and negative points it inward. Runtime steering and wheel
 * headings are positive-right, so the lookup input and each wheel output need
 * the coordinate conversions applied in steering_curve_sample().
 *
 * The centered raw values (+1.64, +1.64 degrees) describe symmetric toe-out.
 * In the common runtime heading frame they become LF=-1.64 and RF=+1.64,
 * leaving the mean steering angle exactly zero.
 */
#define STEERING_CURVE_POINT_COUNT 45
#define STEERING_CURVE_RIGHT_MAX_AVERAGE_DEG 29.738f
#define STEERING_CURVE_LEFT_MAX_AVERAGE_DEG 31.079f

static const float source_servo_deg[STEERING_CURVE_POINT_COUNT] = {
    -45.0f, -43.0f, -40.0f, -38.0f, -36.0f, -34.0f, -32.0f, -30.0f,
    -28.0f, -26.0f, -24.0f, -22.0f, -20.0f, -18.0f, -16.0f, -14.0f,
    -12.0f, -10.0f,  -8.0f,  -6.0f,  -4.0f,  -2.0f,   0.0f,   2.0f,
      4.0f,   6.0f,   8.0f,  10.0f,  12.0f,  14.0f,  16.0f,  18.0f,
     20.0f,  22.0f,  24.0f,  26.0f,  28.0f,  30.0f,  32.0f,  34.0f,
     36.0f,  38.0f,  40.0f,  43.0f,  45.0f,
};

static const float source_left_wheel_deg[STEERING_CURVE_POINT_COUNT] = {
    -29.906f, -28.795f, -27.057f, -25.851f, -24.609f, -23.334f,
    -22.025f, -20.685f, -19.316f, -17.921f, -16.500f, -15.056f,
    -13.591f, -12.109f, -10.610f,  -9.099f,  -7.576f,  -6.045f,
     -4.508f,  -2.968f,  -1.427f,   0.000f,   1.640f,   3.176f,
      4.696f,   6.206f,   7.703f,   9.185f,  10.650f,  12.097f,
     13.523f,  14.927f,  16.307f,  17.662f,  18.990f,  20.290f,
     21.559f,  22.798f,  24.003f,  25.174f,  26.310f,  27.407f,
     28.466f,  29.976f,  30.929f,
};

static const float source_right_wheel_deg[STEERING_CURVE_POINT_COUNT] = {
     29.570f,  28.486f,  26.864f,  25.780f,  24.690f,  23.592f,
     22.428f,  21.358f,  20.217f,  19.059f,  17.880f,  16.679f,
     15.455f,  14.205f,  12.928f,  11.623f,  10.829f,   8.924f,
      7.529f,   6.103f,   4.646f,   3.158f,   1.640f,   0.000f,
     -1.480f,  -3.078f,  -4.698f,  -6.335f,  -7.986f,  -9.645f,
    -11.306f, -12.964f, -14.610f, -16.237f, -17.838f, -19.403f,
    -20.927f, -22.399f, -23.814f, -25.164f, -26.444f, -27.649f,
    -28.774f, -30.309f, -31.229f,
};

static float interpolate(const float *values, float servo_deg)
{
    size_t lower = 0;
    size_t upper = STEERING_CURVE_POINT_COUNT - 1;
    while ((upper - lower) > 1) {
        size_t middle = lower + ((upper - lower) / 2);
        if (servo_deg < source_servo_deg[middle]) {
            upper = middle;
        } else {
            lower = middle;
        }
    }

    float span = source_servo_deg[upper] - source_servo_deg[lower];
    float fraction = (servo_deg - source_servo_deg[lower]) / span;
    return values[lower] + ((values[upper] - values[lower]) * fraction);
}

bool steering_curve_sample(float servo_command_deg, steering_curve_sample_t *sample)
{
    if (sample == NULL) {
        return false;
    }
    *sample = (steering_curve_sample_t){0};
    if (!isfinite(servo_command_deg)) {
        return false;
    }

    float source_command_deg = -servo_command_deg;
    if (source_command_deg < source_servo_deg[0]) {
        source_command_deg = source_servo_deg[0];
        sample->saturated = true;
    } else if (source_command_deg > source_servo_deg[STEERING_CURVE_POINT_COUNT - 1]) {
        source_command_deg = source_servo_deg[STEERING_CURVE_POINT_COUNT - 1];
        sample->saturated = true;
    }

    sample->servo_command_deg = -source_command_deg;
    sample->left_wheel_deg = -interpolate(source_left_wheel_deg, source_command_deg);
    sample->right_wheel_deg = interpolate(source_right_wheel_deg, source_command_deg);
    sample->average_wheel_deg =
        (sample->left_wheel_deg + sample->right_wheel_deg) * 0.5f;
    if (sample->average_wheel_deg > 0.0f) {
        sample->normalized_average = sample->average_wheel_deg /
                                     STEERING_CURVE_RIGHT_MAX_AVERAGE_DEG;
    } else if (sample->average_wheel_deg < 0.0f) {
        sample->normalized_average = sample->average_wheel_deg /
                                     STEERING_CURVE_LEFT_MAX_AVERAGE_DEG;
    }
    sample->valid = true;
    return true;
}
