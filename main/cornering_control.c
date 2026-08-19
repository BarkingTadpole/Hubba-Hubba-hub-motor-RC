#include "cornering_control.h"

#include <math.h>

#define CORNERING_PI 3.14159265358979323846f
#define STANDARD_GRAVITY_MPS2 9.80665f

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

bool cornering_vehicle_speed_from_rear_rpm(const rpm_snapshot_t *rpm,
                                            float wheel_diameter_m,
                                            float *speed_mps)
{
    if (rpm == NULL || speed_mps == NULL || !isfinite(wheel_diameter_m) ||
        wheel_diameter_m <= 0.0f ||
        !rpm->valid[WHEEL_REAR_LEFT] || !rpm->valid[WHEEL_REAR_RIGHT] ||
        !isfinite(rpm->rpm[WHEEL_REAR_LEFT]) ||
        !isfinite(rpm->rpm[WHEEL_REAR_RIGHT]) ||
        rpm->rpm[WHEEL_REAR_LEFT] < 0.0f ||
        rpm->rpm[WHEEL_REAR_RIGHT] < 0.0f) {
        return false;
    }

    float average_rear_rpm = (rpm->rpm[WHEEL_REAR_LEFT] +
                              rpm->rpm[WHEEL_REAR_RIGHT]) * 0.5f;
    *speed_mps = average_rear_rpm * CORNERING_PI * wheel_diameter_m / 60.0f;
    return isfinite(*speed_mps);
}

float cornering_max_average_wheel_angle_deg(float speed_mps,
                                             float wheelbase_m,
                                             float lateral_accel_limit_g,
                                             float physical_max_angle_deg)
{
    if (!isfinite(speed_mps) || !isfinite(wheelbase_m) ||
        !isfinite(lateral_accel_limit_g) || !isfinite(physical_max_angle_deg) ||
        speed_mps < 0.0f || wheelbase_m <= 0.0f ||
        lateral_accel_limit_g <= 0.0f || physical_max_angle_deg <= 0.0f) {
        return 0.0f;
    }
    if (speed_mps < 0.1f) {
        return physical_max_angle_deg;
    }

    float lateral_accel_limit = lateral_accel_limit_g * STANDARD_GRAVITY_MPS2;
    float angle_rad = atanf((wheelbase_m * lateral_accel_limit) /
                            (speed_mps * speed_mps));
    float angle_deg = angle_rad * (180.0f / CORNERING_PI);
    return clampf(angle_deg, 0.0f, physical_max_angle_deg);
}

float cornering_lateral_demand(float speed_mps,
                               float average_wheel_angle_deg,
                               float wheelbase_m,
                               float lateral_accel_limit_g,
                               float *predicted_lateral_accel_mps2)
{
    if (predicted_lateral_accel_mps2 != NULL) {
        *predicted_lateral_accel_mps2 = 0.0f;
    }
    if (!isfinite(speed_mps) || !isfinite(average_wheel_angle_deg) ||
        !isfinite(wheelbase_m) || !isfinite(lateral_accel_limit_g) ||
        speed_mps < 0.0f || wheelbase_m <= 0.0f || lateral_accel_limit_g <= 0.0f) {
        return 0.0f;
    }

    float angle_rad = fabsf(average_wheel_angle_deg) * (CORNERING_PI / 180.0f);
    float lateral_accel = speed_mps * speed_mps * tanf(angle_rad) / wheelbase_m;
    if (!isfinite(lateral_accel) || lateral_accel < 0.0f) {
        return 0.0f;
    }
    if (predicted_lateral_accel_mps2 != NULL) {
        *predicted_lateral_accel_mps2 = lateral_accel;
    }
    return clampf(lateral_accel / (lateral_accel_limit_g * STANDARD_GRAVITY_MPS2),
                  0.0f, 1.0f);
}
