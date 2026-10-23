#include "cornering_control.h"

#include <math.h>
#include <stddef.h>

#define CORNERING_PI 3.14159265358979323846f

bool cornering_vehicle_speed_from_rear_rpm(const rpm_snapshot_t *rpm,
                                            float wheel_diameter_m,
                                            float *speed_mps)
{
    if (rpm == NULL || speed_mps == NULL || !isfinite(wheel_diameter_m) ||
        wheel_diameter_m <= 0.0f ||
        !rpm->valid[RPM_REAR_LEFT] || !rpm->valid[RPM_REAR_RIGHT] ||
        !isfinite(rpm->rpm[RPM_REAR_LEFT]) ||
        !isfinite(rpm->rpm[RPM_REAR_RIGHT]) ||
        rpm->rpm[RPM_REAR_LEFT] < 0.0f ||
        rpm->rpm[RPM_REAR_RIGHT] < 0.0f) {
        return false;
    }

    float average_rear_rpm = (rpm->rpm[RPM_REAR_LEFT] +
                              rpm->rpm[RPM_REAR_RIGHT]) * 0.5f;
    *speed_mps = average_rear_rpm * CORNERING_PI * wheel_diameter_m / 60.0f;
    return isfinite(*speed_mps);
}
