#include <assert.h>
#include <math.h>
#include <stddef.h>

#include "cornering_control.h"

int main(void)
{
    rpm_snapshot_t rpm = {.rpm = {800, 1200}, .valid = {true, true}};
    float speed_mps;
    assert(cornering_vehicle_speed_from_rear_rpm(&rpm, 0.107f, &speed_mps));
    assert(fabsf(speed_mps * 3.6f - 20.169f) < 0.002f);
    rpm.valid[RPM_REAR_RIGHT] = false;
    assert(!cornering_vehicle_speed_from_rear_rpm(&rpm, 0.107f, &speed_mps));
    rpm.valid[RPM_REAR_RIGHT] = true;
    rpm.rpm[RPM_REAR_LEFT] = NAN;
    assert(!cornering_vehicle_speed_from_rear_rpm(&rpm, 0.107f, &speed_mps));
    rpm.rpm[RPM_REAR_LEFT] = -1;
    assert(!cornering_vehicle_speed_from_rear_rpm(&rpm, 0.107f, &speed_mps));
    return 0;
}
