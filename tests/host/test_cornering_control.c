#include <stddef.h>

#include "cornering_control.h"

#define CHECK(condition)            \
    do {                            \
        if (!(condition)) {         \
            __builtin_trap();       \
        }                           \
    } while (0)

static float absolute(float value)
{
    return value < 0.0f ? -value : value;
}

static bool near(float actual, float expected, float tolerance)
{
    return absolute(actual - expected) <= tolerance;
}

static void test_rear_rpm_to_vehicle_speed(void)
{
    rpm_snapshot_t rpm = {0};
    rpm.valid[WHEEL_REAR_LEFT] = true;
    rpm.valid[WHEEL_REAR_RIGHT] = true;
    rpm.rpm[WHEEL_REAR_LEFT] = 1000.0f;
    rpm.rpm[WHEEL_REAR_RIGHT] = 1000.0f;
    float speed_mps = 0.0f;

    CHECK(cornering_vehicle_speed_from_rear_rpm(&rpm, 0.107f, &speed_mps));
    CHECK(near(speed_mps * 3.6f, 20.169f, 0.002f));

    rpm.valid[WHEEL_REAR_RIGHT] = false;
    CHECK(!cornering_vehicle_speed_from_rear_rpm(&rpm, 0.107f, &speed_mps));
}

static void test_one_g_steering_envelope(void)
{
    float speed_mps = 20.0f / 3.6f;
    float angle_deg = cornering_max_average_wheel_angle_deg(
        speed_mps, 0.445f, 1.0f, 31.079f);
    CHECK(near(angle_deg, 8.05f, 0.02f));

    angle_deg = cornering_max_average_wheel_angle_deg(
        0.0f, 0.445f, 1.0f, 31.079f);
    CHECK(angle_deg == 31.079f);
}

static void test_lateral_demand_clamps_at_limit(void)
{
    float predicted_mps2 = 0.0f;
    float demand = cornering_lateral_demand(
        10.0f, 5.0f, 0.445f, 1.0f, &predicted_mps2);
    CHECK(predicted_mps2 > 9.80665f);
    CHECK(demand == 1.0f);
}

int main(void)
{
    test_rear_rpm_to_vehicle_speed();
    test_one_g_steering_envelope();
    test_lateral_demand_clamps_at_limit();
    return 0;
}
