#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "independent_steering.h"

static const float rad = 0.017453292519943295f;
static void near(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) < tolerance);
}

int main(void)
{
    independent_steering_config_t c;
    independent_steering_defaults(&c);
    assert(independent_steering_config_valid(&c));
    assert(c.geometry.wheelbase_tenths_mm == 4450 && c.geometry.track_tenths_mm == 3200);
    uint16_t p[2]; float a[2];
    c.point_count = 3;
    c.points[0] = (ackermann_point_t){0, 1000};
    c.points[1] = (ackermann_point_t){200, 400};
    c.points[2] = (ackermann_point_t){600, 0};
    near(ackermann_strength_percent(&c, 0), 100, .001f);
    near(ackermann_strength_percent(&c, 10), 70, .001f);
    near(ackermann_strength_percent(&c, 20), 40, .001f);
    near(ackermann_strength_percent(&c, 40), 20, .001f);
    near(ackermann_strength_percent(&c, 300), 0, .001f);
    assert(ackermann_strength_percent(&c, NAN) == 0);
    assert(ackermann_strength_percent(&c, -1) == 0);
    // 0% gives equal road-wheel targets, even with asymmetric PWM/angle calibration.
    c.servo[0] = (steering_servo_config_t){2100, 1520, 1100, 400, 300};
    c.servo[1] = (steering_servo_config_t){900, 1480, 1900, 300, 600};
    independent_steering_mix(&c, .5f, 0, p, a);
    near(a[0], 15, .0001f); near(a[1], 15, .0001f);
    assert(p[0] == 1310 && p[1] == 1585);
    independent_steering_mix(&c, -1, 0, p, a);
    near(a[0], -30, .0001f); near(a[1], -30, .0001f);
    assert(p[0] == 1955 && p[1] == 900);
    // Independent oracle: the rear-axle turn center inferred from both front
    // wheels must coincide at 100% Ackermann (cotangent identity).
    for (int sign = -1; sign <= 1; sign += 2) {
        int inside = sign < 0 ? 0 : 1, outside = 1 - inside;
        for (int pct = 1; pct <= 100; pct++) {
            independent_steering_mix(&c, sign * pct / 100.0f, 100, p, a);
            float inner = fabsf(a[inside]), outer = fabsf(a[outside]);
            assert(outer < inner);
            near(inner, .3f * pct, .0001f);
            float radius_in = 445.0f / tanf(inner * rad) + 160.0f;
            float radius_out = 445.0f / tanf(outer * rad) - 160.0f;
            near(radius_in, radius_out, .05f);
            float ideal_outer = outer;
            independent_steering_mix(&c, sign * pct / 100.0f, 50, p, a);
            near(fabsf(a[outside]), (inner + ideal_outer) / 2, .0001f);
        }
    }
    // More track increases split; more wheelbase decreases it. Small turns
    // approach parallel, so a fixed outside/inside PWM ratio cannot substitute.
    independent_steering_mix(&c, 1, 100, p, a); float outer = a[0];
    c.geometry.track_tenths_mm = 5000;
    independent_steering_mix(&c, 1, 100, p, a); assert(a[0] < outer);
    c.geometry.wheelbase_tenths_mm = 10000;
    independent_steering_mix(&c, 1, 100, p, a); assert(a[0] > outer);
    float large_ratio = a[0] / a[1];
    independent_steering_mix(&c, .01f, 100, p, a); assert(a[0] / a[1] > large_ratio);
    c.geometry.max_left_tenths_deg = 200;
    independent_steering_mix(&c, -1, 100, p, a); near(a[0], -20, .0001f);
    for (int strength = 0; strength <= 100; strength++) {
        for (int cmd = -120; cmd <= 120; cmd++) {
            independent_steering_mix(&c, cmd / 100.0f, strength, p, a);
            assert(p[0] >= 1100 && p[0] <= 2100 && p[1] >= 900 && p[1] <= 1900);
            assert(isfinite(a[0]) && isfinite(a[1]));
            if (cmd == 0) assert(p[0] == 1520 && p[1] == 1480 && a[0] == 0 && a[1] == 0);
        }
    }
    independent_steering_mix(&c, INFINITY, 50, p, a);
    assert(p[0] == 1520 && p[1] == 1480 && a[0] == 0 && a[1] == 0);
    independent_steering_mix(&c, 1, NAN, p, NULL);
    assert(p[0] == 1520 && p[1] == 1480);
    independent_steering_mix(&c, 1, -1, p, NULL);
    assert(p[0] == 1520 && p[1] == 1480);
    c.geometry.max_left_tenths_deg = 850; // Exceeds calibrated travel.
    assert(!independent_steering_config_valid(&c));
    independent_steering_mix(&c, 1, 100, p, a);
    assert(p[0] == 1520 && p[1] == 1480 && a[0] == 0);
    independent_steering_defaults(&c);
    c.geometry.wheelbase_tenths_mm = 0;
    assert(!independent_steering_config_valid(&c));
    independent_steering_defaults(&c);
    c.points[1].speed_tenths_kmh = 0;
    assert(!ackermann_curve_valid(c.points, 2));
    assert(!ackermann_curve_valid(c.points, 9));
    c.servo[0].left_tenths_deg = 0; assert(!steering_servo_config_valid(&c.servo[0]));
    independent_steering_mix(NULL, 1, 100, p, a);
    assert(p[0] == 1500 && p[1] == 1500 && a[0] == 0);
    puts("Independent steering: Ackermann turn-center identity, blend, angle/PWM calibration, geometry, interpolation and bounds passed.");
    return 0;
}
