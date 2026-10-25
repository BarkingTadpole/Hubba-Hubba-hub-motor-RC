#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "independent_steering.h"

static const float rad = 0.017453292519943295f;
static void near(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) < tolerance);
}

static void test_servo_trim(void)
{
    independent_steering_config_t c;
    independent_steering_defaults(&c);
    assert(c.trim_us[0] == 0 && c.trim_us[1] == 0);
    c.servo[1] = (steering_servo_config_t){1000,1480,2000,300,300};
    c.trim_us[0] = 40;
    c.trim_us[1] = -30;
    assert(independent_steering_config_valid(&c));
    uint16_t p[2]; float a[2];
    independent_steering_mix(&c, 0, 100, p, a);
    assert(p[0] == 1540 && p[1] == 1450 && a[0] == 0 && a[1] == 0);
    independent_steering_mix(&c, .5f, 0, p, a);
    assert(p[0] == 1270 && p[1] == 1725 && a[0] == 15 && a[1] == 15);
    independent_steering_mix(&c, -.5f, 0, p, a);
    assert(p[0] == 1770 && p[1] == 1225 && a[0] == -15 && a[1] == -15);
    independent_steering_mix(&c, -1, 0, p, a);
    assert(p[0] == 2000 && p[1] == 1000);
    independent_steering_mix(&c, 1, 0, p, a);
    assert(p[0] == 1000 && p[1] == 2000);
    // Trims change calibration, not geometric angle targets or inside selection.
    independent_steering_config_t untrimmed = c;
    untrimmed.trim_us[0] = untrimmed.trim_us[1] = 0;
    for (int strength = 0; strength <= 100; strength += 10) {
        for (int cmd = -100; cmd <= 100; cmd++) {
            uint16_t reference_p[2]; float reference_a[2];
            independent_steering_mix(&c, cmd / 100.0f, strength, p, a);
            independent_steering_mix(&untrimmed, cmd / 100.0f, strength, reference_p, reference_a);
            for (int i = 0; i < 2; i++) {
                assert(p[i] >= 1000 && p[i] <= 2000);
                assert(a[i] == reference_a[i]);
            }
        }
    }
    independent_steering_mix(&c, NAN, 100, p, a);
    assert(p[0] == 1540 && p[1] == 1450 && a[0] == 0 && a[1] == 0);
    c.trim_us[0] = 450; c.trim_us[1] = -430;
    assert(independent_steering_config_valid(&c));
    c.trim_us[0] = 451; assert(!independent_steering_config_valid(&c));
    c.trim_us[0] = -450; assert(independent_steering_config_valid(&c));
    c.trim_us[1] = -431; assert(!independent_steering_config_valid(&c));
    c.trim_us[1] = 470; assert(independent_steering_config_valid(&c));
    c.trim_us[1] = 471; assert(!independent_steering_config_valid(&c));
    c.trim_us[0] = INT16_MAX; c.trim_us[1] = INT16_MIN;
    assert(!independent_steering_config_valid(&c));
    independent_steering_mix(&c, 1, 100, p, a);
    assert(p[0] == 1500 && p[1] == 1480 && a[0] == 0 && a[1] == 0);
    independent_steering_defaults(&c);
    c.servo[0] = (steering_servo_config_t){2500,550,500,300,300};
    c.servo[1] = (steering_servo_config_t){500,2450,2500,300,300};
    c.trim_us[0] = STEERING_TRIM_MAX_US; c.trim_us[1] = -STEERING_TRIM_MAX_US;
    assert(independent_steering_config_valid(&c));
    independent_steering_mix(&c, 0, 0, p, a);
    assert(p[0] == 2450 && p[1] == 550);
}

static void test_180_degree_servo_mounting(void)
{
    independent_steering_config_t c;
    independent_steering_defaults(&c);
    for (int i = 0; i < 2; i++) {
        assert(c.servo[i].left_us == 2000 && c.servo[i].center_us == 1500 && c.servo[i].right_us == 1000);
        assert(c.servo[i].left_tenths_deg == 300 && c.servo[i].right_tenths_deg == 300);
        // Full shaft range is allowed after calibration; road-wheel angles remain measured inputs.
        c.servo[i].left_us = 2500;
        c.servo[i].right_us = 500;
    }
    assert(independent_steering_config_valid(&c));
    uint16_t p[2]; float a[2];
    for (int sign = -1; sign <= 1; sign += 2) {
        for (int pct = 1; pct <= 100; pct++) {
            independent_steering_mix(&c, sign * pct / 100.0f, 100, p, a);
            if (sign < 0) {
                // Left inside CCW; right outside CCW. Higher PWM means CCW.
                assert(p[0] >= p[1] && p[1] > 1500);
                assert(a[0] < a[1] && a[1] < 0);
            } else {
                // Left outside CW; right inside CW. Lower PWM means CW.
                assert(p[1] <= p[0] && p[0] < 1500);
                assert(a[1] > a[0] && a[0] > 0);
            }
        }
    }
    c.linkage.outside_tenths_deg = 900; // Full parallel travel explicitly permitted.
    independent_steering_mix(&c, -1, 0, p, a);
    assert(p[0] == 2500 && p[1] == 2500 && a[0] == -30 && a[1] == -30);
    independent_steering_mix(&c, 1, 0, p, a);
    assert(p[0] == 500 && p[1] == 500 && a[0] == 30 && a[1] == 30);
    c.trim_us[0] = 950; c.trim_us[1] = -950;
    assert(independent_steering_config_valid(&c));
    independent_steering_mix(&c, 0, 0, p, a);
    assert(p[0] == 2450 && p[1] == 550);
    c.trim_us[0] = 951; assert(!independent_steering_config_valid(&c)); c.trim_us[0] = 0;
    c.trim_us[1] = -951; assert(!independent_steering_config_valid(&c)); c.trim_us[1] = 0;
    c.servo[0].right_us = 499; assert(!independent_steering_config_valid(&c)); c.servo[0].right_us = 500;
    c.servo[1].left_us = 2501; assert(!independent_steering_config_valid(&c)); c.servo[1].left_us = 2500;
    c.servo[0].left_tenths_deg = 1800; assert(!independent_steering_config_valid(&c));
}

static void test_linkage_limits(void)
{
    independent_steering_config_t c;
    independent_steering_defaults(&c);
    assert(c.linkage.inside_tenths_deg == 900 && c.linkage.outside_tenths_deg == 680);
    assert(!steering_linkage_limits_valid(NULL));
    c.linkage.inside_tenths_deg = 901; assert(!independent_steering_config_valid(&c));
    c.linkage.inside_tenths_deg = 900;
    c.linkage.outside_tenths_deg = 901; assert(!independent_steering_config_valid(&c));
    c.linkage.outside_tenths_deg = 680;
    for (int i = 0; i < 2; i++) c.servo[i] = (steering_servo_config_t){2500,1500,500,300,300};
    uint16_t p[2]; float a[2];
    // 68 shaft degrees is 755 whole us inward; parallel targets stay equal.
    independent_steering_mix(&c, -1, 0, p, a);
    assert(p[0] == 2255 && p[1] == 2255);
    near(a[0], -22.65f, .0001f); near(a[1], -22.65f, .0001f);
    independent_steering_mix(&c, 1, 0, p, a);
    assert(p[0] == 745 && p[1] == 745);
    near(a[0], 22.65f, .0001f); near(a[1], 22.65f, .0001f);
    // At full Ackermann the outside angle fits: inside still reaches 90 shaft degrees.
    independent_steering_mix(&c, -1, 100, p, a);
    assert(p[0] == 2500 && p[1] <= 2255); near(a[0], -30, .0001f);
    independent_steering_mix(&c, 1, 100, p, a);
    assert(p[1] == 500 && p[0] >= 745); near(a[1], 30, .0001f);
    c.linkage = (steering_linkage_limits_t){450,680};
    independent_steering_mix(&c, 1, 100, p, a);
    assert(p[1] == 1000); near(a[1], 15, .0001f);
    // Tight outside limit: independent inverse-cotangent oracle gives inside target.
    c.linkage = (steering_linkage_limits_t){180,90};
    independent_steering_mix(&c, -1, 100, p, a);
    assert(p[1] == 1600); near(a[1], -3, .0001f);
    near(-a[0], atanf(1.0f / (1.0f / tanf(3 * rad) - 320.0f / 445.0f)) / rad, .0001f);

    const steering_linkage_limits_t limits[] = {{900,680},{180,90},{0,680},{900,0},{1,1},{900,900}};
    const int16_t trims[][2] = {{0,0},{125,-175},{950,-950}};
    for (size_t t = 0; t < sizeof(trims) / sizeof(trims[0]); t++) {
        for (int mirrored = 0; mirrored < 2; mirrored++) {
            c.servo[0] = (steering_servo_config_t){2500,1500,500,750,850};
            c.servo[1] = (steering_servo_config_t){mirrored ? 500 : 2500,1500,mirrored ? 2500 : 500,850,650};
            c.trim_us[0] = trims[t][0]; c.trim_us[1] = trims[t][1];
            c.geometry.max_left_tenths_deg = 750; c.geometry.max_right_tenths_deg = 650;
            for (int geometry = 0; geometry < 3; geometry++) {
                c.geometry.wheelbase_tenths_mm = geometry == 0 ? 4450 : geometry == 1 ? 500 : 20000;
                c.geometry.track_tenths_mm = geometry == 0 ? 3200 : geometry == 1 ? 20000 : 500;
                for (size_t l = 0; l < sizeof(limits) / sizeof(limits[0]); l++) {
                    c.linkage = limits[l];
                    assert(independent_steering_config_valid(&c));
                    independent_steering_config_t before = c;
                    for (int strength = 0; strength <= 100; strength += 10) {
                        for (int cmd = -120; cmd <= 120; cmd += 3) {
                            independent_steering_mix(&c, cmd / 100.0f, strength, p, a);
                            int inside = cmd < 0 ? 0 : 1, outside = 1 - inside;
                            for (int i = 0; i < 2; i++) {
                                int center = c.servo[i].center_us + c.trim_us[i];
                                float shaft_deg = abs((int)p[i] - center) * .09f;
                                float cap_deg = (i == inside ? c.linkage.inside_tenths_deg : c.linkage.outside_tenths_deg) / 10.0f;
                                assert(shaft_deg <= cap_deg + .0001f && p[i] >= 500 && p[i] <= 2500);
                                if (cmd == 0) assert(p[i] == center && a[i] == 0);
                                else {
                                    int endpoint = cmd < 0 ? c.servo[i].left_us : c.servo[i].right_us;
                                    assert((p[i] >= center && p[i] <= endpoint) || (p[i] <= center && p[i] >= endpoint));
                                    float endpoint_angle = (cmd < 0 ? c.servo[i].left_tenths_deg : c.servo[i].right_tenths_deg) / 10.0f;
                                    near(p[i], center + fabsf(a[i]) / endpoint_angle * (endpoint - center), .501f);
                                }
                            }
                            double inner = fabs(a[inside]), outer = fabs(a[outside]);
                            assert(outer <= inner + .0001 && inner <= (cmd < 0 ? 75 : 65));
                            if (inner > 0) {
                                // Solve the turn radius, independently of the production atan2 formulation.
                                double ideal = atan(1.0 / (1.0 / tan(inner * rad) +
                                    (double)c.geometry.track_tenths_mm / c.geometry.wheelbase_tenths_mm)) / rad;
                                near(outer, inner * (1 - strength / 100.0) + ideal * strength / 100.0, .0001f);
                            } else assert(outer == 0);
                        }
                    }
                    assert(memcmp(&before, &c, sizeof(c)) == 0); // Limits never rewrite calibration.
                }
            }
        }
    }
}

static void test_toe_range(void)
{
    independent_steering_config_t c;
    independent_steering_defaults(&c);
    int16_t minimum, maximum;
    assert(independent_steering_toe_range(&c, &minimum, &maximum));
    assert(minimum == -300 && maximum == 300); // Narrower calibrated endpoints bind.
    for (int i = 0; i < 2; i++) {
        c.servo[i].left_us = 2500; c.servo[i].right_us = 500;
    }
    assert(independent_steering_toe_range(&c, &minimum, &maximum));
    assert(minimum == -300 && maximum == 226); // 68 shaft degrees = 755 us, rounded inward.
    c.trim_us[0] = 200; c.trim_us[1] = -100;
    assert(independent_steering_toe_range(&c, &minimum, &maximum));
    assert(minimum == -300 && maximum == 188); // Left wheel's longer PWM span binds.
    c.servo[0].right_tenths_deg = 400;
    assert(independent_steering_toe_range(&c, &minimum, &maximum));
    assert(minimum == -300 && maximum == 205); // Now the right wheel binds.
    c.servo[1].left_us = 500; c.servo[1].right_us = 2500;
    assert(independent_steering_toe_range(&c, &minimum, &maximum));
    assert(minimum == -272 && maximum == 251); // Reversed PWM with asymmetric trims.
    c.linkage.inside_tenths_deg = 0;
    assert(independent_steering_toe_range(&c, &minimum, &maximum));
    assert(minimum == 0 && maximum == 251);
    c.linkage = (steering_linkage_limits_t){900,0};
    assert(independent_steering_toe_range(&c, &minimum, &maximum));
    assert(minimum == -272 && maximum == 0);
    c.linkage.inside_tenths_deg = 0;
    assert(independent_steering_toe_range(&c, &minimum, &maximum));
    assert(minimum == 0 && maximum == 0);
    assert(!independent_steering_toe_range(NULL, &minimum, &maximum));
    assert(!independent_steering_toe_range(&c, NULL, &maximum));
    assert(!independent_steering_toe_range(&c, &minimum, NULL));
    c.linkage.inside_tenths_deg = 901;
    assert(!independent_steering_toe_range(&c, &minimum, &maximum));
    c.linkage.inside_tenths_deg = 900; c.trim_us[0] = 1000;
    assert(!independent_steering_toe_range(&c, &minimum, &maximum));
}

static void test_toe(void)
{
    independent_steering_config_t c;
    independent_steering_defaults(&c);
    assert(c.toe_tenths_deg == 0);
    c.toe_tenths_deg = 30;
    uint16_t p[2]; float a[2];
    independent_steering_mix(&c, 0, 100, p, a);
    assert(p[0] == 1450 && p[1] == 1550 && a[0] == 3 && a[1] == -3);
    independent_steering_mix(&c, -.5f, 0, p, a);
    assert(p[0] == 1700 && p[1] == 1800 && a[0] == -12 && a[1] == -18);
    independent_steering_mix(&c, .5f, 0, p, a);
    assert(p[0] == 1200 && p[1] == 1300 && a[0] == 18 && a[1] == 12);
    independent_steering_mix(&c, 1, 0, p, a);
    assert(p[0] == 1000 && p[1] == 1100 && a[0] == 30 && a[1] == 24);
    c.toe_tenths_deg = -30;
    independent_steering_mix(&c, 0, 0, p, a);
    assert(p[0] == 1550 && p[1] == 1450 && a[0] == -3 && a[1] == 3);
    for (int toe = -300; toe <= 300; toe++) {
        c.toe_tenths_deg = toe;
        assert(independent_steering_config_valid(&c));
    }
    const int invalid_toe[] = {-301, 301, INT16_MIN, INT16_MAX};
    for (unsigned i = 0; i < sizeof(invalid_toe) / sizeof(invalid_toe[0]); i++) {
        c.toe_tenths_deg = invalid_toe[i];
        assert(!independent_steering_config_valid(&c));
    }
    c.linkage.inside_tenths_deg = 0;
    for (int toe = -1; toe <= 300; toe++) {
        c.toe_tenths_deg = toe;
        assert(independent_steering_config_valid(&c) == (toe >= 0));
    }
    c.toe_tenths_deg = 1;
    c.linkage = (steering_linkage_limits_t){900,0};
    assert(!independent_steering_config_valid(&c));
    const steering_linkage_limits_t limits[] = {{900,680},{900,900},{180,90},{1,1},{0,900},{900,0},{0,0}};
    for (int mirrored = 0; mirrored < 2; mirrored++) {
        c.servo[0] = (steering_servo_config_t){2500,1500,500,750,850};
        c.servo[1] = (steering_servo_config_t){mirrored ? 500 : 2500,1500,mirrored ? 2500 : 500,850,650};
        c.geometry.max_left_tenths_deg = 750; c.geometry.max_right_tenths_deg = 650;
        for (int trim = -250; trim <= 250; trim += 250) {
            c.trim_us[0] = trim; c.trim_us[1] = -trim;
            for (unsigned l = 0; l < sizeof(limits) / sizeof(limits[0]); l++) {
                c.linkage = limits[l];
                int16_t minimum, maximum;
                assert(independent_steering_toe_range(&c, &minimum, &maximum));
                c.toe_tenths_deg = minimum - 1;
                assert(!independent_steering_config_valid(&c));
                c.toe_tenths_deg = maximum + 1;
                assert(!independent_steering_config_valid(&c));
                for (int toe = minimum; ; toe = toe + 17 < maximum ? toe + 17 : maximum) {
                    c.toe_tenths_deg = toe;
                    assert(independent_steering_config_valid(&c));
                    independent_steering_config_t before = c;
                    for (int strength = 0; strength <= 100; strength += 25) {
                        for (int cmd = -100; cmd <= 100; cmd += 5) {
                            independent_steering_mix(&c, cmd / 100.0f, strength, p, a);
                            float component[2];
                            for (int i = 0; i < 2; i++) {
                                float offset = toe / 10.0f * (i == 0 ? 1 : -1);
                                component[i] = a[i] - offset;
                                int center = c.servo[i].center_us + c.trim_us[i];
                                bool left = a[i] < 0;
                                int endpoint = left ? c.servo[i].left_us : c.servo[i].right_us;
                                float endpoint_deg = (left ? c.servo[i].left_tenths_deg : c.servo[i].right_tenths_deg) / 10.0f;
                                // Physical role follows each total wheel angle, even through toe's sign crossing.
                                bool inside = i == (left ? 0 : 1);
                                float cap = (inside ? c.linkage.inside_tenths_deg : c.linkage.outside_tenths_deg) / 10.0f;
                                assert(abs((int)p[i] - center) * .09f <= cap + .0001f);
                                assert(p[i] >= 500 && p[i] <= 2500 && fabsf(a[i]) <= endpoint_deg + .0001f);
                                near(p[i], center + fabsf(a[i]) / endpoint_deg * (endpoint - center), .501f);
                                if (cmd == 0) near(a[i], offset, .0001f);
                            }
                            int inside = cmd < 0 ? 0 : 1, outside = 1 - inside;
                            double inner = fabs(component[inside]), outer = fabs(component[outside]);
                            assert(outer <= inner + .0001);
                            if (inner > .0001) {
                                double ideal = atan(1.0 / (1.0 / tan(inner * rad) +
                                    (double)c.geometry.track_tenths_mm / c.geometry.wheelbase_tenths_mm)) / rad;
                                near(outer, inner * (1 - strength / 100.0) + ideal * strength / 100.0, .0001f);
                            }
                        }
                    }
                    assert(memcmp(&before, &c, sizeof(c)) == 0);
                    if (toe == maximum) break;
                }
            }
        }
    }
}

static void test_measured_wheel_angles(void)
{
    independent_steering_config_t c;
    independent_steering_defaults(&c);
    c.wheel_calibration[0] = (wheel_angle_calibration_t){5, {{2000,-300},{1750,-100},{1500,0},{1200,80},{1000,300}}};
    c.servo[1] = (steering_servo_config_t){1000,1500,2000,400,300};
    c.wheel_calibration[1] = (wheel_angle_calibration_t){5, {{1000,-400},{1250,-80},{1500,0},{1800,160},{2000,300}}};
    assert(independent_steering_config_valid(&c));
    uint16_t pulses[2]; float angles[2];
    independent_steering_mix(&c, 8.0f / 30, 0, pulses, angles);
    assert(pulses[0] == 1200 && pulses[1] == 1650); // Same road angle, different nonlinear PWM maps.
    independent_steering_mix(&c, -10.0f / 30, 0, pulses, angles);
    assert(pulses[0] == 1750 && pulses[1] == 1234);
    c.linkage.outside_tenths_deg = 180; // 200 us: lies between measurement points.
    int16_t minimum, maximum;
    assert(independent_steering_toe_range(&c, &minimum, &maximum));
    assert(minimum == -300 && maximum == 53);
    independent_steering_measurement_pulses(&c, STEERING_LEFT, 1, pulses);
    assert(pulses[0] == 1300 && pulses[1] == 1500);
    independent_steering_measurement_pulses(&c, STEERING_RIGHT, -1, pulses);
    assert(pulses[0] == 1500 && pulses[1] == 1300);
    for (int toe = minimum; toe <= maximum; toe++) {
        c.toe_tenths_deg = toe;
        for (int strength = 0; strength <= 100; strength += 25) {
            for (int cmd = -100; cmd <= 100; cmd += 10) {
                independent_steering_mix(&c, cmd / 100.0f, strength, pulses, angles);
                for (int side = 0; side < 2; side++) {
                    const wheel_angle_calibration_t *cal = &c.wheel_calibration[side];
                    bool inside = side == (angles[side] < 0 ? STEERING_LEFT : STEERING_RIGHT);
                    assert(abs((int)pulses[side] - 1500) <= (inside ? 1000 : 200));
                    bool found = false;
                    for (int j = 1; j < cal->count; j++) {
                        const wheel_angle_point_t *a = &cal->points[j-1], *b = &cal->points[j];
                        if (angles[side] * 10 >= a->angle_tenths_deg - .001f && angles[side] * 10 <= b->angle_tenths_deg + .001f) {
                            float expected = a->pulse_us + (angles[side] * 10 - a->angle_tenths_deg) /
                                (b->angle_tenths_deg - a->angle_tenths_deg) * ((int)b->pulse_us - a->pulse_us);
                            near(pulses[side], expected, .501f); found = true; break;
                        }
                    }
                    assert(found);
                }
            }
        }
    }
    c.toe_tenths_deg = maximum + 1; assert(!independent_steering_config_valid(&c));
    c.toe_tenths_deg = 0;
    independent_steering_config_t valid = c;
    c.wheel_calibration[0].count = 2; assert(!independent_steering_config_valid(&c));
    c = valid; c.wheel_calibration[0].count = 10; assert(!independent_steering_config_valid(&c));
    c = valid; c.wheel_calibration[0].points[2].pulse_us = 1501; assert(!independent_steering_config_valid(&c));
    c = valid; c.trim_us[0] = 1; assert(!independent_steering_config_valid(&c));
    c = valid; c.wheel_calibration[0].points[1].pulse_us = 2000; assert(!independent_steering_config_valid(&c));
    c = valid; c.wheel_calibration[0].points[1].pulse_us = 1400; assert(!independent_steering_config_valid(&c));
    c = valid; c.wheel_calibration[0].points[1].angle_tenths_deg = 0; assert(!independent_steering_config_valid(&c));
    c = valid; c.wheel_calibration[0].points[0].angle_tenths_deg = -301; assert(!independent_steering_config_valid(&c));
    assert(!steering_wheel_calibration_valid(NULL, 0));
    assert(!steering_wheel_calibration_valid(&valid, 2));
}

int main(void)
{
    test_measured_wheel_angles();
    test_servo_trim();
    test_180_degree_servo_mounting();
    test_linkage_limits();
    test_toe_range();
    test_toe();
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
    puts("Independent steering: Ackermann identity/blend, linkage travel limits, trims, angle/PWM calibration, geometry, interpolation and bounds passed.");
    return 0;
}
