#include "independent_steering.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include "default_config.h"

_Static_assert(sizeof(independent_steering_config_t) == 152, "Bump NVS version when steering layout changes");
_Static_assert(offsetof(independent_steering_config_t, trim_us) == 66, "Preserve v2 prefix for NVS migration");
_Static_assert(offsetof(independent_steering_config_t, linkage) == 70, "Preserve v3 prefix for NVS migration");
_Static_assert(offsetof(independent_steering_config_t, toe_tenths_deg) == 74, "Preserve v4 prefix for NVS migration");
_Static_assert(offsetof(independent_steering_config_t, wheel_calibration) == 76, "Preserve v5 prefix for NVS migration");

static float linkage_wheel_angle_limit(const independent_steering_config_t *config,
                                       size_t side, bool left);
static uint16_t linkage_pulse_span(const independent_steering_config_t *config,
                                   size_t side, bool left);
static void wheel_angle_fraction(const independent_steering_config_t *config, size_t side,
                                 uint16_t pulse, int32_t *numerator, int32_t *denominator);

static float toe_angle_deg(const independent_steering_config_t *config, size_t side)
{
    return config->toe_tenths_deg / 10.0f * (side == STEERING_LEFT ? 1.0f : -1.0f);
}

void independent_steering_defaults(independent_steering_config_t *config)
{
    *config = (independent_steering_config_t){
        .version = INDEPENDENT_STEERING_VERSION,
        .servo = {
            {DEFAULT_SERVO_WHEEL_LEFT_US, SERVO_PULSE_NEUTRAL_US, DEFAULT_SERVO_WHEEL_RIGHT_US,
             DEFAULT_STEERING_ANGLE_TENTHS_DEG, DEFAULT_STEERING_ANGLE_TENTHS_DEG},
            {DEFAULT_SERVO_WHEEL_LEFT_US, SERVO_PULSE_NEUTRAL_US, DEFAULT_SERVO_WHEEL_RIGHT_US,
             DEFAULT_STEERING_ANGLE_TENTHS_DEG, DEFAULT_STEERING_ANGLE_TENTHS_DEG},
        },
        .geometry = {
            DEFAULT_WHEELBASE_TENTHS_MM, DEFAULT_TRACK_TENTHS_MM,
            DEFAULT_WHEEL_DIAMETER_TENTHS_MM,
            DEFAULT_STEERING_ANGLE_TENTHS_DEG, DEFAULT_STEERING_ANGLE_TENTHS_DEG,
        },
        .point_count = 2,
        .points = {{0, 0}, {1000, 0}},
        .linkage = {DEFAULT_LINKAGE_INSIDE_TENTHS_DEG, DEFAULT_LINKAGE_OUTSIDE_TENTHS_DEG},
    };
}

static bool angle_valid(uint16_t tenths_deg)
{
    return tenths_deg >= 10 && tenths_deg <= 850;
}

static bool center_bracketed(const steering_servo_config_t *servo, int center)
{
    int left = (int)servo->left_us - center;
    int right = (int)servo->right_us - center;
    return abs(left) >= SERVO_CENTER_CLEARANCE_US && abs(right) >= SERVO_CENTER_CLEARANCE_US &&
           ((left > 0) != (right > 0));
}

bool steering_servo_config_valid(const steering_servo_config_t *servo)
{
    if (servo == NULL || servo->left_us < SERVO_PULSE_MIN_US || servo->left_us > SERVO_PULSE_MAX_US ||
        servo->center_us < SERVO_PULSE_MIN_US || servo->center_us > SERVO_PULSE_MAX_US ||
        servo->right_us < SERVO_PULSE_MIN_US || servo->right_us > SERVO_PULSE_MAX_US ||
        !angle_valid(servo->left_tenths_deg) || !angle_valid(servo->right_tenths_deg)) return false;
    return center_bracketed(servo, servo->center_us);
}

bool steering_geometry_valid(const steering_geometry_t *g)
{
    return g != NULL && g->wheelbase_tenths_mm >= 500 && g->wheelbase_tenths_mm <= 20000 &&
           g->track_tenths_mm >= 500 && g->track_tenths_mm <= 20000 &&
           g->wheel_diameter_tenths_mm >= 100 && g->wheel_diameter_tenths_mm <= 5000 &&
           angle_valid(g->max_left_tenths_deg) && angle_valid(g->max_right_tenths_deg);
}

bool steering_linkage_limits_valid(const steering_linkage_limits_t *limits)
{
    return limits != NULL && limits->inside_tenths_deg <= SERVO_LINKAGE_MAX_TENTHS_DEG &&
           limits->outside_tenths_deg <= SERVO_LINKAGE_MAX_TENTHS_DEG;
}

bool steering_wheel_calibration_valid(const independent_steering_config_t *config, uint8_t side)
{
    if (config == NULL || side >= STEERING_SERVO_COUNT) return false;
    const wheel_angle_calibration_t *cal = &config->wheel_calibration[side];
    if (cal->count == 0) return true;
    if (cal->count < 3 || cal->count > WHEEL_CALIBRATION_MAX_POINTS) return false;
    const steering_servo_config_t *servo = &config->servo[side];
    int center = servo->center_us + config->trim_us[side];
    if (!steering_servo_config_valid(servo) || !center_bracketed(servo, center)) return false;
    const wheel_angle_point_t *first = &cal->points[0], *last = &cal->points[cal->count - 1];
    if (first->pulse_us != servo->left_us || last->pulse_us != servo->right_us ||
        first->angle_tenths_deg != -(int)servo->left_tenths_deg ||
        last->angle_tenths_deg != servo->right_tenths_deg) return false;
    bool has_neutral = false;
    for (uint16_t i = 0; i < cal->count; i++) {
        const wheel_angle_point_t *p = &cal->points[i];
        if (p->angle_tenths_deg == 0 && p->pulse_us == center) has_neutral = true;
        if (i > 0) {
            const wheel_angle_point_t *previous = &cal->points[i - 1];
            int delta = (int)p->pulse_us - previous->pulse_us;
            if (p->angle_tenths_deg <= previous->angle_tenths_deg || delta == 0 ||
                ((delta > 0) != (servo->right_us > servo->left_us))) return false;
        }
    }
    return has_neutral;
}

bool ackermann_curve_valid(const ackermann_point_t *points, uint16_t count)
{
    if (points == NULL || count < 2 || count > ACKERMANN_MAX_POINTS ||
        points[0].speed_tenths_kmh != 0) return false;
    for (uint16_t i = 0; i < count; i++) {
        if (points[i].speed_tenths_kmh > 3000 || points[i].strength_tenths_percent > 1000 ||
            (i > 0 && points[i].speed_tenths_kmh <= points[i - 1].speed_tenths_kmh)) return false;
    }
    return true;
}

bool independent_steering_toe_range(const independent_steering_config_t *config,
                                    int16_t *minimum_tenths_deg, int16_t *maximum_tenths_deg)
{
    if (config == NULL || minimum_tenths_deg == NULL || maximum_tenths_deg == NULL ||
        !steering_linkage_limits_valid(&config->linkage)) return false;
    uint16_t inward = UINT16_MAX, outward = UINT16_MAX;
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        const steering_servo_config_t *servo = &config->servo[i];
        int center = (int)servo->center_us + config->trim_us[i];
        if (!steering_servo_config_valid(servo) || !center_bracketed(servo, center) ||
            !steering_wheel_calibration_valid(config, i)) return false;
        for (unsigned direction = 0; direction < 2; direction++) {
            bool left = direction == 0;
            int endpoint = left ? servo->left_us : servo->right_us;
            uint16_t pulse = center + (endpoint > center ? 1 : -1) * linkage_pulse_span(config, i, left);
            int32_t numerator, denominator;
            wheel_angle_fraction(config, i, pulse, &numerator, &denominator);
            uint16_t limit = abs(numerator) / denominator;
            /* Toe-in uses left wheel-right / right wheel-left (outside travel).
             * Toe-out uses the opposite endpoints (inside travel). */
            uint16_t *bound = left == (i == STEERING_RIGHT) ? &inward : &outward;
            if (limit < *bound) *bound = limit;
        }
    }
    *minimum_tenths_deg = -(int16_t)outward;
    *maximum_tenths_deg = (int16_t)inward;
    return true;
}

bool independent_steering_config_valid(const independent_steering_config_t *config)
{
    int16_t toe_min, toe_max;
    if (config == NULL || config->version != INDEPENDENT_STEERING_VERSION ||
        !steering_geometry_valid(&config->geometry) ||
        !ackermann_curve_valid(config->points, config->point_count) ||
        !independent_steering_toe_range(config, &toe_min, &toe_max) ||
        config->toe_tenths_deg < toe_min || config->toe_tenths_deg > toe_max) return false;
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        if (config->geometry.max_left_tenths_deg > config->servo[i].left_tenths_deg ||
            config->geometry.max_right_tenths_deg > config->servo[i].right_tenths_deg) return false;
    }
    return true;
}

float ackermann_strength_percent(const independent_steering_config_t *config, float speed_kmh)
{
    if (config == NULL || !ackermann_curve_valid(config->points, config->point_count) ||
        !isfinite(speed_kmh) || speed_kmh < 0.0f) return 0.0f;
    float speed10 = speed_kmh * 10.0f;
    for (uint16_t i = 1; i < config->point_count; i++) {
        const ackermann_point_t *a = &config->points[i - 1], *b = &config->points[i];
        if (speed10 <= b->speed_tenths_kmh) {
            float t = (speed10 - a->speed_tenths_kmh) /
                      (b->speed_tenths_kmh - a->speed_tenths_kmh);
            return (a->strength_tenths_percent +
                    t * ((float)b->strength_tenths_percent - a->strength_tenths_percent)) / 10.0f;
        }
    }
    return config->points[config->point_count - 1].strength_tenths_percent / 10.0f;
}

static float outside_angle_deg(const steering_geometry_t *g, float inside_deg, float strength_percent)
{
    if (strength_percent == 0.0f) return inside_deg;
    const float radians_per_degree = 0.017453292519943295f;
    float inside_rad = inside_deg * radians_per_degree;
    /* cot(outside) - cot(inside) = track / wheelbase; finite at center. */
    float ideal_deg = atan2f(g->wheelbase_tenths_mm * sinf(inside_rad),
                            g->wheelbase_tenths_mm * cosf(inside_rad) +
                            g->track_tenths_mm * sinf(inside_rad)) / radians_per_degree;
    return inside_deg + (ideal_deg - inside_deg) * strength_percent / 100.0f;
}

static uint16_t linkage_pulse_span(const independent_steering_config_t *config,
                                   size_t side, bool left)
{
    bool inside = side == (left ? STEERING_LEFT : STEERING_RIGHT);
    uint16_t shaft_tenths_deg = inside ? config->linkage.inside_tenths_deg
                                      : config->linkage.outside_tenths_deg;
    const steering_servo_config_t *servo = &config->servo[side];
    int center = (int)servo->center_us + config->trim_us[side];
    int span = abs((left ? servo->left_us : servo->right_us) - center);
    /* 180 shaft degrees / 2000 us. Round inward to whole PWM microseconds. */
    int allowed = (uint32_t)shaft_tenths_deg * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) /
                  SERVO_SHAFT_TRAVEL_TENTHS_DEG;
    if (allowed > span) allowed = span;
    return (uint16_t)allowed;
}

static float linkage_wheel_angle_limit(const independent_steering_config_t *config,
                                       size_t side, bool left)
{
    const steering_servo_config_t *servo = &config->servo[side];
    int center = (int)servo->center_us + config->trim_us[side];
    int endpoint = left ? servo->left_us : servo->right_us;
    uint16_t pulse = center + (endpoint > center ? 1 : -1) * linkage_pulse_span(config, side, left);
    int32_t numerator, denominator;
    wheel_angle_fraction(config, side, pulse, &numerator, &denominator);
    return abs(numerator) / (10.0f * denominator);
}

/* Rational tenths of a degree keep toe bounds rounded inward without float error. */
static void wheel_angle_fraction(const independent_steering_config_t *config, size_t side,
                                 uint16_t pulse, int32_t *numerator, int32_t *denominator)
{
    const wheel_angle_calibration_t *cal = &config->wheel_calibration[side];
    for (uint16_t i = 1; i < cal->count; i++) {
        const wheel_angle_point_t *a = &cal->points[i - 1], *b = &cal->points[i];
        if (pulse >= (a->pulse_us < b->pulse_us ? a->pulse_us : b->pulse_us) &&
            pulse <= (a->pulse_us > b->pulse_us ? a->pulse_us : b->pulse_us)) {
            *denominator = abs((int)b->pulse_us - a->pulse_us);
            *numerator = a->angle_tenths_deg * *denominator +
                         abs((int)pulse - a->pulse_us) * (b->angle_tenths_deg - a->angle_tenths_deg);
            return;
        }
    }
    const steering_servo_config_t *servo = &config->servo[side];
    int center = servo->center_us + config->trim_us[side];
    bool left = ((int)pulse - center) * ((int)servo->left_us - center) > 0;
    *denominator = abs((left ? servo->left_us : servo->right_us) - center);
    *numerator = abs((int)pulse - center) * (left ? -(int)servo->left_tenths_deg : servo->right_tenths_deg);
}

static uint16_t wheel_angle_to_pulse(const independent_steering_config_t *config, size_t side, float angle)
{
    const wheel_angle_calibration_t *cal = &config->wheel_calibration[side];
    if (cal->count > 0) {
        float tenths = angle * 10.0f;
        for (uint16_t i = 1; i < cal->count; i++) {
            const wheel_angle_point_t *a = &cal->points[i - 1], *b = &cal->points[i];
            if (tenths <= b->angle_tenths_deg) {
                float fraction = (tenths - a->angle_tenths_deg) / (b->angle_tenths_deg - a->angle_tenths_deg);
                fraction = fmaxf(0.0f, fminf(1.0f, fraction));
                return (uint16_t)lroundf(a->pulse_us + fraction * ((int)b->pulse_us - a->pulse_us));
            }
        }
        return cal->points[cal->count - 1].pulse_us;
    }
    const steering_servo_config_t *servo = &config->servo[side];
    int center = servo->center_us + config->trim_us[side];
    float endpoint_deg = (angle < 0 ? servo->left_tenths_deg : servo->right_tenths_deg) / 10.0f;
    int endpoint_us = angle < 0 ? servo->left_us : servo->right_us;
    return (uint16_t)lroundf(center + fminf(1.0f, fabsf(angle) / endpoint_deg) * (endpoint_us - center));
}

void independent_steering_measurement_pulses(const independent_steering_config_t *config,
                                            uint8_t side, float command, uint16_t pulses[STEERING_SERVO_COUNT])
{
    bool valid = independent_steering_config_valid(config) && side < STEERING_SERVO_COUNT && isfinite(command);
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        pulses[i] = valid ? config->servo[i].center_us + config->trim_us[i] : SERVO_PULSE_NEUTRAL_US;
    }
    if (!valid) return;
    bool left = command < 0;
    int endpoint = left ? config->servo[side].left_us : config->servo[side].right_us;
    int span = linkage_pulse_span(config, side, left);
    pulses[side] += (int)lroundf(fminf(1.0f, fabsf(command)) * span) * (endpoint > pulses[side] ? 1 : -1);
}

void independent_steering_mix(const independent_steering_config_t *config,
                              float command, float strength_percent,
                              uint16_t pulses[STEERING_SERVO_COUNT],
                              float angles_deg[STEERING_SERVO_COUNT])
{
    if (pulses == NULL) return;
    bool valid = independent_steering_config_valid(config) && isfinite(command) &&
                 isfinite(strength_percent) && strength_percent >= 0.0f && strength_percent <= 100.0f;
    float inside_deg = 0.0f, outside_deg = 0.0f;
    if (valid) {
        command = fmaxf(-1.0f, fminf(1.0f, command));
        const steering_geometry_t *g = &config->geometry;
        inside_deg = fabsf(command) * (command < 0.0f ? g->max_left_tenths_deg : g->max_right_tenths_deg) / 10.0f;
        bool left = command < 0.0f;
        size_t inside_side = left ? STEERING_LEFT : STEERING_RIGHT;
        size_t outside_side = left ? STEERING_RIGHT : STEERING_LEFT;
        float sign = left ? -1.0f : 1.0f;
        /* Toe is measured from trimmed neutral and consumes the same travel.
         * Preserve Ackermann for the steering component before adding toe. */
        float inside_limit = fmaxf(0.0f, linkage_wheel_angle_limit(config, inside_side, left) -
                                  sign * toe_angle_deg(config, inside_side));
        float outside_limit = fmaxf(0.0f, linkage_wheel_angle_limit(config, outside_side, left) -
                                   sign * toe_angle_deg(config, outside_side));
        inside_deg = fminf(inside_deg, inside_limit);
        outside_deg = outside_angle_deg(g, inside_deg, strength_percent);
        if (outside_deg > outside_limit) {
            if (strength_percent == 0.0f || outside_limit == 0.0f) {
                inside_deg = outside_limit;
            } else {
                /* The blended outside angle is monotonic. Bound the search to
                 * keep both wheels within travel without distorting Ackermann. */
                float low = 0.0f, high = inside_deg;
                for (unsigned i = 0; i < 20; i++) {
                    float mid = (low + high) * 0.5f;
                    if (outside_angle_deg(g, mid, strength_percent) <= outside_limit) low = mid;
                    else high = mid;
                }
                inside_deg = low;
            }
            outside_deg = outside_angle_deg(g, inside_deg, strength_percent);
        }
    }
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        if (angles_deg != NULL) angles_deg[i] = 0.0f;
        uint16_t center = SERVO_PULSE_NEUTRAL_US;
        if (config != NULL) {
            const steering_servo_config_t *servo = &config->servo[i];
            int trimmed = (int)servo->center_us + config->trim_us[i];
            if (steering_servo_config_valid(servo) && center_bracketed(servo, trimmed)) {
                center = (uint16_t)trimmed;
            } else if (servo->center_us >= SERVO_PULSE_MIN_US && servo->center_us <= SERVO_PULSE_MAX_US) {
                center = servo->center_us;
            }
        }
        pulses[i] = center;
        if (!valid) continue;
        bool turn_left = command < 0.0f;
        bool inside = i == (turn_left ? STEERING_LEFT : STEERING_RIGHT);
        float angle = (turn_left ? -1.0f : 1.0f) * (inside ? inside_deg : outside_deg) +
                      toe_angle_deg(config, i);
        pulses[i] = wheel_angle_to_pulse(config, i, angle);
        if (angles_deg != NULL) angles_deg[i] = angle;
    }
}
