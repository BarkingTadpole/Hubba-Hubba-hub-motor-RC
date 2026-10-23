#include "independent_steering.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include "default_config.h"

_Static_assert(sizeof(independent_steering_config_t) == 66, "Bump NVS version when steering layout changes");

void independent_steering_defaults(independent_steering_config_t *config)
{
    *config = (independent_steering_config_t){
        .version = INDEPENDENT_STEERING_VERSION,
        .servo = {
            {2000, 1500, 1000, DEFAULT_STEERING_ANGLE_TENTHS_DEG, DEFAULT_STEERING_ANGLE_TENTHS_DEG},
            {2000, 1500, 1000, DEFAULT_STEERING_ANGLE_TENTHS_DEG, DEFAULT_STEERING_ANGLE_TENTHS_DEG},
        },
        .geometry = {
            DEFAULT_WHEELBASE_TENTHS_MM, DEFAULT_TRACK_TENTHS_MM,
            DEFAULT_WHEEL_DIAMETER_TENTHS_MM,
            DEFAULT_STEERING_ANGLE_TENTHS_DEG, DEFAULT_STEERING_ANGLE_TENTHS_DEG,
        },
        .point_count = 2,
        .points = {{0, 0}, {1000, 0}},
    };
}

static bool angle_valid(uint16_t tenths_deg)
{
    return tenths_deg >= 10 && tenths_deg <= 850;
}

bool steering_servo_config_valid(const steering_servo_config_t *servo)
{
    if (servo == NULL || servo->left_us < 800 || servo->left_us > 2200 ||
        servo->center_us < 800 || servo->center_us > 2200 ||
        servo->right_us < 800 || servo->right_us > 2200 ||
        !angle_valid(servo->left_tenths_deg) || !angle_valid(servo->right_tenths_deg)) return false;
    int left = (int)servo->left_us - servo->center_us;
    int right = (int)servo->right_us - servo->center_us;
    return abs(left) >= 50 && abs(right) >= 50 && ((left > 0) != (right > 0));
}

bool steering_geometry_valid(const steering_geometry_t *g)
{
    return g != NULL && g->wheelbase_tenths_mm >= 500 && g->wheelbase_tenths_mm <= 20000 &&
           g->track_tenths_mm >= 500 && g->track_tenths_mm <= 20000 &&
           g->wheel_diameter_tenths_mm >= 100 && g->wheel_diameter_tenths_mm <= 5000 &&
           angle_valid(g->max_left_tenths_deg) && angle_valid(g->max_right_tenths_deg);
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

bool independent_steering_config_valid(const independent_steering_config_t *config)
{
    if (config == NULL || config->version != INDEPENDENT_STEERING_VERSION ||
        !steering_geometry_valid(&config->geometry) ||
        !ackermann_curve_valid(config->points, config->point_count)) return false;
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        if (!steering_servo_config_valid(&config->servo[i]) ||
            config->geometry.max_left_tenths_deg > config->servo[i].left_tenths_deg ||
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
        const float radians_per_degree = 0.017453292519943295f;
        float inside_rad = inside_deg * radians_per_degree;
        /* cot(outside) - cot(inside) = track / wheelbase. atan2 is finite
         * at center; no small-angle division or turn-radius singularity. */
        float ideal_deg = atan2f(g->wheelbase_tenths_mm * sinf(inside_rad),
                                g->wheelbase_tenths_mm * cosf(inside_rad) +
                                g->track_tenths_mm * sinf(inside_rad)) / radians_per_degree;
        outside_deg = inside_deg + (ideal_deg - inside_deg) * strength_percent / 100.0f;
    }
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        if (angles_deg != NULL) angles_deg[i] = 0.0f;
        uint16_t center = config != NULL ? config->servo[i].center_us : 1500;
        pulses[i] = center >= 800 && center <= 2200 ? center : 1500;
        if (!valid || command == 0.0f) continue;
        const steering_servo_config_t *servo = &config->servo[i];
        bool left = command < 0.0f;
        bool inside = i == (left ? STEERING_LEFT : STEERING_RIGHT);
        float angle = inside ? inside_deg : outside_deg;
        float endpoint_deg = (left ? servo->left_tenths_deg : servo->right_tenths_deg) / 10.0f;
        int endpoint_us = left ? servo->left_us : servo->right_us;
        float travel = fminf(1.0f, angle / endpoint_deg);
        pulses[i] = (uint16_t)lroundf(center + travel * (endpoint_us - center));
        if (angles_deg != NULL) angles_deg[i] = left ? -angle : angle;
    }
}
