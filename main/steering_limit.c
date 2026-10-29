#include "steering_limit.h"

#include <math.h>
#include <stddef.h>

_Static_assert(sizeof(steering_limit_config_t) == 36, "Preserve steering-limit NVS format");

void steering_limit_defaults(steering_limit_config_t *config)
{
    *config = (steering_limit_config_t){
        .version = STEERING_LIMIT_VERSION,
        .point_count = 2,
        .points = {{0, 1000}, {1000, 1000}},
    };
}

bool steering_limit_config_valid(const steering_limit_config_t *config)
{
    if (config == NULL || config->version != STEERING_LIMIT_VERSION ||
        config->point_count < 2 || config->point_count > STEERING_LIMIT_MAX_POINTS ||
        config->points[0].speed_tenths_kmh != 0) return false;
    for (uint16_t i = 0; i < config->point_count; i++) {
        if (config->points[i].speed_tenths_kmh > 3000 ||
            config->points[i].limit_tenths_percent > 1000 ||
            (i > 0 && config->points[i].speed_tenths_kmh <= config->points[i - 1].speed_tenths_kmh)) return false;
    }
    return true;
}

float steering_limit_percent(const steering_limit_config_t *config,
                             float speed_kmh, bool speed_valid)
{
    if (!speed_valid || !isfinite(speed_kmh) || speed_kmh < 0.0f ||
        !steering_limit_config_valid(config)) return 100.0f;
    float speed10 = speed_kmh * 10.0f;
    for (uint16_t i = 1; i < config->point_count; i++) {
        const steering_limit_point_t *a = &config->points[i - 1], *b = &config->points[i];
        if (speed10 <= b->speed_tenths_kmh) {
            float fraction = (speed10 - a->speed_tenths_kmh) /
                             (b->speed_tenths_kmh - a->speed_tenths_kmh);
            return (a->limit_tenths_percent + fraction *
                    ((float)b->limit_tenths_percent - a->limit_tenths_percent)) / 10.0f;
        }
    }
    return config->points[config->point_count - 1].limit_tenths_percent / 10.0f;
}
