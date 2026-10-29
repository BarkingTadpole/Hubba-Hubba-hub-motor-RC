#include "steering_limit.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

int main(void)
{
    steering_limit_config_t config;
    steering_limit_defaults(&config);
    assert(steering_limit_config_valid(&config));
    assert(steering_limit_percent(&config, 0, true) == 100);
    assert(steering_limit_percent(&config, 300, true) == 100);
    config.point_count = 3;
    config.points[0] = (steering_limit_point_t){0, 1000};
    config.points[1] = (steering_limit_point_t){400, 600};
    config.points[2] = (steering_limit_point_t){800, 200};
    assert(steering_limit_config_valid(&config));
    assert(steering_limit_percent(&config, 20, true) == 80);
    assert(steering_limit_percent(&config, 40, true) == 60);
    assert(steering_limit_percent(&config, 60, true) == 40);
    assert(steering_limit_percent(&config, 80, true) == 20);
    assert(steering_limit_percent(&config, 300, true) == 20);
    assert(steering_limit_percent(&config, 60, false) == 100);
    assert(steering_limit_percent(&config, NAN, true) == 100);
    assert(steering_limit_percent(&config, INFINITY, true) == 100);
    assert(steering_limit_percent(&config, -1, true) == 100);
    config.points[2].limit_tenths_percent = 0;
    assert(steering_limit_percent(&config, 100, true) == 0);
    config.points[1].limit_tenths_percent = 0;
    config.points[2].limit_tenths_percent = 1000;
    assert(steering_limit_config_valid(&config)); // User curves need not decrease.
    assert(steering_limit_percent(&config, 60, true) == 50);
    steering_limit_config_t good = config;
    config.version++; assert(!steering_limit_config_valid(&config)); config = good;
    config.point_count = 1; assert(!steering_limit_config_valid(&config)); config = good;
    config.point_count = 9; assert(!steering_limit_config_valid(&config)); config = good;
    config.points[0].speed_tenths_kmh = 1; assert(!steering_limit_config_valid(&config)); config = good;
    config.points[1].speed_tenths_kmh = 800; assert(!steering_limit_config_valid(&config)); config = good;
    config.points[2].speed_tenths_kmh = 3001; assert(!steering_limit_config_valid(&config)); config = good;
    config.points[2].limit_tenths_percent = 1001; assert(!steering_limit_config_valid(&config));
    assert(!steering_limit_config_valid(NULL));
    puts("Steering limit: interpolation, endpoints, bounds and unavailable-speed bypass passed.");
}
