#pragma once

#include "powertrain_types.h"

void steering_limit_defaults(steering_limit_config_t *config);
bool steering_limit_config_valid(const steering_limit_config_t *config);
/* Invalid/missing speed bypasses the limiter, including held RPM estimates. */
float steering_limit_percent(const steering_limit_config_t *config,
                             float speed_kmh, bool speed_valid);
