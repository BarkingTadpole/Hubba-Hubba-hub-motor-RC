#pragma once

#include <stdbool.h>

#include "powertrain_types.h"

bool cornering_vehicle_speed_from_rear_rpm(const rpm_snapshot_t *rpm,
                                            float wheel_diameter_m,
                                            float *speed_mps);
