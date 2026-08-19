#pragma once

#include <stdbool.h>

#include "powertrain_types.h"

bool cornering_vehicle_speed_from_rear_rpm(const rpm_snapshot_t *rpm,
                                            float wheel_diameter_m,
                                            float *speed_mps);

float cornering_max_average_wheel_angle_deg(float speed_mps,
                                             float wheelbase_m,
                                             float lateral_accel_limit_g,
                                             float physical_max_angle_deg);

float cornering_lateral_demand(float speed_mps,
                               float average_wheel_angle_deg,
                               float wheelbase_m,
                               float lateral_accel_limit_g,
                               float *predicted_lateral_accel_mps2);
