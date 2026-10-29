#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "powertrain_types.h"

typedef enum {
    REMOTE_DISARM_FOR_CONFIG = 0,
    REMOTE_CONFIG_DRIVETRAIN,
    REMOTE_CONFIG_REVERSE_LIMIT,
    REMOTE_CONFIG_TANK_LIMIT,
    REMOTE_CONFIG_DRIVE_SMOOTHING,
    REMOTE_CONFIG_FAILSAFE,
    REMOTE_CONFIG_FAILSAFE_ENABLED,
    REMOTE_CONFIG_RPM_POLES,
    REMOTE_CONFIG_RPM_PPR,
    REMOTE_CONFIG_STEERING_SMOOTHING,
    REMOTE_CONFIG_STEERING_SERVO,
    REMOTE_CONFIG_WHEEL_CALIBRATION,
    REMOTE_CONFIG_STEERING_TRIM,
    REMOTE_CONFIG_STEERING_TOE,
    REMOTE_CONFIG_STEERING_LINKAGE,
    REMOTE_CONFIG_STEERING_GEOMETRY,
    REMOTE_CONFIG_STEERING_INPUT,
    REMOTE_CONFIG_ACKERMANN,
    REMOTE_CONFIG_STEERING_LIMIT,
    REMOTE_CONFIG_ARM_LATCH,
    REMOTE_CALIBRATE_THROTTLE,
    REMOTE_CALIBRATE_STEERING,
    REMOTE_CALIBRATE_ARM,
    REMOTE_CALIBRATE_WHEEL_LEFT,
    REMOTE_CALIBRATE_WHEEL_RIGHT,
    REMOTE_CALIBRATE_CAPTURE,
    REMOTE_CALIBRATE_ESC_ARM,
    REMOTE_CALIBRATE_ESC_MAX,
    REMOTE_CALIBRATE_ESC_MIN,
    REMOTE_CALIBRATE_ESC_MANUAL,
    REMOTE_CALIBRATE_CANCEL,
} remote_config_operation_t;

typedef struct {
    remote_config_operation_t operation;
    drivetrain_mode_t drivetrain_mode;
    bool enabled;
    uint32_t unsigned_values[2];
    steering_servo_config_t servo;
    wheel_angle_calibration_t wheel_calibration;
    int16_t steering_trim_us;
    int16_t toe_tenths_deg;
    steering_linkage_limits_t linkage;
    steering_geometry_t geometry;
    steering_calibration_t steering_input;
    uint16_t point_count;
    ackermann_point_t points[ACKERMANN_MAX_POINTS];
    steering_limit_config_t steering_limit;
} remote_command_t;

bool remote_command_parse(const char *line,
                          remote_command_t *command,
                          char *error,
                          size_t error_size);
