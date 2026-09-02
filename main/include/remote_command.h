#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "powertrain_types.h"

typedef enum {
    REMOTE_DISARM_FOR_CONFIG = 0,
    REMOTE_CONFIG_DRIVETRAIN,
    REMOTE_CONFIG_REVERSE_LIMIT,
    REMOTE_CONFIG_DRIVE_SMOOTHING,
    REMOTE_CONFIG_FAILSAFE,
    REMOTE_CONFIG_FAILSAFE_ENABLED,
    REMOTE_CONFIG_RPM_POLES,
    REMOTE_CONFIG_RPM_PPR,
    REMOTE_CONFIG_STEERING_TRIM,
    REMOTE_CONFIG_STEERING_SMOOTHING,
    REMOTE_CONFIG_STEERING_SPEED_LIMIT,
    REMOTE_CONFIG_STEERING_LATERAL_G,
    REMOTE_CONFIG_TV_ENABLED,
    REMOTE_CONFIG_TV_AUTHORITY,
    REMOTE_CONFIG_TV_FRONT_RELIEF,
    REMOTE_CONFIG_TV_GAINS,
    REMOTE_CONFIG_IMU_YAW_SIGN,
    REMOTE_CONFIG_ARM_LATCH,
    REMOTE_CONFIG_LOGGING_RATE,
    REMOTE_CALIBRATE_THROTTLE,
    REMOTE_CALIBRATE_STEERING,
    REMOTE_CALIBRATE_ARM,
    REMOTE_CALIBRATE_TV,
    REMOTE_CALIBRATE_IMU,
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
    float float_values[5];
    int32_t signed_value;
} remote_command_t;

bool remote_command_parse(const char *line,
                          remote_command_t *command,
                          char *error,
                          size_t error_size);
