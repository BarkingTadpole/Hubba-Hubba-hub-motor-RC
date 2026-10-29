#pragma once

#include <stddef.h>
#include <stdint.h>
#include "powertrain_types.h"

#define CONFIG_BACKUP_SIZE 250

typedef struct {
    throttle_calibration_t throttle;
    steering_calibration_t input;
    arm_calibration_t arm;
    drive_config_t drive;
} configuration_bundle_t;

bool configuration_bundle_valid(const configuration_bundle_t *bundle);
bool config_backup_encode(const configuration_bundle_t *bundle, uint8_t *data, size_t size);
bool config_backup_decode(const uint8_t *data, size_t size, configuration_bundle_t *bundle);
