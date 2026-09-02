#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define DRAGY_I2C_MAX_DEVICES 16
#define DRAGY_RAW_CAPTURE_BYTES 512

typedef struct {
    double latitude_deg;
    double longitude_deg;
    float speed_mps;
    float course_deg;
    float altitude_m;
    float hdop;
    int64_t updated_at_us;
    uint32_t utc_time_ms;
    uint32_t date_ddmmyy;
    uint32_t baud_rate;
    uint32_t uart_bytes;
    uint32_t valid_sentence_count;
    uint32_t checksum_error_count;
    uint32_t parse_error_count;
    uint8_t satellites;
    uint8_t i2c_device_count;
    uint8_t i2c_addresses[DRAGY_I2C_MAX_DEVICES];
    bool initialized;
    bool nmea_recent;
    bool fix_valid;
    bool position_valid;
    bool speed_valid;
    bool course_valid;
    bool altitude_valid;
    bool i2c_scan_complete;
    bool compass_candidate_present;
} dragy_sensor_snapshot_t;

typedef struct {
    uint8_t bytes[DRAGY_RAW_CAPTURE_BYTES];
    size_t length;
    uint32_t total_received;
    uint32_t overwritten;
} dragy_raw_capture_t;

esp_err_t dragy_sensor_start(void);
void dragy_sensor_get_snapshot(dragy_sensor_snapshot_t *snapshot);
void dragy_sensor_reset_raw_capture(void);
void dragy_sensor_get_raw_capture(dragy_raw_capture_t *capture);
