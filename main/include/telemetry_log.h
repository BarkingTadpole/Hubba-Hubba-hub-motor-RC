#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool initialized;
    bool mounted;
    bool recording;
    bool full;
    bool faulted;
    int32_t last_errno;
    uint32_t rate_hz;
    uint32_t interval_ms;
    uint32_t boot_id;
    uint32_t sample_count;
    uint32_t write_error_count;
    uint64_t file_bytes;
    uint64_t capacity_bytes;
    uint64_t free_bytes;
} telemetry_log_status_t;

/* Mount storage and start the idle worker. Recording is always stopped at boot. */
esp_err_t telemetry_log_init(void);
/* Append to the existing CSV; stop flushes/fsyncs before returning success.
 * These operations do not change powertrain state and are allowed while armed. */
esp_err_t telemetry_log_set_recording(bool recording);
void telemetry_log_get_status(telemetry_log_status_t *status);

/* Read a stable byte range from the CSV. The current size is returned so the
 * HTTP viewer can take a download snapshot even while new rows are appended. */
esp_err_t telemetry_log_read(uint32_t offset,
                             uint8_t *buffer,
                             size_t buffer_size,
                             size_t *bytes_read,
                             uint32_t *file_size);

/* The caller owns the safety policy. Wi-Fi only calls this while DISARMED. */
esp_err_t telemetry_log_clear(void);
