#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    STEERING_INPUT_FILTER_WARMUP = 0,
    STEERING_INPUT_FILTER_ACTIVE,
} steering_input_filter_status_t;

typedef struct {
    uint16_t history_us[3];
    uint8_t history_count;
    int64_t last_frame_at_us;
    uint16_t output_us;
    bool output_valid;
    uint32_t rejected_spike_count;
} steering_input_filter_t;

void steering_input_filter_init(steering_input_filter_t *state);
void steering_input_filter_reset(steering_input_filter_t *state);
/* Returns a median only after three distinct receiver frames are available. */
bool steering_input_filter_update(steering_input_filter_t *state,
                                  uint16_t pulse_us,
                                  uint16_t outlier_threshold_us,
                                  int64_t frame_at_us,
                                  uint16_t *filtered_us);
steering_input_filter_status_t steering_input_filter_status(
    const steering_input_filter_t *state);
