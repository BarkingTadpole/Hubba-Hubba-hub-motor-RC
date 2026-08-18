#include "steering_input_filter.h"

#include <stddef.h>

#define STEERING_MEDIAN_SAMPLE_COUNT 3U

static uint16_t median_of_three(uint16_t first, uint16_t second, uint16_t third)
{
    if (first > second) {
        uint16_t temporary = first;
        first = second;
        second = temporary;
    }
    if (second > third) {
        uint16_t temporary = second;
        second = third;
        third = temporary;
    }
    return first > second ? first : second;
}

static uint16_t pulse_distance(uint16_t first, uint16_t second)
{
    return first > second ? first - second : second - first;
}

static void record_rejected_spike(steering_input_filter_t *state)
{
    if (state->rejected_spike_count != UINT32_MAX) {
        state->rejected_spike_count++;
    }
}

void steering_input_filter_reset(steering_input_filter_t *state)
{
    if (state == NULL) {
        return;
    }

    state->history_us[0] = 0;
    state->history_us[1] = 0;
    state->history_us[2] = 0;
    state->history_count = 0;
    state->last_frame_at_us = 0;
    state->output_us = 0;
    state->output_valid = false;
}

void steering_input_filter_init(steering_input_filter_t *state)
{
    if (state == NULL) {
        return;
    }

    state->rejected_spike_count = 0;
    steering_input_filter_reset(state);
}

bool steering_input_filter_update(steering_input_filter_t *state,
                                  uint16_t pulse_us,
                                  uint16_t outlier_threshold_us,
                                  int64_t frame_at_us,
                                  uint16_t *filtered_us)
{
    if (state == NULL || filtered_us == NULL) {
        return false;
    }
    if (frame_at_us > 0 && frame_at_us == state->last_frame_at_us) {
        if (state->output_valid) {
            *filtered_us = state->output_us;
        }
        return state->output_valid;
    }

    state->last_frame_at_us = frame_at_us;
    if (state->history_count < STEERING_MEDIAN_SAMPLE_COUNT) {
        state->history_us[state->history_count] = pulse_us;
        state->history_count++;
    } else {
        state->history_us[0] = state->history_us[1];
        state->history_us[1] = state->history_us[2];
        state->history_us[2] = pulse_us;
    }

    if (state->history_count < STEERING_MEDIAN_SAMPLE_COUNT) {
        return false;
    }

    uint16_t median_us = median_of_three(state->history_us[0],
                                         state->history_us[1],
                                         state->history_us[2]);
    if (pulse_distance(state->history_us[0], state->history_us[2]) <=
            outlier_threshold_us &&
        pulse_distance(state->history_us[1], median_us) >
            outlier_threshold_us) {
        record_rejected_spike(state);
    }

    state->output_us = median_us;
    state->output_valid = true;
    *filtered_us = median_us;
    return true;
}

steering_input_filter_status_t steering_input_filter_status(
    const steering_input_filter_t *state)
{
    return state != NULL && state->output_valid
               ? STEERING_INPUT_FILTER_ACTIVE
               : STEERING_INPUT_FILTER_WARMUP;
}
