#include "steering_center_guard.h"

#include <stddef.h>

#define STEERING_CONFIRM_FRAMES 2U

static void record_rejected_spike(steering_center_guard_t *state)
{
    if (state->rejected_spike_count != UINT32_MAX) {
        state->rejected_spike_count++;
    }
}

void steering_center_guard_recenter(steering_center_guard_t *state)
{
    if (state == NULL) {
        return;
    }

    state->locked = true;
    state->pending_side = 0;
    state->pending_count = 0;
    state->centered_count = STEERING_CONFIRM_FRAMES;
    state->last_frame_at_us = 0;
    state->last_frame_accepted = false;
}

void steering_center_guard_init(steering_center_guard_t *state)
{
    if (state == NULL) {
        return;
    }

    state->rejected_spike_count = 0;
    steering_center_guard_recenter(state);
}

bool steering_center_guard_accept(steering_center_guard_t *state,
                                  uint16_t pulse_us,
                                  uint16_t center_us,
                                  uint16_t deadband_us,
                                  int64_t frame_at_us)
{
    if (state == NULL) {
        return false;
    }
    if (frame_at_us > 0 && frame_at_us == state->last_frame_at_us) {
        return state->last_frame_accepted;
    }

    state->last_frame_at_us = frame_at_us;

    int32_t center_delta = (int32_t)pulse_us - (int32_t)center_us;
    bool centered = center_delta >= -(int32_t)deadband_us &&
                    center_delta <= (int32_t)deadband_us;

    if (state->locked) {
        if (centered) {
            if (state->pending_count != 0) {
                record_rejected_spike(state);
            }
            state->pending_side = 0;
            state->pending_count = 0;
            state->centered_count = STEERING_CONFIRM_FRAMES;
            state->last_frame_accepted = false;
            return state->last_frame_accepted;
        }

        int8_t side = center_delta < 0 ? -1 : 1;
        if (state->pending_side != side) {
            if (state->pending_count != 0) {
                record_rejected_spike(state);
            }
            state->pending_side = side;
            state->pending_count = 1;
        } else if (state->pending_count < STEERING_CONFIRM_FRAMES) {
            state->pending_count++;
        }

        if (state->pending_count < STEERING_CONFIRM_FRAMES) {
            state->last_frame_accepted = false;
            return state->last_frame_accepted;
        }

        state->locked = false;
        state->pending_side = 0;
        state->pending_count = 0;
        state->centered_count = 0;
        state->last_frame_accepted = true;
        return state->last_frame_accepted;
    }

    if (centered) {
        if (state->centered_count < STEERING_CONFIRM_FRAMES) {
            state->centered_count++;
        }
        if (state->centered_count >= STEERING_CONFIRM_FRAMES) {
            state->locked = true;
            state->pending_side = 0;
            state->pending_count = 0;
        }
        state->last_frame_accepted = false;
        return state->last_frame_accepted;
    }

    state->centered_count = 0;
    state->last_frame_accepted = true;
    return state->last_frame_accepted;
}

steering_center_guard_status_t steering_center_guard_status(
    const steering_center_guard_t *state)
{
    if (state == NULL || !state->locked) {
        return STEERING_CENTER_GUARD_TRACKING;
    }
    if (state->pending_side < 0) {
        return STEERING_CENTER_GUARD_PENDING_LOW;
    }
    if (state->pending_side > 0) {
        return STEERING_CENTER_GUARD_PENDING_HIGH;
    }
    return STEERING_CENTER_GUARD_LOCKED;
}
