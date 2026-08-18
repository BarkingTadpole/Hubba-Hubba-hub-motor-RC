#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    STEERING_CENTER_GUARD_LOCKED = 0,
    STEERING_CENTER_GUARD_PENDING_LOW,
    STEERING_CENTER_GUARD_PENDING_HIGH,
    STEERING_CENTER_GUARD_TRACKING,
} steering_center_guard_status_t;

typedef struct {
    bool locked;
    int8_t pending_side;
    uint8_t pending_count;
    uint8_t centered_count;
    uint32_t rejected_spike_count;
    int64_t last_frame_at_us;
    bool last_frame_accepted;
} steering_center_guard_t;

void steering_center_guard_init(steering_center_guard_t *state);
void steering_center_guard_recenter(steering_center_guard_t *state);
/* Returns true when this distinct receiver frame may command away from center. */
bool steering_center_guard_accept(steering_center_guard_t *state,
                                  uint16_t pulse_us,
                                  uint16_t center_us,
                                  uint16_t deadband_us,
                                  int64_t frame_at_us);
steering_center_guard_status_t steering_center_guard_status(
    const steering_center_guard_t *state);
