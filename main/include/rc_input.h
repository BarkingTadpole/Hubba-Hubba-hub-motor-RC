#pragma once

#include "esp_err.h"
#include "powertrain_types.h"

typedef enum {
    RC_CHANNEL_THROTTLE = 0,
    RC_CHANNEL_STEERING,
    RC_CHANNEL_ARM,
    RC_CHANNEL_TV_MODE,
    RC_CHANNEL_COUNT,
} rc_channel_id_t;

esp_err_t rc_input_init(void);
bool rc_input_get(rc_channel_id_t channel, rc_channel_sample_t *sample);
bool rc_input_is_valid(rc_channel_id_t channel);
uint16_t rc_input_pulse_us(rc_channel_id_t channel);
