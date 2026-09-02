#include "rc_input.h"

#include <stddef.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "pin_config.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#define RC_PULSE_MIN_US 800
#define RC_PULSE_MAX_US 2200
#define RC_SIGNAL_TIMEOUT_US 100000

typedef struct {
    gpio_num_t gpio;
    volatile int64_t rise_time_us;
    volatile int64_t updated_at_us;
    volatile uint16_t pulse_us;
    portMUX_TYPE lock;
} rc_channel_state_t;

static rc_channel_state_t channels[RC_CHANNEL_COUNT] = {
    [RC_CHANNEL_THROTTLE] = {
        .gpio = PIN_RC_THROTTLE_INPUT,
        .lock = portMUX_INITIALIZER_UNLOCKED,
    },
    [RC_CHANNEL_STEERING] = {
        .gpio = PIN_RC_STEERING_INPUT,
        .lock = portMUX_INITIALIZER_UNLOCKED,
    },
    [RC_CHANNEL_ARM] = {
        .gpio = PIN_RC_ARM_INPUT,
        .lock = portMUX_INITIALIZER_UNLOCKED,
    },
    [RC_CHANNEL_TV_MODE] = {
        .gpio = PIN_RC_TV_MODE_INPUT,
        .lock = portMUX_INITIALIZER_UNLOCKED,
    },
};

static void IRAM_ATTR rc_channel_isr(void *arg)
{
    rc_channel_state_t *channel = arg;
    int64_t now_us = esp_timer_get_time();
    uint32_t levels = channel->gpio < 32 ? REG_READ(GPIO_IN_REG)
                                         : REG_READ(GPIO_IN1_REG);
    uint32_t bit = channel->gpio < 32 ? (uint32_t)channel->gpio
                                      : (uint32_t)channel->gpio - 32U;

    if ((levels & (1U << bit)) != 0) {
        channel->rise_time_us = now_us;
        return;
    }

    int64_t pulse_us = now_us - channel->rise_time_us;
    if (pulse_us < RC_PULSE_MIN_US || pulse_us > RC_PULSE_MAX_US) {
        return;
    }

    portENTER_CRITICAL_ISR(&channel->lock);
    channel->pulse_us = (uint16_t)pulse_us;
    channel->updated_at_us = now_us;
    portEXIT_CRITICAL_ISR(&channel->lock);
}

esp_err_t rc_input_init(void)
{
    uint64_t pin_mask = 0;
    for (size_t i = 0; i < RC_CHANNEL_COUNT; i++) {
        pin_mask |= 1ULL << channels[i].gpio;
    }

    gpio_config_t input_config = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    esp_err_t err = gpio_config(&input_config);
    if (err != ESP_OK) {
        return err;
    }

    /*
     * Telemetry logging writes to internal flash. Keep receiver edge capture
     * running while the flash cache is disabled or a delayed edge can look
     * like a valid but badly distorted PWM pulse. The ISR therefore uses only
     * IRAM/DRAM-safe operations, including a direct GPIO input-register read.
     */
    err = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK) {
        return err;
    }

    for (size_t i = 0; i < RC_CHANNEL_COUNT; i++) {
        err = gpio_isr_handler_add(channels[i].gpio, rc_channel_isr, &channels[i]);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

bool rc_input_get(rc_channel_id_t channel_id, rc_channel_sample_t *sample)
{
    if (channel_id >= RC_CHANNEL_COUNT || sample == NULL) {
        return false;
    }

    rc_channel_state_t *channel = &channels[channel_id];
    portENTER_CRITICAL(&channel->lock);
    sample->pulse_us = channel->pulse_us;
    sample->updated_at_us = channel->updated_at_us;
    portEXIT_CRITICAL(&channel->lock);

    sample->valid = sample->updated_at_us > 0 &&
                    (esp_timer_get_time() - sample->updated_at_us) <= RC_SIGNAL_TIMEOUT_US;
    return sample->valid;
}

bool rc_input_is_valid(rc_channel_id_t channel)
{
    rc_channel_sample_t sample;
    return rc_input_get(channel, &sample);
}

uint16_t rc_input_pulse_us(rc_channel_id_t channel)
{
    rc_channel_sample_t sample = {0};
    rc_input_get(channel, &sample);
    return sample.pulse_us;
}
