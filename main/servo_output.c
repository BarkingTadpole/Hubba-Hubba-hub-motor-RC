#include "servo_output.h"

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "pin_config.h"

#define SERVO_FRAME_HZ 50
#define SERVO_FRAME_US 20000
#define SERVO_PULSE_MIN_US 800
#define SERVO_PULSE_MAX_US 2200
#define SERVO_DUTY_RES LEDC_TIMER_16_BIT
#define SERVO_MAX_DUTY ((1U << 16) - 1U)

static uint16_t neutral_pulse_us;
static uint16_t current_pulse_us;
static portMUX_TYPE output_lock = portMUX_INITIALIZER_UNLOCKED;

static uint16_t clamp_pulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_PULSE_MIN_US) {
        return SERVO_PULSE_MIN_US;
    }
    if (pulse_us > SERVO_PULSE_MAX_US) {
        return SERVO_PULSE_MAX_US;
    }
    return pulse_us;
}

static uint32_t pulse_us_to_duty(uint16_t pulse_us)
{
    return ((uint32_t)pulse_us * SERVO_MAX_DUTY) / SERVO_FRAME_US;
}

esp_err_t servo_output_init(uint16_t center_us)
{
    if (center_us < SERVO_PULSE_MIN_US || center_us > SERVO_PULSE_MAX_US) {
        return ESP_ERR_INVALID_ARG;
    }

    neutral_pulse_us = center_us;
    current_pulse_us = center_us;

    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_DUTY_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_FRAME_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t channel_config = {
        .gpio_num = PIN_STEERING_SERVO_OUTPUT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = pulse_us_to_duty(center_us),
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags.output_invert = 0,
        .deconfigure = false,
    };
    return ledc_channel_config(&channel_config);
}

void servo_output_set_pulse(uint16_t pulse_us)
{
    uint16_t clamped_pulse_us = clamp_pulse(pulse_us);
    if (clamped_pulse_us == current_pulse_us) {
        return;
    }

    ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE,
                                             LEDC_CHANNEL_0,
                                             pulse_us_to_duty(clamped_pulse_us),
                                             0));
    portENTER_CRITICAL(&output_lock);
    current_pulse_us = clamped_pulse_us;
    portEXIT_CRITICAL(&output_lock);
}

void servo_output_set_neutral(void)
{
    servo_output_set_pulse(neutral_pulse_us);
}

uint16_t servo_output_get_pulse(void)
{
    portENTER_CRITICAL(&output_lock);
    uint16_t pulse_us = current_pulse_us;
    portEXIT_CRITICAL(&output_lock);
    return pulse_us;
}
