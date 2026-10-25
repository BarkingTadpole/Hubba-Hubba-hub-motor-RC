#include "servo_output.h"

#include <stddef.h>

#include "pwm_output.h"
#include "freertos/FreeRTOS.h"
#include "pin_config.h"

_Static_assert(PWM_FRAME_HZ >= SERVO_FREQUENCY_MIN_HZ && PWM_FRAME_HZ <= SERVO_FREQUENCY_MAX_HZ,
               "Steering PWM frequency must fit the servo specification");

static uint16_t current_pulse_us[STEERING_SERVO_COUNT];
static const gpio_num_t servo_pins[STEERING_SERVO_COUNT] = {
    PIN_STEERING_LEFT_OUTPUT, PIN_STEERING_RIGHT_OUTPUT,
};
static portMUX_TYPE output_lock = portMUX_INITIALIZER_UNLOCKED;

esp_err_t servo_output_init(const uint16_t centers[STEERING_SERVO_COUNT])
{
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        if (centers[i] < SERVO_PULSE_MIN_US || centers[i] > SERVO_PULSE_MAX_US) {
            return ESP_ERR_INVALID_ARG;
        }
        current_pulse_us[i] = centers[i];
    }
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_DUTY_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FRAME_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) return err;
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        ledc_channel_config_t channel_config = {
            .gpio_num = servo_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = pwm_pulse_to_duty(centers[i]),
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        };
        err = ledc_channel_config(&channel_config);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

void servo_output_set_pulses(const uint16_t pulses[STEERING_SERVO_COUNT])
{
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) {
        uint16_t pulse = pwm_clamp_pulse(pulses[i], SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
        if (pulse == current_pulse_us[i]) continue;
        /* ESC initialization installs the LEDC fade service before this API. */
        ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE,
                                                (ledc_channel_t)i,
                                                pwm_pulse_to_duty(pulse), 0));
        portENTER_CRITICAL(&output_lock);
        current_pulse_us[i] = pulse;
        portEXIT_CRITICAL(&output_lock);
    }
}

void servo_output_get_pulses(uint16_t pulses[STEERING_SERVO_COUNT])
{
    portENTER_CRITICAL(&output_lock);
    for (size_t i = 0; i < STEERING_SERVO_COUNT; i++) pulses[i] = current_pulse_us[i];
    portEXIT_CRITICAL(&output_lock);
}
