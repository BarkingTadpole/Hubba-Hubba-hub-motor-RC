#include "esc_output.h"

#include <stddef.h>
#include <string.h>

#include "driver/ledc.h"
#include "pin_config.h"

#define SERVO_FRAME_HZ 50
#define SERVO_FRAME_US 20000
#define LEDC_DUTY_RES LEDC_TIMER_16_BIT
#define LEDC_MAX_DUTY ((1U << 16) - 1U)

typedef struct {
    gpio_num_t gpio;
    ledc_channel_t channel;
} pwm_pin_t;

static const pwm_pin_t throttle_pins[POWERTRAIN_WHEEL_COUNT] = {
    [WHEEL_FRONT_LEFT] = {PIN_ESC_FL_THROTTLE, LEDC_CHANNEL_0},
    [WHEEL_FRONT_RIGHT] = {PIN_ESC_FR_THROTTLE, LEDC_CHANNEL_1},
    [WHEEL_REAR_LEFT] = {PIN_ESC_RL_THROTTLE, LEDC_CHANNEL_2},
    [WHEEL_REAR_RIGHT] = {PIN_ESC_RR_THROTTLE, LEDC_CHANNEL_3},
};

static const pwm_pin_t reverse_pins[POWERTRAIN_WHEEL_COUNT] = {
    [WHEEL_FRONT_LEFT] = {PIN_ESC_FL_REVERSE, LEDC_CHANNEL_4},
    [WHEEL_FRONT_RIGHT] = {PIN_ESC_FR_REVERSE, LEDC_CHANNEL_5},
    [WHEEL_REAR_LEFT] = {PIN_ESC_RL_REVERSE, LEDC_CHANNEL_6},
    [WHEEL_REAR_RIGHT] = {PIN_ESC_RR_REVERSE, LEDC_CHANNEL_7},
};

static esc_limits_t configured_limits;
static wheel_output_command_t current_output;

static uint32_t pulse_us_to_duty(uint16_t pulse_us)
{
    return ((uint32_t)pulse_us * LEDC_MAX_DUTY) / SERVO_FRAME_US;
}

static void set_pin(const pwm_pin_t *pin, uint16_t pulse_us)
{
    uint32_t duty = pulse_us_to_duty(pulse_us);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, pin->channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, pin->channel));
}

static esp_err_t configure_pin(const pwm_pin_t *pin, uint16_t initial_pulse_us)
{
    ledc_channel_config_t channel_config = {
        .gpio_num = pin->gpio,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = pin->channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = pulse_us_to_duty(initial_pulse_us),
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags.output_invert = 0,
        .deconfigure = false,
    };
    return ledc_channel_config(&channel_config);
}

esp_err_t esc_output_init(const esc_limits_t *limits)
{
    if (limits == NULL || limits->throttle_min_us >= limits->throttle_max_us) {
        return ESP_ERR_INVALID_ARG;
    }

    configured_limits = *limits;

    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_FRAME_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        return err;
    }

    for (size_t i = 0; i < POWERTRAIN_WHEEL_COUNT; i++) {
        err = configure_pin(&throttle_pins[i], configured_limits.throttle_min_us);
        if (err != ESP_OK) {
            return err;
        }
        err = configure_pin(&reverse_pins[i], configured_limits.reverse_low_us);
        if (err != ESP_OK) {
            return err;
        }
    }

    esc_output_set_safe();
    return ESP_OK;
}

void esc_output_set_all(uint16_t throttle_us, uint16_t reverse_us)
{
    wheel_output_command_t command;
    for (size_t i = 0; i < POWERTRAIN_WHEEL_COUNT; i++) {
        command.throttle_us[i] = throttle_us;
        command.reverse_us[i] = reverse_us;
    }
    esc_output_set_wheels(&command);
}

void esc_output_set_wheels(const wheel_output_command_t *command)
{
    if (command == NULL) {
        return;
    }

    for (size_t i = 0; i < POWERTRAIN_WHEEL_COUNT; i++) {
        if (current_output.throttle_us[i] != command->throttle_us[i]) {
            current_output.throttle_us[i] = command->throttle_us[i];
            set_pin(&throttle_pins[i], command->throttle_us[i]);
        }
        if (current_output.reverse_us[i] != command->reverse_us[i]) {
            current_output.reverse_us[i] = command->reverse_us[i];
            set_pin(&reverse_pins[i], command->reverse_us[i]);
        }
    }
}

void esc_output_set_safe(void)
{
    esc_output_set_all(configured_limits.throttle_min_us, configured_limits.reverse_low_us);
}

void esc_output_get(wheel_output_command_t *command)
{
    if (command != NULL) {
        memcpy(command, &current_output, sizeof(*command));
    }
}

const esc_limits_t *esc_output_limits(void)
{
    return &configured_limits;
}
