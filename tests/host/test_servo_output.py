"""Exercise the production servo driver with simulated LEDC, not physical PWM."""
from pathlib import Path
import os
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
STUBS = {
    'esp_err.h': '''#pragma once
#include <assert.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL 1
#define ESP_ERR_INVALID_ARG 2
#define ESP_ERROR_CHECK(x) assert((x) == ESP_OK)
''',
    'freertos/FreeRTOS.h': '''#pragma once
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
''',
    'driver/gpio.h': '''#pragma once
typedef int gpio_num_t;
#define GPIO_NUM_23 23
#define GPIO_NUM_32 32
#define GPIO_NUM_26 26
''',
    'driver/ledc.h': '''#pragma once
#include <stdint.h>
#include "esp_err.h"
typedef int ledc_channel_t;
#define LEDC_LOW_SPEED_MODE 1
#define LEDC_TIMER_16_BIT 16
#define LEDC_TIMER_0 0
#define LEDC_AUTO_CLK 0
#define LEDC_INTR_DISABLE 0
#define LEDC_SLEEP_MODE_NO_ALIVE_NO_PD 0
typedef struct { int speed_mode, duty_resolution, timer_num, freq_hz, clk_cfg; } ledc_timer_config_t;
typedef struct { int gpio_num, speed_mode, channel, intr_type, timer_sel;
    uint32_t duty; int hpoint, sleep_mode; } ledc_channel_config_t;
esp_err_t ledc_timer_config(const ledc_timer_config_t *);
esp_err_t ledc_channel_config(const ledc_channel_config_t *);
esp_err_t ledc_set_duty_and_update(int, ledc_channel_t, uint32_t, uint32_t);
''',
}
SOURCE = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "servo_output.h"
#include "pwm_output.h"
#include "pin_config.h"
static ledc_timer_config_t timer;
static ledc_channel_config_t channels[2];
static uint32_t duty[2];
static int timer_error, channel_error = -1, updates;
int ledc_timer_config(const ledc_timer_config_t *value) { timer = *value; return timer_error; }
int ledc_channel_config(const ledc_channel_config_t *value) {
    assert(value->channel >= 0 && value->channel < 2);
    channels[value->channel] = *value;
    duty[value->channel] = value->duty;
    return value->channel == channel_error ? ESP_FAIL : ESP_OK;
}
int ledc_set_duty_and_update(int mode, int channel, uint32_t value, uint32_t hpoint) {
    assert(mode == LEDC_LOW_SPEED_MODE && channel >= 0 && channel < 2 && hpoint == 0);
    duty[channel] = value; updates++; return ESP_OK;
}
static void expect_pulses(uint16_t left, uint16_t right) {
    uint16_t actual[2]; servo_output_get_pulses(actual);
    assert(actual[0] == left && actual[1] == right);
    const uint16_t expected[] = {left, right};
    for (int i = 0; i < 2; i++) {
        // Reconstruct on-wire pulse time from 16-bit LEDC duty and 50 Hz.
        double width_us = duty[i] * (1000000.0 / timer.freq_hz) / 65536.0;
        assert(fabs(width_us - expected[i]) < 0.35); // Well below the rated 2 us deadband.
    }
}
int main(void) {
    uint16_t centers[] = {1500,1500};
    assert(servo_output_init((uint16_t[]){499,1500}) == ESP_ERR_INVALID_ARG);
    assert(servo_output_init((uint16_t[]){1500,2501}) == ESP_ERR_INVALID_ARG);
    timer_error = ESP_FAIL; assert(servo_output_init(centers) == ESP_FAIL); timer_error = ESP_OK;
    channel_error = 1; assert(servo_output_init(centers) == ESP_FAIL); channel_error = -1;
    assert(servo_output_init(centers) == ESP_OK);
    assert(timer.freq_hz == 50 && timer.duty_resolution == 16 && timer.speed_mode == LEDC_LOW_SPEED_MODE);
    assert(channels[0].gpio_num == PIN_STEERING_LEFT_OUTPUT && channels[1].gpio_num == PIN_STEERING_RIGHT_OUTPUT);
    for (int i = 0; i < 2; i++) assert(channels[i].speed_mode == LEDC_LOW_SPEED_MODE && channels[i].timer_sel == LEDC_TIMER_0);
    expect_pulses(1500,1500);
    servo_output_set_pulses(centers); assert(updates == 0);
    servo_output_set_pulses((uint16_t[]){500,2500}); expect_pulses(500,2500);
    servo_output_set_pulses((uint16_t[]){2500,500}); expect_pulses(2500,500);
    servo_output_set_pulses((uint16_t[]){0,UINT16_MAX}); expect_pulses(500,2500);
    for (uint16_t pulse = 500; pulse <= 2500; pulse++) {
        servo_output_set_pulses((uint16_t[]){pulse,3000-pulse}); expect_pulses(pulse,3000-pulse);
    }
    assert(servo_output_init((uint16_t[]){550,2450}) == ESP_OK); expect_pulses(550,2450);
    puts("Servo output passed: 500-2500 us, 50 Hz low-speed LEDC, quantization, clamps and init errors (simulated).");
}
'''
with tempfile.TemporaryDirectory() as directory:
    temp = Path(directory)
    for name, content in STUBS.items():
        target = temp / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)
    source = temp / 'test.c'
    source.write_text(SOURCE)
    executable = temp / 'servo_output'
    subprocess.run([*shlex.split(os.environ.get('CC', 'cc')), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(temp), '-I', str(ROOT/'main'), '-I', str(ROOT/'main/include'),
                    str(source), str(ROOT/'main/servo_output.c'), '-lm', '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
