#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "pin_config.h"

#define SERVO_FRAME_HZ 50
#define SERVO_FRAME_US 20000
#define LEDC_DUTY_RES LEDC_TIMER_16_BIT
#define LEDC_MAX_DUTY ((1U << 16) - 1U)

#define DEFAULT_ESC_MIN_US 1100
#define DEFAULT_ESC_MAX_US 1940
#define DEFAULT_RECEIVER_MIN_US 1000
#define DEFAULT_RECEIVER_NEUTRAL_US 1250
#define DEFAULT_RECEIVER_MAX_US 1750
#define DEFAULT_RECEIVER_DEADBAND_US 80
#define DEFAULT_REVERSE_LIMIT_PERCENT 10
#define DEFAULT_RECEIVER_FAILSAFE_US 1565
#define DEFAULT_RECEIVER_FAILSAFE_WINDOW_US 10

#define RECEIVER_SIGNAL_TIMEOUT_US 250000
#define RECEIVER_FAILSAFE_HOLD_US 300000
#define ESC_CAL_TIMEOUT_US 30000000
#define THROTTLE_MONITOR_DURATION_US 30000000
#define THROTTLE_MONITOR_INTERVAL_MS 250
#define ARM_SWITCH_MONITOR_DURATION_US 30000000
#define ARM_SWITCH_MONITOR_INTERVAL_MS 250
#define DRIVE_UPDATE_INTERVAL_MS 20
#define DRIVE_DIRECTION_CHANGE_HOLD_US 120000
#define DRIVE_ACCEL_STEP_US 12
#define DRIVE_DECEL_STEP_US 100
#define ARM_SWITCH_DEBOUNCE_US 50000
#define CLI_LINE_MAX 96

typedef enum {
    SYSTEM_DISARMED = 0,
    SYSTEM_DRIVE_ARMED,
    SYSTEM_ESC_CAL_ARMED,
    SYSTEM_ESC_CAL_MAX,
    SYSTEM_ESC_CAL_MIN,
    SYSTEM_ESC_CAL_MANUAL,
} system_state_t;

typedef enum {
    DRIVE_DIRECTION_FORWARD = 0,
    DRIVE_DIRECTION_REVERSE,
} drive_direction_t;

typedef struct {
    uint16_t throttle_us;
    drive_direction_t direction;
    bool moving;
} drive_command_t;

typedef struct {
    uint16_t full_throttle_us;
    uint16_t neutral_us;
    uint16_t full_reverse_us;
    uint16_t deadband_us;
    bool loaded_from_nvs;
} receiver_calibration_t;

typedef struct {
    uint16_t throttle_min_us;
    uint16_t throttle_max_us;
    uint16_t reverse_low_us;
    uint16_t reverse_high_us;
} esc_output_limits_t;

typedef struct {
    uint8_t reverse_limit_percent;
    uint16_t receiver_failsafe_us;
    uint16_t receiver_failsafe_window_us;
    bool loaded_from_nvs;
} drive_config_t;

typedef struct {
    const char *name;
    gpio_num_t gpio;
    ledc_channel_t ledc_channel;
} pwm_output_t;

static const char *TAG = "rc_car";

static const pwm_output_t throttle_outputs[] = {
    {"FL throttle", PIN_ESC_FL_THROTTLE, LEDC_CHANNEL_0},
    {"FR throttle", PIN_ESC_FR_THROTTLE, LEDC_CHANNEL_1},
    {"RL throttle", PIN_ESC_RL_THROTTLE, LEDC_CHANNEL_2},
    {"RR throttle", PIN_ESC_RR_THROTTLE, LEDC_CHANNEL_3},
};

static const pwm_output_t reverse_outputs[] = {
    {"FL reverse", PIN_ESC_FL_REVERSE, LEDC_CHANNEL_4},
    {"FR reverse", PIN_ESC_FR_REVERSE, LEDC_CHANNEL_5},
    {"RL reverse", PIN_ESC_RL_REVERSE, LEDC_CHANNEL_6},
    {"RR reverse", PIN_ESC_RR_REVERSE, LEDC_CHANNEL_7},
};

static const esc_output_limits_t esc_limits = {
    .throttle_min_us = DEFAULT_ESC_MIN_US,
    .throttle_max_us = DEFAULT_ESC_MAX_US,
    .reverse_low_us = DEFAULT_ESC_MIN_US,
    .reverse_high_us = DEFAULT_ESC_MAX_US,
};

static receiver_calibration_t receiver_cal = {
    .full_throttle_us = DEFAULT_RECEIVER_MAX_US,
    .neutral_us = DEFAULT_RECEIVER_NEUTRAL_US,
    .full_reverse_us = DEFAULT_RECEIVER_MIN_US,
    .deadband_us = DEFAULT_RECEIVER_DEADBAND_US,
    .loaded_from_nvs = false,
};

static drive_config_t drive_config = {
    .reverse_limit_percent = DEFAULT_REVERSE_LIMIT_PERCENT,
    .receiver_failsafe_us = DEFAULT_RECEIVER_FAILSAFE_US,
    .receiver_failsafe_window_us = DEFAULT_RECEIVER_FAILSAFE_WINDOW_US,
    .loaded_from_nvs = false,
};

static volatile int64_t rc_rise_time_us;
static volatile int64_t rc_last_pulse_time_us;
static volatile uint32_t rc_last_pulse_us;

static system_state_t system_state = SYSTEM_DISARMED;
static int64_t esc_cal_last_command_us;
static uint16_t current_throttle_us = DEFAULT_ESC_MIN_US;
static uint16_t current_reverse_us = DEFAULT_ESC_MIN_US;
static drive_direction_t active_drive_direction = DRIVE_DIRECTION_FORWARD;
static drive_direction_t pending_drive_direction = DRIVE_DIRECTION_FORWARD;
static int64_t drive_direction_hold_until_us;
static int64_t receiver_failsafe_candidate_since_us;
static bool arm_switch_stable_on;
static bool arm_switch_raw_on;
static int64_t arm_switch_last_change_us;
static bool arm_switch_rearm_locked;
static bool arm_switch_turned_on_event;
static bool arm_switch_turned_off_event;

static bool receiver_signal_valid(void);
static uint32_t receiver_pulse_us(void);

static uint32_t pulse_us_to_duty(uint16_t pulse_us)
{
    return ((uint32_t)pulse_us * LEDC_MAX_DUTY) / SERVO_FRAME_US;
}

static void set_pwm_outputs(const pwm_output_t *outputs, size_t output_count, uint16_t pulse_us)
{
    uint32_t duty = pulse_us_to_duty(pulse_us);

    for (size_t i = 0; i < output_count; i++) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, outputs[i].ledc_channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, outputs[i].ledc_channel));
    }
}

static void set_all_outputs(uint16_t throttle_us, uint16_t reverse_us)
{
    current_throttle_us = throttle_us;
    current_reverse_us = reverse_us;

    set_pwm_outputs(throttle_outputs,
                    sizeof(throttle_outputs) / sizeof(throttle_outputs[0]),
                    throttle_us);
    set_pwm_outputs(reverse_outputs,
                    sizeof(reverse_outputs) / sizeof(reverse_outputs[0]),
                    reverse_us);
}

static void set_safe_outputs(void)
{
    set_all_outputs(esc_limits.throttle_min_us, esc_limits.reverse_low_us);
}

static void reset_drive_state(void)
{
    active_drive_direction = DRIVE_DIRECTION_FORWARD;
    pending_drive_direction = DRIVE_DIRECTION_FORWARD;
    drive_direction_hold_until_us = 0;
    receiver_failsafe_candidate_since_us = 0;
}

static void disarm_to_safe_outputs(const char *message)
{
    system_state = SYSTEM_DISARMED;
    reset_drive_state();
    set_safe_outputs();

    if (message != NULL) {
        printf("%s\n", message);
        fflush(stdout);
    }
}

static uint16_t ramp_toward(uint16_t current_us, uint16_t target_us, uint16_t step_us)
{
    if (current_us < target_us) {
        uint16_t next_us = current_us + step_us;
        return next_us > target_us ? target_us : next_us;
    }

    if (current_us > target_us) {
        uint16_t next_us = current_us - step_us;
        return next_us < target_us ? target_us : next_us;
    }

    return current_us;
}

static uint16_t reverse_pulse_for_direction(drive_direction_t direction)
{
    return direction == DRIVE_DIRECTION_REVERSE ? esc_limits.reverse_high_us : esc_limits.reverse_low_us;
}

static bool pulse_is_on_endpoint_side(uint32_t pulse_us, uint16_t endpoint_us, uint16_t neutral_us)
{
    int32_t pulse_delta = (int32_t)pulse_us - (int32_t)neutral_us;
    int32_t endpoint_delta = (int32_t)endpoint_us - (int32_t)neutral_us;

    if (pulse_delta == 0 || endpoint_delta == 0) {
        return false;
    }

    return (pulse_delta > 0) == (endpoint_delta > 0);
}

static uint16_t map_receiver_to_esc_throttle(uint32_t pulse_us, uint16_t endpoint_us)
{
    uint32_t input_span_us = abs((int32_t)endpoint_us - (int32_t)receiver_cal.neutral_us);
    uint32_t output_span_us = esc_limits.throttle_max_us - esc_limits.throttle_min_us;
    uint32_t input_delta_us = abs((int32_t)pulse_us - (int32_t)receiver_cal.neutral_us);

    if (input_span_us == 0) {
        return esc_limits.throttle_min_us;
    }

    if (input_delta_us > input_span_us) {
        input_delta_us = input_span_us;
    }

    uint32_t curved_delta_us = (input_delta_us * input_delta_us) / input_span_us;
    return esc_limits.throttle_min_us + (uint16_t)((curved_delta_us * output_span_us) / input_span_us);
}

static uint16_t apply_reverse_limit(uint16_t throttle_us)
{
    if (throttle_us <= esc_limits.throttle_min_us) {
        return esc_limits.throttle_min_us;
    }

    uint32_t throttle_delta_us = throttle_us - esc_limits.throttle_min_us;
    uint32_t limited_delta_us = (throttle_delta_us * drive_config.reverse_limit_percent) / 100;
    return esc_limits.throttle_min_us + (uint16_t)limited_delta_us;
}

static drive_command_t read_drive_command(void)
{
    drive_command_t command = {
        .throttle_us = esc_limits.throttle_min_us,
        .direction = DRIVE_DIRECTION_FORWARD,
        .moving = false,
    };

    if (!receiver_signal_valid()) {
        return command;
    }

    uint32_t pulse_us = receiver_pulse_us();
    int32_t neutral_delta_us = (int32_t)pulse_us - (int32_t)receiver_cal.neutral_us;
    if (abs(neutral_delta_us) <= receiver_cal.deadband_us) {
        return command;
    }

    if (pulse_is_on_endpoint_side(pulse_us, receiver_cal.full_throttle_us, receiver_cal.neutral_us)) {
        command.throttle_us = map_receiver_to_esc_throttle(pulse_us, receiver_cal.full_throttle_us);
        command.direction = DRIVE_DIRECTION_FORWARD;
        command.moving = command.throttle_us > esc_limits.throttle_min_us;
        return command;
    }

    if (pulse_is_on_endpoint_side(pulse_us, receiver_cal.full_reverse_us, receiver_cal.neutral_us)) {
        command.throttle_us = map_receiver_to_esc_throttle(pulse_us, receiver_cal.full_reverse_us);
        command.throttle_us = apply_reverse_limit(command.throttle_us);
        command.direction = DRIVE_DIRECTION_REVERSE;
        command.moving = command.throttle_us > esc_limits.throttle_min_us;
        return command;
    }

    return command;
}

static const char *state_name(system_state_t state)
{
    switch (state) {
    case SYSTEM_DISARMED:
        return "DISARMED";
    case SYSTEM_DRIVE_ARMED:
        return "DRIVE_ARMED";
    case SYSTEM_ESC_CAL_ARMED:
        return "ESC_CAL_ARMED";
    case SYSTEM_ESC_CAL_MAX:
        return "ESC_CAL_MAX";
    case SYSTEM_ESC_CAL_MIN:
        return "ESC_CAL_MIN";
    case SYSTEM_ESC_CAL_MANUAL:
        return "ESC_CAL_MANUAL";
    default:
        return "UNKNOWN";
    }
}

static void IRAM_ATTR rc_throttle_isr(void *arg)
{
    (void)arg;

    int level = gpio_get_level(PIN_RC_THROTTLE_INPUT);
    int64_t now_us = esp_timer_get_time();

    if (level) {
        rc_rise_time_us = now_us;
        return;
    }

    int64_t pulse_us = now_us - rc_rise_time_us;
    if (pulse_us >= 800 && pulse_us <= 2200) {
        rc_last_pulse_us = (uint32_t)pulse_us;
        rc_last_pulse_time_us = now_us;
    }
}

static bool receiver_signal_valid(void)
{
    int64_t pulse_time_us = rc_last_pulse_time_us;
    return pulse_time_us > 0 &&
           (esp_timer_get_time() - pulse_time_us) <= RECEIVER_SIGNAL_TIMEOUT_US;
}

static uint32_t receiver_pulse_us(void)
{
    return rc_last_pulse_us;
}

static bool receiver_pulse_matches_failsafe(void)
{
    if (!receiver_signal_valid()) {
        receiver_failsafe_candidate_since_us = 0;
        return false;
    }

    uint32_t pulse_us = receiver_pulse_us();
    int32_t delta_us = (int32_t)pulse_us - (int32_t)drive_config.receiver_failsafe_us;
    if (abs(delta_us) <= drive_config.receiver_failsafe_window_us) {
        return true;
    }

    receiver_failsafe_candidate_since_us = 0;
    return false;
}

static bool receiver_failsafe_active(void)
{
    if (!receiver_pulse_matches_failsafe()) {
        return false;
    }

    int64_t now_us = esp_timer_get_time();
    if (receiver_failsafe_candidate_since_us == 0) {
        receiver_failsafe_candidate_since_us = now_us;
        return false;
    }

    return (now_us - receiver_failsafe_candidate_since_us) >= RECEIVER_FAILSAFE_HOLD_US;
}

static bool receiver_is_neutral(void)
{
    if (!receiver_signal_valid()) {
        return false;
    }

    int32_t delta = (int32_t)receiver_pulse_us() - (int32_t)receiver_cal.neutral_us;
    return abs(delta) <= receiver_cal.deadband_us;
}

static bool arm_switch_raw_requested(void)
{
    return gpio_get_level(PIN_ARM_SWITCH) == 0;
}

static bool arm_switch_requested(void)
{
    return arm_switch_stable_on;
}

static void update_arm_switch(void)
{
    bool raw_on = arm_switch_raw_requested();
    int64_t now_us = esp_timer_get_time();

    if (raw_on != arm_switch_raw_on) {
        arm_switch_raw_on = raw_on;
        arm_switch_last_change_us = now_us;
    }

    if (raw_on != arm_switch_stable_on &&
        (now_us - arm_switch_last_change_us) >= ARM_SWITCH_DEBOUNCE_US) {
        arm_switch_stable_on = raw_on;

        if (arm_switch_stable_on) {
            arm_switch_turned_on_event = true;
        } else {
            arm_switch_turned_off_event = true;
            arm_switch_rearm_locked = false;
        }
    }
}

static bool drive_arm_allowed(void)
{
    if (!receiver_signal_valid()) {
        printf("ERR: receiver throttle signal missing\n");
        return false;
    }

    if (receiver_pulse_matches_failsafe()) {
        printf("ERR: receiver is outputting failsafe pulse (%lu us, failsafe %u +/- %u us)\n",
               (unsigned long)receiver_pulse_us(),
               drive_config.receiver_failsafe_us,
               drive_config.receiver_failsafe_window_us);
        return false;
    }

    if (!receiver_is_neutral()) {
        printf("ERR: receiver throttle is not neutral (%lu us, neutral %u +/- %u us)\n",
               (unsigned long)receiver_pulse_us(),
               receiver_cal.neutral_us,
               receiver_cal.deadband_us);
        return false;
    }

    return true;
}

static void arm_drive_mode(void)
{
    reset_drive_state();
    set_safe_outputs();
    system_state = SYSTEM_DRIVE_ARMED;
    printf("OK: drive mode armed\n");
}

static bool controller_is_disarmed(void)
{
    return system_state == SYSTEM_DISARMED ||
           system_state == SYSTEM_ESC_CAL_ARMED ||
           system_state == SYSTEM_ESC_CAL_MAX ||
           system_state == SYSTEM_ESC_CAL_MIN ||
           system_state == SYSTEM_ESC_CAL_MANUAL;
}

static bool calibration_command_allowed(void)
{
    if (!controller_is_disarmed()) {
        printf("ERR: controller is not disarmed\n");
        return false;
    }

    if (!receiver_signal_valid()) {
        printf("ERR: receiver throttle signal missing\n");
        return false;
    }

    if (!receiver_is_neutral()) {
        printf("ERR: receiver throttle is not neutral (%lu us, neutral %u +/- %u us)\n",
               (unsigned long)receiver_pulse_us(),
               receiver_cal.neutral_us,
               receiver_cal.deadband_us);
        return false;
    }

    return true;
}

static bool receiver_calibration_allowed(void)
{
    if (!controller_is_disarmed()) {
        printf("ERR: controller is not disarmed\n");
        return false;
    }

    if (!receiver_signal_valid()) {
        printf("ERR: receiver throttle signal missing\n");
        return false;
    }

    return true;
}

static bool manual_calibration_allowed(void)
{
    if (!controller_is_disarmed()) {
        printf("ERR: controller is not disarmed\n");
        return false;
    }

    if (!receiver_signal_valid()) {
        printf("ERR: receiver throttle signal missing\n");
        return false;
    }

    return true;
}

static esp_err_t save_receiver_calibration(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("cal", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u16(nvs, "rx_throttle", receiver_cal.full_throttle_us);
    if (err == ESP_OK) {
        err = nvs_set_u16(nvs, "rx_neutral", receiver_cal.neutral_us);
    }
    if (err == ESP_OK) {
        err = nvs_set_u16(nvs, "rx_reverse", receiver_cal.full_reverse_us);
    }
    if (err == ESP_OK) {
        err = nvs_set_u16(nvs, "rx_deadband", receiver_cal.deadband_us);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }

    nvs_close(nvs);
    return err;
}

static void load_receiver_calibration(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("cal", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return;
    }

    uint16_t full_throttle_us = 0;
    uint16_t neutral_us = 0;
    uint16_t full_reverse_us = 0;
    uint16_t deadband_us = 0;

    err = nvs_get_u16(nvs, "rx_throttle", &full_throttle_us);
    if (err == ESP_OK) {
        err = nvs_get_u16(nvs, "rx_neutral", &neutral_us);
    }
    if (err == ESP_OK) {
        err = nvs_get_u16(nvs, "rx_reverse", &full_reverse_us);
    }
    if (err == ESP_OK) {
        err = nvs_get_u16(nvs, "rx_deadband", &deadband_us);
    }

    nvs_close(nvs);

    if (err == ESP_OK &&
        full_throttle_us >= 800 &&
        full_throttle_us <= 2200 &&
        neutral_us >= 800 &&
        neutral_us <= 2200 &&
        full_reverse_us >= 800 &&
        full_reverse_us <= 2200 &&
        abs((int32_t)full_throttle_us - (int32_t)neutral_us) >= 150 &&
        abs((int32_t)full_reverse_us - (int32_t)neutral_us) >= 150 &&
        full_throttle_us != full_reverse_us &&
        deadband_us >= 10 &&
        deadband_us <= 150) {
        receiver_cal.full_throttle_us = full_throttle_us;
        receiver_cal.neutral_us = neutral_us;
        receiver_cal.full_reverse_us = full_reverse_us;
        receiver_cal.deadband_us = deadband_us;
        receiver_cal.loaded_from_nvs = true;
    }
}

static esp_err_t save_drive_config(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("cfg", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u8(nvs, "rev_limit", drive_config.reverse_limit_percent);
    if (err == ESP_OK) {
        err = nvs_set_u16(nvs, "fs_us", drive_config.receiver_failsafe_us);
    }
    if (err == ESP_OK) {
        err = nvs_set_u16(nvs, "fs_window", drive_config.receiver_failsafe_window_us);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }

    nvs_close(nvs);
    return err;
}

static void load_drive_config(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("cfg", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return;
    }

    uint8_t reverse_limit_percent = DEFAULT_REVERSE_LIMIT_PERCENT;
    uint16_t receiver_failsafe_us = DEFAULT_RECEIVER_FAILSAFE_US;
    uint16_t receiver_failsafe_window_us = DEFAULT_RECEIVER_FAILSAFE_WINDOW_US;
    err = nvs_get_u8(nvs, "rev_limit", &reverse_limit_percent);
    if (err == ESP_OK) {
        esp_err_t fs_err = nvs_get_u16(nvs, "fs_us", &receiver_failsafe_us);
        if (fs_err != ESP_OK && fs_err != ESP_ERR_NVS_NOT_FOUND) {
            err = fs_err;
        }
    }
    if (err == ESP_OK) {
        esp_err_t fs_err = nvs_get_u16(nvs, "fs_window", &receiver_failsafe_window_us);
        if (fs_err != ESP_OK && fs_err != ESP_ERR_NVS_NOT_FOUND) {
            err = fs_err;
        }
    }
    nvs_close(nvs);

    if (err == ESP_OK &&
        reverse_limit_percent <= 100 &&
        receiver_failsafe_us >= 800 &&
        receiver_failsafe_us <= 2200 &&
        receiver_failsafe_window_us >= 1 &&
        receiver_failsafe_window_us <= 100) {
        drive_config.reverse_limit_percent = reverse_limit_percent;
        drive_config.receiver_failsafe_us = receiver_failsafe_us;
        drive_config.receiver_failsafe_window_us = receiver_failsafe_window_us;
        drive_config.loaded_from_nvs = true;
    }
}

static uint16_t average_receiver_pulse(uint32_t sample_ms)
{
    uint32_t samples = 0;
    uint32_t sum = 0;
    int64_t end_us = esp_timer_get_time() + ((int64_t)sample_ms * 1000);

    while (esp_timer_get_time() < end_us) {
        if (receiver_signal_valid()) {
            sum += receiver_pulse_us();
            samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (samples == 0) {
        return 0;
    }

    return (uint16_t)(sum / samples);
}

static void wait_for_enter(const char *prompt)
{
    printf("%s Press Enter when ready.\n", prompt);
    fflush(stdout);

    int c = 0;
    do {
        c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } while (c != '\r' && c != '\n');
}

static uint16_t capture_receiver_position(const char *position_name)
{
    wait_for_enter(position_name);

    printf("Sampling %s for 1500 ms...\n", position_name);
    uint16_t pulse_us = average_receiver_pulse(1500);
    if (pulse_us == 0) {
        printf("ERR: receiver signal lost while sampling %s\n", position_name);
        return 0;
    }

    printf("Captured %s: %u us\n", position_name, pulse_us);
    return pulse_us;
}

static void run_receiver_calibration(void)
{
    if (!receiver_calibration_allowed()) {
        return;
    }

    printf("Receiver calibration started.\n");
    printf("This will capture neutral, full throttle, and full reverse as three separate positions.\n");

    uint16_t neutral_us = capture_receiver_position("neutral throttle");
    if (neutral_us == 0) {
        return;
    }

    uint16_t full_throttle_us = capture_receiver_position("full throttle");
    if (full_throttle_us == 0) {
        return;
    }

    uint16_t full_reverse_us = capture_receiver_position("full reverse");
    if (full_reverse_us == 0) {
        return;
    }

    int32_t throttle_delta = (int32_t)full_throttle_us - (int32_t)neutral_us;
    int32_t reverse_delta = (int32_t)full_reverse_us - (int32_t)neutral_us;

    if (abs(throttle_delta) < 150 || abs(reverse_delta) < 150 ||
        full_throttle_us == full_reverse_us ||
        ((throttle_delta > 0) == (reverse_delta > 0))) {
        printf("ERR: invalid receiver calibration throttle=%u neutral=%u reverse=%u\n",
               full_throttle_us,
               neutral_us,
               full_reverse_us);
        printf("ERR: full throttle and full reverse must be distinct and on opposite sides of neutral\n");
        return;
    }

    receiver_cal.full_throttle_us = full_throttle_us;
    receiver_cal.neutral_us = neutral_us;
    receiver_cal.full_reverse_us = full_reverse_us;
    receiver_cal.deadband_us = DEFAULT_RECEIVER_DEADBAND_US;
    receiver_cal.loaded_from_nvs = true;

    esp_err_t err = save_receiver_calibration();
    if (err != ESP_OK) {
        printf("ERR: failed to save receiver calibration: %s\n", esp_err_to_name(err));
        return;
    }

    printf("OK: receiver calibration saved throttle=%u neutral=%u reverse=%u deadband=%u\n",
           receiver_cal.full_throttle_us,
           receiver_cal.neutral_us,
           receiver_cal.full_reverse_us,
           receiver_cal.deadband_us);
}

static void print_status(void)
{
    printf("\nRC car powertrain status\n");
    printf("  state: %s\n", state_name(system_state));
    printf("  armed: %s\n", system_state == SYSTEM_DRIVE_ARMED ? "yes" : "no");
    printf("  arm switch: %s%s\n",
           arm_switch_requested() ? "on" : "off",
           arm_switch_rearm_locked ? ", rearm locked" : "");
    printf("  receiver: %s", receiver_signal_valid() ? "valid" : "missing");
    if (receiver_signal_valid()) {
        printf(", %lu us, %s",
               (unsigned long)receiver_pulse_us(),
               receiver_is_neutral() ? "neutral" : "not neutral");
        if (receiver_pulse_matches_failsafe()) {
            printf(", failsafe-pulse-match");
        }
    }
    printf("\n");
    printf("  receiver cal: throttle=%u neutral=%u reverse=%u deadband=%u source=%s\n",
           receiver_cal.full_throttle_us,
           receiver_cal.neutral_us,
           receiver_cal.full_reverse_us,
           receiver_cal.deadband_us,
           receiver_cal.loaded_from_nvs ? "nvs" : "defaults");
    printf("  esc limits: throttle_min=%u throttle_max=%u reverse_low=%u reverse_high=%u\n",
           esc_limits.throttle_min_us,
           esc_limits.throttle_max_us,
           esc_limits.reverse_low_us,
           esc_limits.reverse_high_us);
    printf("  drive config: reverse_limit=%u%% receiver_failsafe=%u +/- %u us source=%s\n",
           drive_config.reverse_limit_percent,
           drive_config.receiver_failsafe_us,
           drive_config.receiver_failsafe_window_us,
           drive_config.loaded_from_nvs ? "nvs" : "defaults");
    printf("  outputs: throttle=%u us to 4 channels, reverse=%u us to 4 channels\n",
           current_throttle_us,
           current_reverse_us);
    printf("  drive direction: %s\n\n",
           active_drive_direction == DRIVE_DIRECTION_REVERSE ? "reverse" : "forward");
}

static void monitor_throttle_signal(void)
{
    int64_t end_us = esp_timer_get_time() + THROTTLE_MONITOR_DURATION_US;

    printf("Monitoring receiver throttle for 30 seconds.\n");
    printf("Move the throttle through neutral, full throttle, and full reverse.\n");

    while (esp_timer_get_time() < end_us) {
        if (receiver_signal_valid()) {
            uint32_t pulse_us = receiver_pulse_us();
            int32_t neutral_delta_us = (int32_t)pulse_us - (int32_t)receiver_cal.neutral_us;

            printf("receiver throttle: %lu us, neutral_delta: %+ld us, %s\n",
                   (unsigned long)pulse_us,
                   (long)neutral_delta_us,
                   receiver_is_neutral() ? "neutral" : "not neutral");
        } else {
            printf("receiver throttle: missing signal\n");
        }

        vTaskDelay(pdMS_TO_TICKS(THROTTLE_MONITOR_INTERVAL_MS));
    }

    printf("Throttle monitor complete.\n");
}

static void monitor_arm_switch_signal(void)
{
    int64_t end_us = esp_timer_get_time() + ARM_SWITCH_MONITOR_DURATION_US;

    printf("Monitoring arm switch input for 30 seconds.\n");
    printf("Switch open should read HIGH/OFF; switch closed to ground should read LOW/ON.\n");

    while (esp_timer_get_time() < end_us) {
        update_arm_switch();

        bool raw_on = arm_switch_raw_requested();
        printf("arm switch: digital=%s raw_level=%d stable=%s%s\n",
               raw_on ? "LOW/ON" : "HIGH/OFF",
               gpio_get_level(PIN_ARM_SWITCH),
               arm_switch_requested() ? "ON" : "OFF",
               arm_switch_rearm_locked ? " rearm_locked" : "");

        vTaskDelay(pdMS_TO_TICKS(ARM_SWITCH_MONITOR_INTERVAL_MS));
    }

    printf("Arm switch monitor complete.\n");
}

static void update_drive_outputs(void)
{
    if (!receiver_signal_valid()) {
        arm_switch_rearm_locked = arm_switch_requested();
        disarm_to_safe_outputs("\nWARN: receiver signal lost; drive disarmed and safe outputs restored\n> ");
        return;
    }

    if (receiver_failsafe_active()) {
        arm_switch_rearm_locked = arm_switch_requested();
        disarm_to_safe_outputs("\nWARN: receiver failsafe pulse detected; drive disarmed and safe outputs restored\n> ");
        return;
    }

    drive_command_t command = read_drive_command();

    if (!command.moving) {
        active_drive_direction = DRIVE_DIRECTION_FORWARD;
        pending_drive_direction = DRIVE_DIRECTION_FORWARD;
        drive_direction_hold_until_us = 0;
        current_throttle_us = ramp_toward(current_throttle_us,
                                          esc_limits.throttle_min_us,
                                          DRIVE_DECEL_STEP_US);
        set_all_outputs(current_throttle_us, esc_limits.reverse_low_us);
        return;
    }

    int64_t now_us = esp_timer_get_time();
    if (command.direction != active_drive_direction) {
        if (pending_drive_direction != command.direction || drive_direction_hold_until_us == 0) {
            pending_drive_direction = command.direction;
            drive_direction_hold_until_us = now_us + DRIVE_DIRECTION_CHANGE_HOLD_US;
        }

        current_throttle_us = ramp_toward(current_throttle_us,
                                          esc_limits.throttle_min_us,
                                          DRIVE_DECEL_STEP_US);
        set_all_outputs(current_throttle_us, reverse_pulse_for_direction(active_drive_direction));

        if (now_us >= drive_direction_hold_until_us &&
            current_throttle_us == esc_limits.throttle_min_us) {
            active_drive_direction = pending_drive_direction;
            drive_direction_hold_until_us = 0;
        }
        return;
    }

    pending_drive_direction = command.direction;
    drive_direction_hold_until_us = 0;
    uint16_t ramp_step_us = command.throttle_us > current_throttle_us
                                ? DRIVE_ACCEL_STEP_US
                                : DRIVE_DECEL_STEP_US;
    current_throttle_us = ramp_toward(current_throttle_us,
                                      command.throttle_us,
                                      ramp_step_us);
    set_all_outputs(current_throttle_us, reverse_pulse_for_direction(active_drive_direction));
}

static void handle_arm_switch_control(void)
{
    update_arm_switch();

    if (arm_switch_turned_off_event) {
        arm_switch_turned_off_event = false;
        if (system_state == SYSTEM_DRIVE_ARMED) {
            disarm_to_safe_outputs("\nOK: arm switch off; drive mode disarmed and safe outputs restored\n> ");
        }
    }

    if (system_state == SYSTEM_DRIVE_ARMED && !arm_switch_requested()) {
        disarm_to_safe_outputs("\nOK: arm switch off; drive mode disarmed and safe outputs restored\n> ");
    }

    if (arm_switch_turned_on_event) {
        arm_switch_turned_on_event = false;

        if (system_state != SYSTEM_DISARMED) {
            return;
        }

        if (arm_switch_rearm_locked) {
            printf("\nWARN: re-arm locked after failsafe; cycle arm switch OFF then ON\n> ");
            fflush(stdout);
            return;
        }

        if (drive_arm_allowed()) {
            arm_drive_mode();
            printf("> ");
            fflush(stdout);
        }
    }
}

static void print_help(void)
{
    printf("\nCommands:\n");
    printf("  status       Print state, receiver, calibration, and output pulse data\n");
    printf("  arm          Enter drive mode; requires arm switch on and neutral receiver throttle\n");
    printf("  disarm       Leave drive mode and restore safe outputs; arm switch off also disarms\n");
    printf("  config reverse <0-100>\n");
    printf("               Set max reverse throttle as percent of full ESC throttle\n");
    printf("  config failsafe <pulse_us> <window_us>\n");
    printf("               Set receiver transmitter-off failsafe pulse, default 1565 +/- 10 us\n");
    printf("  monitor throttle\n");
    printf("               Print receiver throttle pulse width for 30 seconds\n");
    printf("  monitor arm  Print arm switch digital state for 30 seconds\n");
    printf("  cal receiver Capture receiver neutral, full throttle, and full reverse\n");
    printf("  cal esc arm  Prepare ESC calibration while ESC battery remains disconnected\n");
    printf("  cal esc max  Output throttle max + reverse low for ESC max endpoint\n");
    printf("  cal esc min  Output throttle min + reverse low for ESC min endpoint\n");
    printf("  cal manual   Relay receiver throttle directly to ESC throttle outputs for 30 seconds\n");
    printf("  cal cancel   Return to disarmed safe outputs\n");
    printf("  help         Show this command list\n\n");
}

static void handle_command(char *line)
{
    while (isspace((unsigned char)*line)) {
        line++;
    }

    size_t len = strlen(line);
    while (len > 0 && isspace((unsigned char)line[len - 1])) {
        line[--len] = '\0';
    }

    for (size_t i = 0; line[i] != '\0'; i++) {
        line[i] = (char)tolower((unsigned char)line[i]);
    }

    if (line[0] == '\0') {
        return;
    }

    if (strcmp(line, "status") == 0) {
        print_status();
        return;
    }

    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        print_help();
        return;
    }

    if (strncmp(line, "config reverse ", strlen("config reverse ")) == 0) {
        if (system_state != SYSTEM_DISARMED) {
            printf("ERR: reverse limit can only be changed while DISARMED\n");
            return;
        }

        const char *percent_text = line + strlen("config reverse ");
        char *end = NULL;
        unsigned long percent = strtoul(percent_text, &end, 10);
        if (percent_text[0] == '\0' || *end != '\0' || percent > 100) {
            printf("ERR: usage is 'config reverse <0-100>'\n");
            return;
        }

        drive_config.reverse_limit_percent = (uint8_t)percent;
        drive_config.loaded_from_nvs = true;

        esp_err_t err = save_drive_config();
        if (err != ESP_OK) {
            printf("ERR: failed to save reverse limit: %s\n", esp_err_to_name(err));
            return;
        }

        printf("OK: reverse limit set to %lu%%\n", percent);
        return;
    }

    if (strncmp(line, "config failsafe ", strlen("config failsafe ")) == 0) {
        if (system_state != SYSTEM_DISARMED) {
            printf("ERR: receiver failsafe config can only be changed while DISARMED\n");
            return;
        }

        const char *args = line + strlen("config failsafe ");
        char *end = NULL;
        unsigned long pulse_us = strtoul(args, &end, 10);
        if (args[0] == '\0' || !isspace((unsigned char)*end)) {
            printf("ERR: usage is 'config failsafe <pulse_us> <window_us>'\n");
            return;
        }

        while (isspace((unsigned char)*end)) {
            end++;
        }

        char *window_end = NULL;
        unsigned long window_us = strtoul(end, &window_end, 10);
        if (end[0] == '\0' || *window_end != '\0' ||
            pulse_us < 800 || pulse_us > 2200 ||
            window_us < 1 || window_us > 100) {
            printf("ERR: failsafe pulse must be 800-2200 us and window must be 1-100 us\n");
            return;
        }

        drive_config.receiver_failsafe_us = (uint16_t)pulse_us;
        drive_config.receiver_failsafe_window_us = (uint16_t)window_us;
        drive_config.loaded_from_nvs = true;
        receiver_failsafe_candidate_since_us = 0;

        esp_err_t err = save_drive_config();
        if (err != ESP_OK) {
            printf("ERR: failed to save receiver failsafe config: %s\n", esp_err_to_name(err));
            return;
        }

        printf("OK: receiver failsafe set to %lu +/- %lu us\n", pulse_us, window_us);
        return;
    }

    if (strcmp(line, "arm") == 0) {
        if (system_state != SYSTEM_DISARMED) {
            printf("ERR: controller must be DISARMED before arming drive mode\n");
            return;
        }
        if (!arm_switch_requested()) {
            printf("ERR: arm switch is OFF\n");
            return;
        }
        if (arm_switch_rearm_locked) {
            printf("ERR: re-arm locked after failsafe; cycle arm switch OFF then ON\n");
            return;
        }
        if (!drive_arm_allowed()) {
            return;
        }

        arm_drive_mode();
        return;
    }

    if (strcmp(line, "disarm") == 0 || strcmp(line, "stop") == 0) {
        disarm_to_safe_outputs("OK: drive mode disarmed; safe outputs restored");
        return;
    }

    if (strcmp(line, "monitor throttle") == 0 || strcmp(line, "mon throttle") == 0) {
        monitor_throttle_signal();
        return;
    }

    if (strcmp(line, "monitor arm") == 0 || strcmp(line, "mon arm") == 0) {
        monitor_arm_switch_signal();
        return;
    }

    if (strcmp(line, "cal receiver") == 0) {
        run_receiver_calibration();
        return;
    }

    if (strcmp(line, "cal cancel") == 0) {
        system_state = SYSTEM_DISARMED;
        receiver_failsafe_candidate_since_us = 0;
        set_safe_outputs();
        printf("OK: calibration canceled; safe outputs restored\n");
        return;
    }

    if (strcmp(line, "cal manual") == 0) {
        if (!manual_calibration_allowed()) {
            return;
        }
        system_state = SYSTEM_ESC_CAL_MANUAL;
        esc_cal_last_command_us = esp_timer_get_time();
        printf("OK: manual ESC calibration relay active for 30 seconds.\n");
        printf("Receiver throttle will be copied directly to all ESC throttle outputs; reverse outputs stay low.\n");
        printf("Use 'cal cancel' to stop early.\n");
        return;
    }

    if (strcmp(line, "cal esc arm") == 0) {
        if (!calibration_command_allowed()) {
            return;
        }
        system_state = SYSTEM_ESC_CAL_ARMED;
        esc_cal_last_command_us = esp_timer_get_time();
        set_safe_outputs();
        printf("OK: ESC calibration armed. Keep ESC battery disconnected, then send 'cal esc max'.\n");
        return;
    }

    if (strcmp(line, "cal esc max") == 0) {
        if (system_state != SYSTEM_ESC_CAL_ARMED &&
            system_state != SYSTEM_ESC_CAL_MAX &&
            system_state != SYSTEM_ESC_CAL_MIN) {
            printf("ERR: send 'cal esc arm' before 'cal esc max'\n");
            return;
        }
        if (!calibration_command_allowed()) {
            return;
        }
        system_state = SYSTEM_ESC_CAL_MAX;
        esc_cal_last_command_us = esp_timer_get_time();
        set_all_outputs(esc_limits.throttle_max_us, esc_limits.reverse_low_us);
        printf("OK: throttle max output active. Power ESCs now; after max-endpoint beeps, send 'cal esc min'.\n");
        return;
    }

    if (strcmp(line, "cal esc min") == 0) {
        if (system_state != SYSTEM_ESC_CAL_MAX) {
            printf("ERR: send 'cal esc max' before 'cal esc min'\n");
            return;
        }
        if (!calibration_command_allowed()) {
            return;
        }
        system_state = SYSTEM_ESC_CAL_MIN;
        esc_cal_last_command_us = esp_timer_get_time();
        set_all_outputs(esc_limits.throttle_min_us, esc_limits.reverse_low_us);
        printf("OK: throttle min output active. Wait for ESC ready beeps, then send 'cal cancel'.\n");
        return;
    }

    printf("ERR: unknown command '%s'. Type 'help'.\n", line);
}

static void cli_task(void *arg)
{
    (void)arg;

    char line[CLI_LINE_MAX];
    size_t line_len = 0;

    print_help();
    printf("> ");
    fflush(stdout);

    while (true) {
        int c = getchar();

        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (c == '\r' || c == '\n') {
            putchar('\n');
            line[line_len] = '\0';
            handle_command(line);
            line_len = 0;
            printf("> ");
            fflush(stdout);
            continue;
        }

        if (c == 0x08 || c == 0x7f) {
            if (line_len > 0) {
                line_len--;
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }

        if (isprint(c) && line_len < (CLI_LINE_MAX - 1)) {
            line[line_len++] = (char)c;
            putchar(c);
            fflush(stdout);
        }
    }
}

static void powertrain_task(void *arg)
{
    (void)arg;

    while (true) {
        handle_arm_switch_control();

        if (system_state == SYSTEM_DRIVE_ARMED) {
            update_drive_outputs();
        }

        if (system_state == SYSTEM_ESC_CAL_MANUAL) {
            if (receiver_failsafe_active()) {
                system_state = SYSTEM_DISARMED;
                receiver_failsafe_candidate_since_us = 0;
                set_safe_outputs();
                printf("\nWARN: receiver failsafe pulse detected; manual ESC calibration relay stopped\n> ");
                fflush(stdout);
            } else if (receiver_signal_valid()) {
                set_all_outputs((uint16_t)receiver_pulse_us(), esc_limits.reverse_low_us);
            } else {
                system_state = SYSTEM_DISARMED;
                receiver_failsafe_candidate_since_us = 0;
                set_safe_outputs();
                printf("\nWARN: receiver signal lost; manual ESC calibration relay stopped\n> ");
                fflush(stdout);
            }
        }

        if (system_state == SYSTEM_ESC_CAL_ARMED ||
            system_state == SYSTEM_ESC_CAL_MAX ||
            system_state == SYSTEM_ESC_CAL_MIN ||
            system_state == SYSTEM_ESC_CAL_MANUAL) {
            int64_t since_last_command_us = esp_timer_get_time() - esc_cal_last_command_us;
            if (since_last_command_us > ESC_CAL_TIMEOUT_US) {
                system_state = SYSTEM_DISARMED;
                receiver_failsafe_candidate_since_us = 0;
                set_safe_outputs();
                printf("\nWARN: ESC calibration timed out; safe outputs restored\n> ");
                fflush(stdout);
            }
        }

        vTaskDelay((system_state == SYSTEM_DRIVE_ARMED || system_state == SYSTEM_ESC_CAL_MANUAL)
                       ? pdMS_TO_TICKS(DRIVE_UPDATE_INTERVAL_MS)
                       : pdMS_TO_TICKS(100));
    }
}

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void init_pwm_outputs(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_FRAME_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    const size_t throttle_count = sizeof(throttle_outputs) / sizeof(throttle_outputs[0]);
    const size_t reverse_count = sizeof(reverse_outputs) / sizeof(reverse_outputs[0]);

    for (size_t i = 0; i < throttle_count; i++) {
        ledc_channel_config_t channel_config = {
            .gpio_num = throttle_outputs[i].gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = throttle_outputs[i].ledc_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = pulse_us_to_duty(esc_limits.throttle_min_us),
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
            .flags.output_invert = 0,
            .deconfigure = false,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    }

    for (size_t i = 0; i < reverse_count; i++) {
        ledc_channel_config_t channel_config = {
            .gpio_num = reverse_outputs[i].gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = reverse_outputs[i].ledc_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = pulse_us_to_duty(esc_limits.reverse_low_us),
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
            .flags.output_invert = 0,
            .deconfigure = false,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    }

    set_safe_outputs();
}

static void init_receiver_input(void)
{
    gpio_config_t input_config = {
        .pin_bit_mask = 1ULL << PIN_RC_THROTTLE_INPUT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_RC_THROTTLE_INPUT, rc_throttle_isr, NULL));
}

static void init_arm_switch_input(void)
{
    gpio_config_t input_config = {
        .pin_bit_mask = 1ULL << PIN_ARM_SWITCH,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_config));

    arm_switch_raw_on = arm_switch_raw_requested();
    arm_switch_stable_on = arm_switch_raw_on;
    arm_switch_last_change_us = esp_timer_get_time();
}

void app_main(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    init_pwm_outputs();
    init_nvs();
    load_receiver_calibration();
    load_drive_config();
    init_receiver_input();
    init_arm_switch_input();

    ESP_LOGI(TAG, "RC car powertrain controller started");
    ESP_LOGI(TAG, "USB serial CLI baud: 115200");
    ESP_LOGI(TAG, "Safe outputs active: throttle=%u us reverse=%u us",
             current_throttle_us,
             current_reverse_us);

    xTaskCreate(cli_task, "serial_cli", 4096, NULL, 5, NULL);
    xTaskCreate(powertrain_task, "powertrain", 4096, NULL, 4, NULL);
}
