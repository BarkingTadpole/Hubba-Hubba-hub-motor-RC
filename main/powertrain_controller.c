#include "powertrain_controller.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "config_store.h"
#include "esc_output.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_sensor.h"
#include "powertrain_types.h"
#include "rc_input.h"
#include "rpm_sensor.h"
#include "torque_vectoring.h"

#define ESC_THROTTLE_MIN_US 1100
#define ESC_THROTTLE_MAX_US 1940
#define ESC_REVERSE_LOW_US 1100
#define ESC_REVERSE_HIGH_US 1940
#define DRIVE_UPDATE_INTERVAL_MS 20
#define SENSOR_UPDATE_INTERVAL_MS 5
#define RPM_UPDATE_INTERVAL_MS 20
#define DIRECTION_CHANGE_HOLD_US 120000
#define DRIVE_ACCEL_STEP_US 12
#define DRIVE_DECEL_STEP_US 100
#define FAILSAFE_CONFIRM_US 60000
#define ESC_CAL_TIMEOUT_US 30000000
#define MONITOR_DURATION_US 30000000
#define MONITOR_INTERVAL_MS 250

typedef struct {
    uint16_t throttle_us;
    drive_direction_t direction;
    bool moving;
} drive_command_t;

static const char *TAG = "powertrain";
static const char *wheel_names[POWERTRAIN_WHEEL_COUNT] = {"FL", "FR", "RL", "RR"};

static const esc_limits_t esc_limits = {
    .throttle_min_us = ESC_THROTTLE_MIN_US,
    .throttle_max_us = ESC_THROTTLE_MAX_US,
    .reverse_low_us = ESC_REVERSE_LOW_US,
    .reverse_high_us = ESC_REVERSE_HIGH_US,
};

static throttle_calibration_t throttle_cal;
static steering_calibration_t steering_cal;
static arm_calibration_t arm_cal;
static tv_mode_calibration_t tv_mode_cal;
static drive_config_t drive_config;

static volatile system_state_t system_state = SYSTEM_DISARMED;
static volatile bool calibration_in_progress;
static bool rpm_initialized;
static bool imu_initialized;
static int64_t esc_cal_last_command_us;
static int64_t failsafe_candidate_since_us;
static uint16_t current_base_throttle_us = ESC_THROTTLE_MIN_US;
static drive_direction_t active_direction = DRIVE_DIRECTION_FORWARD;
static drive_direction_t pending_direction = DRIVE_DIRECTION_FORWARD;
static int64_t direction_hold_until_us;
static bool arm_cycle_ready;
static bool previous_arm_run;
static bool rpm_seen[POWERTRAIN_WHEEL_COUNT];
static torque_vectoring_state_t vectoring_state;
static torque_vectoring_output_t vectoring_output;

static const char *state_name(system_state_t state)
{
    switch (state) {
    case SYSTEM_DISARMED: return "DISARMED";
    case SYSTEM_DRIVE_ARMED: return "DRIVE_ARMED";
    case SYSTEM_ESC_CAL_ARMED: return "ESC_CAL_ARMED";
    case SYSTEM_ESC_CAL_MAX: return "ESC_CAL_MAX";
    case SYSTEM_ESC_CAL_MIN: return "ESC_CAL_MIN";
    case SYSTEM_ESC_CAL_MANUAL: return "ESC_CAL_MANUAL";
    default: return "UNKNOWN";
    }
}

static const char *tv_mode_name(tv_mode_t mode)
{
    switch (mode) {
    case TV_MODE_OFF: return "OFF";
    case TV_MODE_STRAIGHT: return "STRAIGHT";
    case TV_MODE_FULL: return "FULL";
    default: return "UNKNOWN";
    }
}

static uint16_t ramp_toward(uint16_t current, uint16_t target, uint16_t step)
{
    if (current < target) {
        uint32_t next = (uint32_t)current + step;
        return next > target ? target : (uint16_t)next;
    }
    if (current > target) {
        return current - target <= step ? target : current - step;
    }
    return current;
}

static uint16_t reverse_pulse_for_direction(drive_direction_t direction)
{
    return direction == DRIVE_DIRECTION_REVERSE
               ? esc_limits.reverse_high_us
               : esc_limits.reverse_low_us;
}

static bool pulse_is_on_endpoint_side(uint16_t pulse_us,
                                      uint16_t endpoint_us,
                                      uint16_t center_us)
{
    int32_t pulse_delta = (int32_t)pulse_us - center_us;
    int32_t endpoint_delta = (int32_t)endpoint_us - center_us;
    return pulse_delta != 0 && endpoint_delta != 0 &&
           ((pulse_delta > 0) == (endpoint_delta > 0));
}

static bool throttle_sample(rc_channel_sample_t *sample)
{
    return rc_input_get(RC_CHANNEL_THROTTLE, sample);
}

static bool throttle_is_neutral(void)
{
    rc_channel_sample_t sample;
    if (!throttle_sample(&sample)) {
        return false;
    }
    return abs((int32_t)sample.pulse_us - (int32_t)throttle_cal.neutral_us) <=
           throttle_cal.deadband_us;
}

static bool throttle_matches_failsafe(void)
{
    rc_channel_sample_t sample;
    if (!throttle_sample(&sample)) {
        failsafe_candidate_since_us = 0;
        return false;
    }

    bool matches = abs((int32_t)sample.pulse_us -
                       (int32_t)drive_config.receiver_failsafe_us) <=
                   drive_config.receiver_failsafe_window_us;
    if (!matches) {
        failsafe_candidate_since_us = 0;
    }
    return matches;
}

static bool throttle_failsafe_confirmed(void)
{
    if (!throttle_matches_failsafe()) {
        return false;
    }

    int64_t now_us = esp_timer_get_time();
    if (failsafe_candidate_since_us == 0) {
        failsafe_candidate_since_us = now_us;
        return false;
    }
    return now_us - failsafe_candidate_since_us >= FAILSAFE_CONFIRM_US;
}

static bool arm_requests_run(uint16_t pulse_us)
{
    int run_distance = abs((int32_t)pulse_us - (int32_t)arm_cal.run_us);
    int stop_distance = abs((int32_t)pulse_us - (int32_t)arm_cal.stop_us);
    return run_distance < stop_distance;
}

static bool arm_channel_state(bool *run)
{
    rc_channel_sample_t sample;
    if (!rc_input_get(RC_CHANNEL_ARM, &sample)) {
        return false;
    }
    *run = arm_requests_run(sample.pulse_us);
    return true;
}

static bool shutdown_channel_is_safe(void)
{
    if (!arm_cal.loaded_from_nvs) {
        return true;
    }

    bool arm_run = false;
    return arm_channel_state(&arm_run) && !arm_run;
}

static float steering_normalized(void)
{
    rc_channel_sample_t sample;
    if (!steering_cal.loaded_from_nvs ||
        !rc_input_get(RC_CHANNEL_STEERING, &sample)) {
        return 0.0f;
    }

    int32_t delta = (int32_t)sample.pulse_us - steering_cal.center_us;
    if (abs(delta) <= steering_cal.deadband_us) {
        return 0.0f;
    }

    uint16_t endpoint = pulse_is_on_endpoint_side(sample.pulse_us,
                                                  steering_cal.right_us,
                                                  steering_cal.center_us)
                            ? steering_cal.right_us
                            : steering_cal.left_us;
    int32_t span = abs((int32_t)endpoint - steering_cal.center_us);
    float magnitude = span > 0 ? (float)abs(delta) / span : 0.0f;
    if (magnitude > 1.0f) {
        magnitude = 1.0f;
    }
    return endpoint == steering_cal.right_us ? magnitude : -magnitude;
}

static tv_mode_t requested_tv_mode(void)
{
    rc_channel_sample_t sample;
    if (!tv_mode_cal.loaded_from_nvs ||
        !rc_input_get(RC_CHANNEL_TV_MODE, &sample)) {
        return TV_MODE_OFF;
    }

    int off_distance = abs((int32_t)sample.pulse_us - tv_mode_cal.off_us);
    int straight_distance = abs((int32_t)sample.pulse_us - tv_mode_cal.straight_us);
    int full_distance = abs((int32_t)sample.pulse_us - tv_mode_cal.full_us);
    if (off_distance <= straight_distance && off_distance <= full_distance) {
        return TV_MODE_OFF;
    }
    return straight_distance <= full_distance ? TV_MODE_STRAIGHT : TV_MODE_FULL;
}

static void reset_drive_state(void)
{
    current_base_throttle_us = esc_limits.throttle_min_us;
    active_direction = DRIVE_DIRECTION_FORWARD;
    pending_direction = DRIVE_DIRECTION_FORWARD;
    direction_hold_until_us = 0;
    failsafe_candidate_since_us = 0;
    torque_vectoring_reset(&vectoring_state);
    vectoring_output = (torque_vectoring_output_t){
        .inactive_reason = "drive is disarmed",
    };
}

static void disarm_to_safe(const char *message, bool require_arm_cycle)
{
    bool was_armed = system_state == SYSTEM_DRIVE_ARMED;
    system_state = SYSTEM_DISARMED;
    reset_drive_state();
    esc_output_set_safe();
    if (require_arm_cycle) {
        arm_cycle_ready = false;
    }
    if (message != NULL && was_armed) {
        printf("\n%s\n> ", message);
        fflush(stdout);
    }
}

static bool drive_arm_allowed(void)
{
    rc_channel_sample_t throttle;
    bool arm_run = false;

    if (!throttle_cal.loaded_from_nvs) {
        printf("ERR: run 'cal receiver' before arming\n");
        return false;
    }
    if (!arm_cal.loaded_from_nvs) {
        printf("ERR: run 'cal arm' before arming\n");
        return false;
    }
    if (!throttle_sample(&throttle)) {
        printf("ERR: receiver throttle signal missing\n");
        return false;
    }
    if (!arm_channel_state(&arm_run)) {
        printf("ERR: receiver arm channel signal missing\n");
        return false;
    }
    if (!arm_run) {
        printf("ERR: receiver arm switch is in STOP\n");
        return false;
    }
    if (throttle_matches_failsafe()) {
        printf("ERR: receiver is outputting the configured throttle failsafe pulse\n");
        return false;
    }
    if (!throttle_is_neutral()) {
        printf("ERR: receiver throttle is not neutral (%u us, neutral %u +/- %u us)\n",
               throttle.pulse_us, throttle_cal.neutral_us, throttle_cal.deadband_us);
        return false;
    }
    return true;
}

static void arm_drive(void)
{
    reset_drive_state();
    esc_output_set_safe();
    system_state = SYSTEM_DRIVE_ARMED;
    arm_cycle_ready = false;
    printf("OK: drive mode armed\n");
}

static drive_command_t read_drive_command(void)
{
    drive_command_t command = {
        .throttle_us = esc_limits.throttle_min_us,
        .direction = DRIVE_DIRECTION_FORWARD,
    };
    rc_channel_sample_t sample;
    if (!throttle_sample(&sample)) {
        return command;
    }

    int32_t neutral_delta = (int32_t)sample.pulse_us - throttle_cal.neutral_us;
    if (abs(neutral_delta) <= throttle_cal.deadband_us) {
        return command;
    }

    uint16_t endpoint = 0;
    if (pulse_is_on_endpoint_side(sample.pulse_us,
                                  throttle_cal.full_throttle_us,
                                  throttle_cal.neutral_us)) {
        endpoint = throttle_cal.full_throttle_us;
        command.direction = DRIVE_DIRECTION_FORWARD;
    } else if (pulse_is_on_endpoint_side(sample.pulse_us,
                                         throttle_cal.full_reverse_us,
                                         throttle_cal.neutral_us)) {
        endpoint = throttle_cal.full_reverse_us;
        command.direction = DRIVE_DIRECTION_REVERSE;
    } else {
        return command;
    }

    uint32_t input_span = abs((int32_t)endpoint - throttle_cal.neutral_us);
    uint32_t input_delta = abs((int32_t)sample.pulse_us - throttle_cal.neutral_us);
    if (input_span == 0) {
        return command;
    }
    if (input_delta > input_span) {
        input_delta = input_span;
    }

    float normalized = (float)input_delta / input_span;
    normalized *= normalized;
    if (command.direction == DRIVE_DIRECTION_REVERSE) {
        normalized *= (float)drive_config.reverse_limit_percent / 100.0f;
    }

    uint16_t output_span = esc_limits.throttle_max_us - esc_limits.throttle_min_us;
    command.throttle_us = esc_limits.throttle_min_us +
                          (uint16_t)lroundf(normalized * output_span);
    command.moving = command.throttle_us > esc_limits.throttle_min_us;
    return command;
}

static void set_drive_outputs_with_vectoring(void)
{
    wheel_output_command_t command;
    uint16_t output_span = esc_limits.throttle_max_us - esc_limits.throttle_min_us;
    float base_throttle = (float)(current_base_throttle_us - esc_limits.throttle_min_us) /
                          output_span;

    rpm_snapshot_t rpm = {0};
    imu_snapshot_t imu = {0};
    if (rpm_initialized) {
        rpm_sensor_get_snapshot(&rpm);
    }
    if (imu_initialized) {
        imu_sensor_get_snapshot(&imu);
    }

    torque_vectoring_input_t input = {
        .requested_mode = requested_tv_mode(),
        .direction = active_direction,
        .base_throttle = base_throttle,
        .steering = steering_normalized(),
        .rpm = rpm,
        .imu = imu,
        .dt_seconds = (float)DRIVE_UPDATE_INTERVAL_MS / 1000.0f,
    };
    torque_vectoring_update(&vectoring_state, &drive_config, &input, &vectoring_output);

    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        int32_t corrected = current_base_throttle_us +
                            (int32_t)lroundf(vectoring_output.wheel_correction[wheel] *
                                             output_span);
        if (corrected < esc_limits.throttle_min_us) corrected = esc_limits.throttle_min_us;
        if (corrected > esc_limits.throttle_max_us) corrected = esc_limits.throttle_max_us;
        command.throttle_us[wheel] = (uint16_t)corrected;
        command.reverse_us[wheel] = reverse_pulse_for_direction(active_direction);
    }
    esc_output_set_wheels(&command);
}

static void update_drive_outputs(void)
{
    rc_channel_sample_t throttle;
    if (!throttle_sample(&throttle)) {
        disarm_to_safe("WARN: throttle signal lost; drive disarmed", true);
        return;
    }

    if (throttle_matches_failsafe()) {
        current_base_throttle_us = esc_limits.throttle_min_us;
        esc_output_set_safe();
        if (throttle_failsafe_confirmed()) {
            disarm_to_safe("WARN: receiver failsafe pulse detected; drive disarmed", true);
        }
        return;
    }

    drive_command_t drive_command = read_drive_command();
    if (!drive_command.moving) {
        active_direction = DRIVE_DIRECTION_FORWARD;
        pending_direction = DRIVE_DIRECTION_FORWARD;
        direction_hold_until_us = 0;
        current_base_throttle_us = ramp_toward(current_base_throttle_us,
                                               esc_limits.throttle_min_us,
                                               DRIVE_DECEL_STEP_US);
        set_drive_outputs_with_vectoring();
        return;
    }

    int64_t now_us = esp_timer_get_time();
    if (drive_command.direction != active_direction) {
        if (pending_direction != drive_command.direction || direction_hold_until_us == 0) {
            pending_direction = drive_command.direction;
            direction_hold_until_us = now_us + DIRECTION_CHANGE_HOLD_US;
        }

        current_base_throttle_us = ramp_toward(current_base_throttle_us,
                                               esc_limits.throttle_min_us,
                                               DRIVE_DECEL_STEP_US);
        set_drive_outputs_with_vectoring();
        if (now_us >= direction_hold_until_us &&
            current_base_throttle_us == esc_limits.throttle_min_us) {
            active_direction = pending_direction;
            direction_hold_until_us = 0;
        }
        return;
    }

    pending_direction = drive_command.direction;
    direction_hold_until_us = 0;
    uint16_t step = drive_command.throttle_us > current_base_throttle_us
                        ? DRIVE_ACCEL_STEP_US
                        : DRIVE_DECEL_STEP_US;
    current_base_throttle_us = ramp_toward(current_base_throttle_us,
                                           drive_command.throttle_us,
                                           step);
    set_drive_outputs_with_vectoring();
}

static void handle_receiver_arm_control(void)
{
    bool arm_run = false;
    bool arm_valid = arm_channel_state(&arm_run);
    bool healthy_throttle = rc_input_is_valid(RC_CHANNEL_THROTTLE) &&
                            !throttle_matches_failsafe();
    bool esc_calibration_active = system_state == SYSTEM_ESC_CAL_ARMED ||
                                  system_state == SYSTEM_ESC_CAL_MAX ||
                                  system_state == SYSTEM_ESC_CAL_MIN ||
                                  system_state == SYSTEM_ESC_CAL_MANUAL;

    if (esc_calibration_active && arm_cal.loaded_from_nvs &&
        (!arm_valid || arm_run)) {
        powertrain_cal_cancel();
        printf("\nWARN: ESC calibration stopped because the receiver shutdown "
               "channel is not in STOP\n> ");
        fflush(stdout);
        previous_arm_run = arm_valid && arm_run;
        return;
    }

    if (!arm_valid) {
        if (system_state == SYSTEM_DRIVE_ARMED) {
            disarm_to_safe("WARN: receiver arm channel lost; drive disarmed", true);
        }
        previous_arm_run = false;
        return;
    }

    if (!arm_run) {
        if (system_state == SYSTEM_DRIVE_ARMED) {
            disarm_to_safe("OK: receiver arm switch moved to STOP; drive disarmed", false);
        }
        if (healthy_throttle && !calibration_in_progress) {
            arm_cycle_ready = true;
        }
    } else if (!previous_arm_run && arm_cycle_ready &&
               system_state == SYSTEM_DISARMED && !calibration_in_progress) {
        if (drive_arm_allowed()) {
            arm_drive();
        }
    }
    previous_arm_run = arm_run;
}

static bool calibration_allowed(bool require_neutral)
{
    if (system_state != SYSTEM_DISARMED) {
        printf("ERR: controller must be DISARMED\n");
        return false;
    }
    if (!rc_input_is_valid(RC_CHANNEL_THROTTLE)) {
        printf("ERR: receiver throttle signal missing\n");
        return false;
    }
    if (require_neutral && !throttle_is_neutral()) {
        printf("ERR: receiver throttle is not neutral\n");
        return false;
    }

    if (!shutdown_channel_is_safe()) {
        printf("ERR: receiver arm switch must be in STOP for calibration\n");
        return false;
    }
    return true;
}

static void drain_serial_line_endings(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));
    while (getchar() != EOF) {
    }
}

static void wait_for_enter(const char *prompt)
{
    drain_serial_line_endings();
    printf("%s Press Enter when ready.\n", prompt);
    fflush(stdout);
    while (true) {
        int c = getchar();
        if (c == '\r' || c == '\n') {
            return;
        }
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static uint16_t average_channel(rc_channel_id_t channel, uint32_t duration_ms)
{
    uint32_t sum = 0;
    uint32_t samples = 0;
    int64_t end_at_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;
    while (esp_timer_get_time() < end_at_us) {
        rc_channel_sample_t sample;
        if (rc_input_get(channel, &sample)) {
            sum += sample.pulse_us;
            samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return samples == 0 ? 0 : (uint16_t)(sum / samples);
}

static uint16_t capture_channel(rc_channel_id_t channel, const char *position_name)
{
    wait_for_enter(position_name);
    printf("Sampling %s for 1500 ms...\n", position_name);
    uint16_t pulse_us = average_channel(channel, 1500);
    if (pulse_us == 0) {
        printf("ERR: signal lost while sampling %s\n", position_name);
    } else {
        printf("Captured %s: %u us\n", position_name, pulse_us);
    }
    return pulse_us;
}

static bool endpoints_opposite(uint16_t first, uint16_t center, uint16_t second)
{
    int32_t first_delta = (int32_t)first - center;
    int32_t second_delta = (int32_t)second - center;
    return abs(first_delta) >= 150 && abs(second_delta) >= 150 &&
           ((first_delta > 0) != (second_delta > 0));
}

void powertrain_calibrate_throttle(void)
{
    if (!calibration_allowed(false)) return;
    calibration_in_progress = true;
    printf("Receiver throttle calibration started.\n");
    uint16_t neutral = capture_channel(RC_CHANNEL_THROTTLE, "Hold neutral throttle.");
    uint16_t full = neutral ? capture_channel(RC_CHANNEL_THROTTLE, "Hold full throttle.") : 0;
    uint16_t reverse = full ? capture_channel(RC_CHANNEL_THROTTLE, "Hold full reverse.") : 0;

    if (neutral && full && reverse && endpoints_opposite(full, neutral, reverse)) {
        throttle_cal = (throttle_calibration_t){
            .full_throttle_us = full,
            .neutral_us = neutral,
            .full_reverse_us = reverse,
            .deadband_us = 80,
            .loaded_from_nvs = true,
        };
        esp_err_t err = config_store_save_throttle(&throttle_cal);
        printf(err == ESP_OK
                   ? "OK: throttle calibration saved\n"
                   : "ERR: failed to save throttle calibration: %s\n",
               err == ESP_OK ? "" : esp_err_to_name(err));
    } else if (neutral && full && reverse) {
        printf("ERR: throttle endpoints must be at least 150 us from opposite sides of neutral\n");
    }
    calibration_in_progress = false;
}

void powertrain_calibrate_steering(void)
{
    if (!calibration_allowed(true)) return;
    calibration_in_progress = true;
    printf("Steering calibration started. The steering PWM tap is read-only.\n");
    uint16_t center = capture_channel(RC_CHANNEL_STEERING, "Hold steering centered.");
    uint16_t left = center ? capture_channel(RC_CHANNEL_STEERING, "Hold full left steering.") : 0;
    uint16_t right = left ? capture_channel(RC_CHANNEL_STEERING, "Hold full right steering.") : 0;

    if (center && left && right && endpoints_opposite(left, center, right)) {
        steering_cal = (steering_calibration_t){
            .left_us = left,
            .center_us = center,
            .right_us = right,
            .deadband_us = 30,
            .loaded_from_nvs = true,
        };
        esp_err_t err = config_store_save_steering(&steering_cal);
        printf(err == ESP_OK
                   ? "OK: steering calibration saved\n"
                   : "ERR: failed to save steering calibration: %s\n",
               err == ESP_OK ? "" : esp_err_to_name(err));
    } else if (center && left && right) {
        printf("ERR: steering endpoints must be at least 150 us from opposite sides of center\n");
    }
    calibration_in_progress = false;
}

void powertrain_calibrate_arm(void)
{
    if (!calibration_allowed(true)) return;
    calibration_in_progress = true;
    printf("Receiver arm-channel calibration started.\n");
    uint16_t stop = capture_channel(RC_CHANNEL_ARM, "Put the shutdown switch in STOP.");
    uint16_t run = stop ? capture_channel(RC_CHANNEL_ARM, "Put the shutdown switch in RUN.") : 0;

    if (stop && run && abs((int32_t)stop - (int32_t)run) >= 250) {
        arm_cal = (arm_calibration_t){
            .run_us = run,
            .stop_us = stop,
            .loaded_from_nvs = true,
        };
        esp_err_t err = config_store_save_arm(&arm_cal);
        printf(err == ESP_OK
                   ? "OK: receiver arm-channel calibration saved\n"
                   : "ERR: failed to save arm calibration: %s\n",
               err == ESP_OK ? "" : esp_err_to_name(err));
    } else if (stop && run) {
        printf("ERR: RUN and STOP must differ by at least 250 us\n");
    }
    arm_cycle_ready = false;
    calibration_in_progress = false;
}

void powertrain_calibrate_tv_mode(void)
{
    if (!calibration_allowed(true)) return;
    calibration_in_progress = true;
    printf("CH5 torque-vectoring selector calibration started.\n");
    uint16_t off = capture_channel(RC_CHANNEL_TV_MODE, "Put CH5 in OFF.");
    uint16_t straight = off ? capture_channel(RC_CHANNEL_TV_MODE, "Put CH5 in STRAIGHT.") : 0;
    uint16_t full = straight ? capture_channel(RC_CHANNEL_TV_MODE, "Put CH5 in FULL.") : 0;

    bool distinct = off && straight && full &&
                    abs((int32_t)off - (int32_t)straight) >= 150 &&
                    abs((int32_t)straight - (int32_t)full) >= 150 &&
                    abs((int32_t)off - (int32_t)full) >= 300;
    if (distinct) {
        tv_mode_cal = (tv_mode_calibration_t){
            .off_us = off,
            .straight_us = straight,
            .full_us = full,
            .loaded_from_nvs = true,
        };
        esp_err_t err = config_store_save_tv_mode(&tv_mode_cal);
        printf(err == ESP_OK
                   ? "OK: CH5 mode calibration saved\n"
                   : "ERR: failed to save CH5 calibration: %s\n",
               err == ESP_OK ? "" : esp_err_to_name(err));
    } else if (off && straight && full) {
        printf("ERR: CH5 positions are not sufficiently distinct\n");
    }
    calibration_in_progress = false;
}

void powertrain_calibrate_imu(void)
{
    if (!calibration_allowed(true)) return;
    if (!imu_initialized) {
        printf("ERR: ISM330DHCX is not available\n");
        return;
    }

    calibration_in_progress = true;
    wait_for_enter("Place the car level and completely still.");
    printf("Sampling IMU gyro bias for 2 seconds...\n");
    bool success = imu_sensor_calibrate_bias(2000);
    printf(success
               ? "OK: IMU yaw-rate bias calibrated for this boot\n"
               : "ERR: IMU calibration failed; keep the car still and level, then retry\n");
    calibration_in_progress = false;
}

void powertrain_cal_esc_arm(void)
{
    if (!calibration_allowed(true)) return;
    system_state = SYSTEM_ESC_CAL_ARMED;
    esc_cal_last_command_us = esp_timer_get_time();
    esc_output_set_safe();
    printf("OK: ESC calibration armed. Keep ESC battery disconnected, then send 'cal esc max'.\n");
}

void powertrain_cal_esc_max(void)
{
    if (system_state != SYSTEM_ESC_CAL_ARMED &&
        system_state != SYSTEM_ESC_CAL_MAX &&
        system_state != SYSTEM_ESC_CAL_MIN) {
        printf("ERR: send 'cal esc arm' before 'cal esc max'\n");
        return;
    }
    if (!throttle_is_neutral()) {
        printf("ERR: receiver throttle is not neutral\n");
        return;
    }
    if (!shutdown_channel_is_safe()) {
        printf("ERR: receiver arm switch must be in STOP for calibration\n");
        return;
    }
    system_state = SYSTEM_ESC_CAL_MAX;
    esc_cal_last_command_us = esp_timer_get_time();
    esc_output_set_all(esc_limits.throttle_max_us, esc_limits.reverse_low_us);
    printf("OK: maximum throttle active. Power ESCs, wait for endpoint beeps, then send 'cal esc min'.\n");
}

void powertrain_cal_esc_min(void)
{
    if (system_state != SYSTEM_ESC_CAL_MAX) {
        printf("ERR: send 'cal esc max' before 'cal esc min'\n");
        return;
    }
    if (!throttle_is_neutral()) {
        printf("ERR: receiver throttle is not neutral\n");
        return;
    }
    if (!shutdown_channel_is_safe()) {
        printf("ERR: receiver arm switch must be in STOP for calibration\n");
        return;
    }
    system_state = SYSTEM_ESC_CAL_MIN;
    esc_cal_last_command_us = esp_timer_get_time();
    esc_output_set_all(esc_limits.throttle_min_us, esc_limits.reverse_low_us);
    printf("OK: minimum throttle active. Wait for ready beeps, then send 'cal cancel'.\n");
}

void powertrain_cal_manual(void)
{
    if (!calibration_allowed(false)) return;
    system_state = SYSTEM_ESC_CAL_MANUAL;
    esc_cal_last_command_us = esp_timer_get_time();
    printf("OK: direct throttle relay active for 30 seconds; all reverse outputs remain low.\n");
}

void powertrain_cal_cancel(void)
{
    system_state = SYSTEM_DISARMED;
    calibration_in_progress = false;
    arm_cycle_ready = false;
    reset_drive_state();
    esc_output_set_safe();
    printf("OK: calibration canceled; safe outputs restored\n");
}

void powertrain_arm_from_cli(void)
{
    if (system_state != SYSTEM_DISARMED) {
        printf("ERR: controller must be DISARMED\n");
        return;
    }
    if (!arm_cycle_ready) {
        printf("ERR: cycle the receiver shutdown switch through STOP before arming\n");
        return;
    }
    if (drive_arm_allowed()) {
        arm_drive();
    }
}

void powertrain_disarm(void)
{
    disarm_to_safe("OK: drive disarmed; safe outputs restored", true);
    if (system_state == SYSTEM_DISARMED) {
        esc_output_set_safe();
    }
}

static void save_drive_config_or_report(const char *success_message)
{
    drive_config.loaded_from_nvs = true;
    esp_err_t err = config_store_save_drive(&drive_config);
    if (err == ESP_OK) {
        printf("%s\n", success_message);
    } else {
        printf("ERR: failed to save drive configuration: %s\n", esp_err_to_name(err));
    }
}

void powertrain_set_reverse_limit(uint8_t percent)
{
    if (system_state != SYSTEM_DISARMED || percent > 100) {
        printf("ERR: reverse limit must be 0-100 and changed while DISARMED\n");
        return;
    }
    drive_config.reverse_limit_percent = percent;
    char message[64];
    snprintf(message, sizeof(message), "OK: reverse limit set to %u%%", percent);
    save_drive_config_or_report(message);
}

void powertrain_set_failsafe(uint16_t pulse_us, uint16_t window_us)
{
    if (system_state != SYSTEM_DISARMED || pulse_us < 800 || pulse_us > 2200 ||
        window_us < 1 || window_us > 100) {
        printf("ERR: failsafe pulse must be 800-2200 us, window 1-100 us, while DISARMED\n");
        return;
    }
    drive_config.receiver_failsafe_us = pulse_us;
    drive_config.receiver_failsafe_window_us = window_us;
    failsafe_candidate_since_us = 0;
    char message[96];
    snprintf(message, sizeof(message), "OK: throttle failsafe set to %u +/- %u us",
             pulse_us, window_us);
    save_drive_config_or_report(message);
}

void powertrain_set_motor_poles(uint8_t poles)
{
    if (system_state != SYSTEM_DISARMED || poles < 2 || poles > 60 || (poles % 2) != 0) {
        printf("ERR: motor pole count must be even, 2-60, and changed while DISARMED\n");
        return;
    }
    drive_config.motor_poles = poles;
    if (rpm_initialized) rpm_sensor_set_motor_poles(poles);
    char message[80];
    snprintf(message, sizeof(message),
             "OK: motor pole count set to %u (%u pole pairs)", poles, poles / 2);
    save_drive_config_or_report(message);
}

void powertrain_set_tv_enabled(bool enabled)
{
    if (system_state != SYSTEM_DISARMED) {
        printf("ERR: torque vectoring can only be configured while DISARMED\n");
        return;
    }
    if (enabled && (!steering_cal.loaded_from_nvs || !tv_mode_cal.loaded_from_nvs)) {
        printf("ERR: run 'cal steering' and 'cal tv' before enabling torque vectoring\n");
        return;
    }
    if (enabled) {
        imu_snapshot_t imu = {0};
        if (imu_initialized) {
            imu_sensor_get_snapshot(&imu);
        }
        if (!imu.valid || !imu.bias_calibrated) {
            printf("ERR: run 'cal imu' successfully before enabling torque vectoring\n");
            return;
        }
        for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
            if (!rpm_seen[wheel]) {
                printf("ERR: RPM input %s has not produced a valid pulse this boot; "
                       "use 'monitor rpm' and spin each wheel\n",
                       wheel_names[wheel]);
                return;
            }
        }
    }
    drive_config.torque_vectoring_enabled = enabled;
    save_drive_config_or_report(enabled
                                    ? "OK: torque vectoring enabled; CH5 still selects its mode"
                                    : "OK: torque vectoring disabled");
}

void powertrain_set_tv_authority(uint8_t percent)
{
    if (system_state != SYSTEM_DISARMED || percent > 25) {
        printf("ERR: vectoring authority must be 0-25 percent while DISARMED\n");
        return;
    }
    drive_config.tv_authority_percent = percent;
    char message[80];
    snprintf(message, sizeof(message), "OK: vectoring authority set to +/- %u%%", percent);
    save_drive_config_or_report(message);
}

void powertrain_set_tv_gains(float yaw_gain_dps,
                             float turn_rpm_gain,
                             float yaw_kp,
                             float yaw_ki,
                             float rpm_kp)
{
    if (system_state != SYSTEM_DISARMED ||
        yaw_gain_dps < 0.0f || yaw_gain_dps > 500.0f ||
        turn_rpm_gain < 0.0f || turn_rpm_gain > 1.0f ||
        yaw_kp < 0.0f || yaw_kp > 0.01f ||
        yaw_ki < 0.0f || yaw_ki > 0.01f ||
        rpm_kp < 0.0f || rpm_kp > 5.0f) {
        printf("ERR: invalid TV gains or controller is not DISARMED\n");
        return;
    }
    drive_config.tv_turn_yaw_gain_dps = yaw_gain_dps;
    drive_config.tv_turn_rpm_gain = turn_rpm_gain;
    drive_config.tv_yaw_kp = yaw_kp;
    drive_config.tv_yaw_ki = yaw_ki;
    drive_config.tv_rpm_kp = rpm_kp;
    save_drive_config_or_report("OK: torque-vectoring gains saved");
}

void powertrain_set_imu_yaw_sign(int8_t sign)
{
    if (system_state != SYSTEM_DISARMED || (sign != -1 && sign != 1)) {
        printf("ERR: IMU yaw sign must be -1 or 1 while DISARMED\n");
        return;
    }
    drive_config.imu_yaw_sign = sign;
    if (imu_initialized) imu_sensor_set_yaw_sign(sign);
    save_drive_config_or_report("OK: IMU yaw sign saved");
}

static void print_channel(const char *name, rc_channel_id_t channel)
{
    rc_channel_sample_t sample;
    if (rc_input_get(channel, &sample)) {
        printf("  %-10s valid, %u us\n", name, sample.pulse_us);
    } else {
        printf("  %-10s missing\n", name);
    }
}

void powertrain_print_status(void)
{
    wheel_output_command_t outputs;
    rpm_snapshot_t rpm = {0};
    imu_snapshot_t imu = {0};
    bool arm_run = false;
    bool arm_valid = arm_channel_state(&arm_run);
    esc_output_get(&outputs);
    if (rpm_initialized) rpm_sensor_get_snapshot(&rpm);
    if (imu_initialized) imu_sensor_get_snapshot(&imu);

    printf("\nRC car powertrain status\n");
    printf("  state: %s, rearm cycle: %s\n",
           state_name(system_state), arm_cycle_ready ? "ready" : "required");
    print_channel("throttle", RC_CHANNEL_THROTTLE);
    print_channel("steering", RC_CHANNEL_STEERING);
    print_channel("arm/stop", RC_CHANNEL_ARM);
    print_channel("TV CH5", RC_CHANNEL_TV_MODE);
    printf("  arm request: %s\n", arm_valid ? (arm_run ? "RUN" : "STOP") : "missing");
    printf("  throttle cal: full=%u neutral=%u reverse=%u deadband=%u source=%s\n",
           throttle_cal.full_throttle_us, throttle_cal.neutral_us,
           throttle_cal.full_reverse_us, throttle_cal.deadband_us,
           throttle_cal.loaded_from_nvs ? "nvs" : "defaults/unarmed");
    printf("  steering cal: left=%u center=%u right=%u deadband=%u source=%s\n",
           steering_cal.left_us, steering_cal.center_us, steering_cal.right_us,
           steering_cal.deadband_us,
           steering_cal.loaded_from_nvs ? "nvs" : "defaults");
    printf("  arm cal: run=%u stop=%u source=%s\n",
           arm_cal.run_us, arm_cal.stop_us,
           arm_cal.loaded_from_nvs ? "nvs" : "defaults/unarmed");
    printf("  CH5 cal: off=%u straight=%u full=%u source=%s\n",
           tv_mode_cal.off_us, tv_mode_cal.straight_us, tv_mode_cal.full_us,
           tv_mode_cal.loaded_from_nvs ? "nvs" : "defaults");
    printf("  drive: reverse_limit=%u%%, failsafe=%u +/- %u us, motor=%u poles\n",
           drive_config.reverse_limit_percent, drive_config.receiver_failsafe_us,
           drive_config.receiver_failsafe_window_us, drive_config.motor_poles);
    printf("  TV: configured=%s, CH5=%s, active=%s, authority=+/- %u%%, reason=%s\n",
           drive_config.torque_vectoring_enabled ? "enabled" : "disabled",
           tv_mode_name(requested_tv_mode()),
           vectoring_output.active ? tv_mode_name(vectoring_output.active_mode) : "no",
           drive_config.tv_authority_percent,
           vectoring_output.inactive_reason ? vectoring_output.inactive_reason : "not evaluated");
    printf("  IMU: %s, bias=%s, yaw=%+.2f dps\n",
           imu.valid ? "valid" : "unavailable",
           imu.bias_calibrated ? "calibrated" : "not calibrated",
           imu.yaw_rate_dps);
    printf("  RPM: FL=%.0f%s FR=%.0f%s RL=%.0f%s RR=%.0f%s\n",
           rpm.rpm[0], rpm.valid[0] ? "" : "?",
           rpm.rpm[1], rpm.valid[1] ? "" : "?",
           rpm.rpm[2], rpm.valid[2] ? "" : "?",
           rpm.rpm[3], rpm.valid[3] ? "" : "?");
    printf("  RPM seen this boot: FL=%s FR=%s RL=%s RR=%s\n",
           rpm_seen[0] ? "yes" : "no", rpm_seen[1] ? "yes" : "no",
           rpm_seen[2] ? "yes" : "no", rpm_seen[3] ? "yes" : "no");
    printf("  outputs:");
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        printf(" %s=%u/%u", wheel_names[wheel],
               outputs.throttle_us[wheel], outputs.reverse_us[wheel]);
    }
    printf("\n\n");
}

static void monitor_channel(rc_channel_id_t channel, const char *name)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    printf("Monitoring %s PWM for 30 seconds.\n", name);
    while (esp_timer_get_time() < end_at_us) {
        rc_channel_sample_t sample;
        if (rc_input_get(channel, &sample)) {
            printf("%s: %u us\n", name, sample.pulse_us);
        } else {
            printf("%s: missing\n", name);
        }
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_throttle(void)
{
    monitor_channel(RC_CHANNEL_THROTTLE, "throttle");
}

void powertrain_monitor_steering(void)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    while (esp_timer_get_time() < end_at_us) {
        rc_channel_sample_t sample;
        if (rc_input_get(RC_CHANNEL_STEERING, &sample)) {
            printf("steering: %u us, normalized=%+.3f\n",
                   sample.pulse_us, steering_normalized());
        } else {
            printf("steering: missing\n");
        }
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_arm(void)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    printf("Monitoring receiver arm/stop PWM for 30 seconds.\n");
    while (esp_timer_get_time() < end_at_us) {
        rc_channel_sample_t sample;
        if (rc_input_get(RC_CHANNEL_ARM, &sample)) {
            printf("arm: %u us, request=%s, rearm_cycle=%s\n",
                   sample.pulse_us,
                   arm_requests_run(sample.pulse_us) ? "RUN" : "STOP",
                   arm_cycle_ready ? "ready" : "required");
        } else {
            printf("arm: missing\n");
        }
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_tv_mode(void)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    while (esp_timer_get_time() < end_at_us) {
        rc_channel_sample_t sample;
        if (rc_input_get(RC_CHANNEL_TV_MODE, &sample)) {
            printf("TV CH5: %u us, mode=%s\n",
                   sample.pulse_us, tv_mode_name(requested_tv_mode()));
        } else {
            printf("TV CH5: missing, mode=OFF\n");
        }
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_rpm(void)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    while (esp_timer_get_time() < end_at_us) {
        rpm_snapshot_t rpm = {0};
        if (rpm_initialized) rpm_sensor_get_snapshot(&rpm);
        printf("RPM: FL=%.0f%s FR=%.0f%s RL=%.0f%s RR=%.0f%s\n",
               rpm.rpm[0], rpm.valid[0] ? "" : " missing",
               rpm.rpm[1], rpm.valid[1] ? "" : " missing",
               rpm.rpm[2], rpm.valid[2] ? "" : " missing",
               rpm.rpm[3], rpm.valid[3] ? "" : " missing");
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_imu(void)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    while (esp_timer_get_time() < end_at_us) {
        imu_snapshot_t imu = {0};
        if (imu_initialized) imu_sensor_get_snapshot(&imu);
        printf("IMU: %s yaw=%+.2f dps gyro=[%+.2f %+.2f %+.2f] accel=[%+.2f %+.2f %+.2f]\n",
               imu.valid ? "valid" : "missing",
               imu.yaw_rate_dps,
               imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2],
               imu.accel_mps2[0], imu.accel_mps2[1], imu.accel_mps2[2]);
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_vectoring(void)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    while (esp_timer_get_time() < end_at_us) {
        printf("TV: CH5=%s active=%s target_yaw=%+.2f error=%+.2f rpm_error=%+.3f "
               "correction=[%+.3f %+.3f %+.3f %+.3f] reason=%s\n",
               tv_mode_name(requested_tv_mode()),
               vectoring_output.active ? "yes" : "no",
               vectoring_output.target_yaw_rate_dps,
               vectoring_output.yaw_error_dps,
               vectoring_output.side_rpm_error,
               vectoring_output.wheel_correction[0],
               vectoring_output.wheel_correction[1],
               vectoring_output.wheel_correction[2],
               vectoring_output.wheel_correction[3],
               vectoring_output.inactive_reason ? vectoring_output.inactive_reason : "not evaluated");
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_print_help(void)
{
    printf("\nCommands:\n");
    printf("  status | arm | disarm | help\n");
    printf("  cal receiver | cal steering | cal arm | cal tv | cal imu\n");
    printf("  cal esc arm | cal esc max | cal esc min | cal manual | cal cancel\n");
    printf("  monitor throttle | steering | arm | tv | rpm | imu | vector\n");
    printf("  config reverse <0-100>\n");
    printf("  config failsafe <pulse_us> <window_us>\n");
    printf("  config rpm poles <even 2-60>\n");
    printf("  config tv authority <0-25>\n");
    printf("  config tv gains <yaw_gain_dps> <turn_rpm_gain> <yaw_kp> <yaw_ki> <rpm_kp>\n");
    printf("  config imu yaw-sign <-1|1>\n");
    printf("  tv enable | tv disable\n\n");
}

static void sensor_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t elapsed_ms = 0;
    while (true) {
        if (imu_initialized) {
            imu_sensor_update();
        }
        elapsed_ms += SENSOR_UPDATE_INTERVAL_MS;
        if (rpm_initialized && elapsed_ms >= RPM_UPDATE_INTERVAL_MS) {
            rpm_sensor_update();
            rpm_snapshot_t rpm;
            rpm_sensor_get_snapshot(&rpm);
            for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
                if (rpm.valid[wheel]) {
                    rpm_seen[wheel] = true;
                }
            }
            elapsed_ms = 0;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_UPDATE_INTERVAL_MS));
    }
}

static void powertrain_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        handle_receiver_arm_control();

        if (system_state == SYSTEM_DRIVE_ARMED) {
            update_drive_outputs();
        } else if (system_state == SYSTEM_ESC_CAL_MANUAL) {
            rc_channel_sample_t throttle;
            if (throttle_matches_failsafe() || !throttle_sample(&throttle)) {
                powertrain_cal_cancel();
                printf("\nWARN: manual ESC relay stopped due to receiver failsafe\n> ");
            } else {
                esc_output_set_all(throttle.pulse_us, esc_limits.reverse_low_us);
            }
        }

        if (system_state == SYSTEM_ESC_CAL_ARMED ||
            system_state == SYSTEM_ESC_CAL_MAX ||
            system_state == SYSTEM_ESC_CAL_MIN ||
            system_state == SYSTEM_ESC_CAL_MANUAL) {
            if (esp_timer_get_time() - esc_cal_last_command_us > ESC_CAL_TIMEOUT_US) {
                powertrain_cal_cancel();
                printf("\nWARN: ESC calibration timed out\n> ");
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DRIVE_UPDATE_INTERVAL_MS));
    }
}

esp_err_t powertrain_controller_init(void)
{
    esp_err_t err = config_store_init();
    if (err != ESP_OK) return err;

    config_store_load_throttle(&throttle_cal);
    config_store_load_steering(&steering_cal);
    config_store_load_arm(&arm_cal);
    config_store_load_tv_mode(&tv_mode_cal);
    config_store_load_drive(&drive_config);

    err = esc_output_init(&esc_limits);
    if (err != ESP_OK) return err;
    err = rc_input_init();
    if (err != ESP_OK) return err;

    err = rpm_sensor_init(drive_config.motor_poles);
    if (err == ESP_OK) {
        rpm_initialized = true;
    } else {
        ESP_LOGW(TAG, "RPM capture unavailable: %s", esp_err_to_name(err));
    }

    err = imu_sensor_init(drive_config.imu_yaw_sign);
    if (err == ESP_OK) {
        imu_initialized = true;
    } else {
        ESP_LOGW(TAG, "ISM330DHCX unavailable: %s", esp_err_to_name(err));
    }

    reset_drive_state();
    esc_output_set_safe();
    return ESP_OK;
}

void powertrain_controller_start(void)
{
    xTaskCreate(sensor_task, "sensors", 4096, NULL, 6, NULL);
    xTaskCreate(powertrain_task, "powertrain", 6144, NULL, 5, NULL);
}
