#include "powertrain_controller.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "config_store.h"
#include "default_config.h"
#include "driver/gpio.h"
#include "esc_output.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "imu_sensor.h"
#include "pin_config.h"
#include "powertrain_types.h"
#include "rc_input.h"
#include "rpm_sensor.h"
#include "servo_output.h"
#include "steering_curve.h"
#include "steering_input_filter.h"
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
#define ARM_UNRECOGNIZED_CONFIRM_US 60000
#define ARM_RUN_STABLE_US 250000
#define ESC_CAL_TIMEOUT_US 30000000
#define MONITOR_DURATION_US 30000000
#define MONITOR_INTERVAL_MS 250
#define TV_MODE_POSITION_WINDOW_US 125
#define SERVO_NOMINAL_US_PER_90_DEG 1000
#define IMU_STARTUP_CALIBRATION_MS 5000U

_Static_assert(pdMS_TO_TICKS(SENSOR_UPDATE_INTERVAL_MS) > 0,
               "SENSOR_UPDATE_INTERVAL_MS must convert to at least one FreeRTOS tick");

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

static system_state_t system_state = SYSTEM_DISARMED;
static bool calibration_in_progress;
static bool rpm_initialized;
static bool imu_initialized;
static bool sensor_task_started;
static int64_t esc_cal_last_command_us;
static int64_t failsafe_candidate_since_us;
static uint16_t current_base_throttle_us = ESC_THROTTLE_MIN_US;
static drive_direction_t active_direction = DRIVE_DIRECTION_FORWARD;
static drive_direction_t pending_direction = DRIVE_DIRECTION_FORWARD;
static int64_t direction_hold_until_us;
static bool arm_cycle_ready;
static int64_t arm_run_candidate_since_us;
static int64_t arm_unrecognized_since_us;
static uint16_t arm_unrecognized_pulse_us;
static bool rpm_seen[POWERTRAIN_WHEEL_COUNT];
static uint32_t sensor_overrun_count;
static uint32_t powertrain_overrun_count;
static torque_vectoring_state_t vectoring_state;
static torque_vectoring_output_t vectoring_output;
static SemaphoreHandle_t controller_mutex;
static TaskHandle_t sensor_task_handle;
static TaskHandle_t powertrain_task_handle;
static portMUX_TYPE telemetry_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t sensor_max_execution_us;
static uint32_t powertrain_max_execution_us;
static UBaseType_t sensor_stack_min_free_bytes;
static UBaseType_t powertrain_stack_min_free_bytes;
static int32_t steering_filtered_q16;
static bool steering_filter_initialized;
static steering_input_filter_t steering_input_filter;

static void controller_lock(void)
{
    configASSERT(controller_mutex != NULL);
    configASSERT(xSemaphoreTakeRecursive(controller_mutex, portMAX_DELAY) == pdTRUE);
}

static void controller_unlock(void)
{
    configASSERT(xSemaphoreGiveRecursive(controller_mutex) == pdTRUE);
}

static uint16_t clamp_steering_pulse(int32_t pulse_us)
{
    uint16_t minimum_us = steering_cal.left_us < steering_cal.right_us
                              ? steering_cal.left_us
                              : steering_cal.right_us;
    uint16_t maximum_us = steering_cal.left_us > steering_cal.right_us
                              ? steering_cal.left_us
                              : steering_cal.right_us;
    if (pulse_us < minimum_us) return minimum_us;
    if (pulse_us > maximum_us) return maximum_us;
    return (uint16_t)pulse_us;
}

static int32_t steering_trim_us_from_tenths(int16_t trim_tenths_deg)
{
    return (int32_t)lroundf((float)trim_tenths_deg *
                            ((float)SERVO_NOMINAL_US_PER_90_DEG / 900.0f));
}

static int32_t steering_trim_us(void)
{
    return steering_trim_us_from_tenths(drive_config.steering_trim_tenths_deg);
}

static bool steering_trim_fits_endpoints(int16_t trim_tenths_deg)
{
    int32_t center_us = (int32_t)steering_cal.center_us +
                        steering_trim_us_from_tenths(trim_tenths_deg);
    int32_t minimum_us = steering_cal.left_us < steering_cal.right_us
                             ? steering_cal.left_us
                             : steering_cal.right_us;
    int32_t maximum_us = steering_cal.left_us > steering_cal.right_us
                             ? steering_cal.left_us
                             : steering_cal.right_us;
    return center_us > minimum_us && center_us < maximum_us;
}

static uint16_t trimmed_steering_center_us(void)
{
    return clamp_steering_pulse((int32_t)steering_cal.center_us + steering_trim_us());
}

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

static const char *steering_input_filter_status_name(
    steering_input_filter_status_t status)
{
    switch (status) {
    case STEERING_INPUT_FILTER_WARMUP: return "warmup";
    case STEERING_INPUT_FILTER_ACTIVE: return "active";
    default: return "unknown";
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

static void wait_for_next_period(TickType_t *last_wake,
                                 TickType_t period_ticks,
                                 uint32_t *overrun_count)
{
    if (xTaskDelayUntil(last_wake, period_ticks) == pdFALSE) {
        portENTER_CRITICAL(&telemetry_lock);
        (*overrun_count)++;
        portEXIT_CRITICAL(&telemetry_lock);
        *last_wake = xTaskGetTickCount();
        vTaskDelay(1);
    }
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
    if (!drive_config.receiver_failsafe_enabled) {
        failsafe_candidate_since_us = 0;
        return false;
    }

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

static void update_steering_servo(void)
{
    uint16_t target_us = steering_cal.center_us;
    rc_channel_sample_t sample;
    bool input_valid = false;

    if (!throttle_matches_failsafe() &&
        rc_input_get(RC_CHANNEL_STEERING, &sample)) {
        uint16_t filtered_input_us = steering_cal.center_us;
        bool filter_valid = steering_input_filter_update(&steering_input_filter,
                                                         sample.pulse_us,
                                                         steering_cal.deadband_us,
                                                         sample.updated_at_us,
                                                         &filtered_input_us);
        uint16_t minimum_us = steering_cal.left_us < steering_cal.right_us
                                  ? steering_cal.left_us
                                  : steering_cal.right_us;
        uint16_t maximum_us = steering_cal.left_us > steering_cal.right_us
                                  ? steering_cal.left_us
                                  : steering_cal.right_us;
        int32_t center_delta = (int32_t)filtered_input_us - steering_cal.center_us;

        if (!filter_valid || abs(center_delta) <= steering_cal.deadband_us) {
            target_us = steering_cal.center_us;
        } else if (filtered_input_us < minimum_us) {
            target_us = minimum_us;
        } else if (filtered_input_us > maximum_us) {
            target_us = maximum_us;
        } else {
            target_us = filtered_input_us;
        }
        input_valid = true;
    } else {
        steering_input_filter_reset(&steering_input_filter);
    }

    target_us = clamp_steering_pulse((int32_t)target_us + steering_trim_us());
    int32_t target_q16 = (int32_t)target_us << 16;
    if (!input_valid || drive_config.steering_smoothing_ms == 0 ||
        !steering_filter_initialized) {
        steering_filtered_q16 = target_q16;
        steering_filter_initialized = true;
    } else {
        int32_t difference_q16 = target_q16 - steering_filtered_q16;
        int32_t denominator_ms = drive_config.steering_smoothing_ms +
                                 DRIVE_UPDATE_INTERVAL_MS;
        steering_filtered_q16 +=
            (int32_t)(((int64_t)difference_q16 * DRIVE_UPDATE_INTERVAL_MS) /
                      denominator_ms);
    }

    uint16_t output_us = (uint16_t)((steering_filtered_q16 + (1 << 15)) >> 16);
    servo_output_set_pulse(clamp_steering_pulse(output_us));
}

typedef enum {
    ARM_POSITION_UNKNOWN = 0,
    ARM_POSITION_RUN,
    ARM_POSITION_STOP_1,
    ARM_POSITION_STOP_2,
} arm_position_t;

static arm_position_t arm_position(uint16_t pulse_us)
{
    int run_distance = abs((int32_t)pulse_us - (int32_t)arm_cal.run_us);
    int stop_1_distance = abs((int32_t)pulse_us - (int32_t)arm_cal.stop_1_us);
    int stop_2_distance = abs((int32_t)pulse_us - (int32_t)arm_cal.stop_2_us);

    if (stop_1_distance <= DEFAULT_RC_ARM_POSITION_WINDOW_US) return ARM_POSITION_STOP_1;
    if (stop_2_distance <= DEFAULT_RC_ARM_POSITION_WINDOW_US) return ARM_POSITION_STOP_2;
    if (run_distance <= DEFAULT_RC_ARM_POSITION_WINDOW_US) return ARM_POSITION_RUN;
    return ARM_POSITION_UNKNOWN;
}

static const char *arm_position_name(uint16_t pulse_us)
{
    switch (arm_position(pulse_us)) {
    case ARM_POSITION_RUN:
        return "RUN";
    case ARM_POSITION_STOP_1:
        return "STOP 1";
    case ARM_POSITION_STOP_2:
        return "STOP 2";
    default:
        return "UNRECOGNIZED";
    }
}

static bool arm_requests_run(uint16_t pulse_us)
{
    return arm_position(pulse_us) == ARM_POSITION_RUN;
}

static bool arm_channel_state(bool *run)
{
    rc_channel_sample_t sample;
    if (!rc_input_get(RC_CHANNEL_ARM, &sample)) {
        return false;
    }
    arm_position_t position = arm_position(sample.pulse_us);
    if (position == ARM_POSITION_UNKNOWN) {
        return false;
    }
    *run = position == ARM_POSITION_RUN;
    return true;
}

static uint32_t rc_sample_age_ms(const rc_channel_sample_t *sample, int64_t now_us)
{
    if (sample->updated_at_us <= 0 || now_us <= sample->updated_at_us) {
        return 0;
    }

    int64_t age_ms = (now_us - sample->updated_at_us) / 1000;
    return age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
}

static bool shutdown_channel_is_safe(void)
{
    if (!arm_cal.loaded_from_nvs) {
        return false;
    }

    bool arm_run = false;
    return arm_channel_state(&arm_run) && !arm_run;
}

static steering_curve_sample_t steering_geometry_sample(void)
{
    steering_curve_sample_t geometry = {0};
    if (!steering_cal.loaded_from_nvs) {
        return geometry;
    }

    uint16_t applied_us = servo_output_get_pulse();
    uint16_t center_us = trimmed_steering_center_us();
    int32_t delta = (int32_t)applied_us - center_us;
    if (abs(delta) <= 1) {
        steering_curve_sample(0.0f, &geometry);
        return geometry;
    }

    bool right = pulse_is_on_endpoint_side(applied_us,
                                           steering_cal.right_us,
                                           center_us);
    float servo_command_deg = (float)abs(delta) *
                              (90.0f / SERVO_NOMINAL_US_PER_90_DEG);
    if (!right) {
        servo_command_deg = -servo_command_deg;
    }
    steering_curve_sample(servo_command_deg, &geometry);
    return geometry;
}

static float steering_normalized(void)
{
    steering_curve_sample_t geometry = steering_geometry_sample();
    return geometry.valid ? geometry.normalized_average : 0.0f;
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
    if (off_distance <= TV_MODE_POSITION_WINDOW_US &&
        off_distance <= straight_distance && off_distance <= full_distance) {
        return TV_MODE_OFF;
    }
    if (straight_distance <= TV_MODE_POSITION_WINDOW_US &&
        straight_distance <= full_distance) {
        return TV_MODE_STRAIGHT;
    }
    return full_distance <= TV_MODE_POSITION_WINDOW_US ? TV_MODE_FULL : TV_MODE_OFF;
}

static void reset_drive_state(void)
{
    current_base_throttle_us = esc_limits.throttle_min_us;
    active_direction = DRIVE_DIRECTION_FORWARD;
    pending_direction = DRIVE_DIRECTION_FORWARD;
    direction_hold_until_us = 0;
    failsafe_candidate_since_us = 0;
    arm_run_candidate_since_us = 0;
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
        printf("ERR: receiver arm channel is missing or outside its calibrated positions\n");
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
        pending_direction = active_direction;
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
    rc_channel_sample_t arm_sample = {0};
    bool arm_signal_valid = rc_input_get(RC_CHANNEL_ARM, &arm_sample);
    arm_position_t position = arm_signal_valid
                                  ? arm_position(arm_sample.pulse_us)
                                  : ARM_POSITION_UNKNOWN;
    bool arm_unrecognized = arm_signal_valid && position == ARM_POSITION_UNKNOWN;
    bool arm_unrecognized_confirmed = false;
    int64_t now_us = esp_timer_get_time();

    if (arm_unrecognized) {
        if (arm_unrecognized_since_us == 0) {
            arm_unrecognized_since_us = now_us;
        }
        arm_unrecognized_pulse_us = arm_sample.pulse_us;
        arm_unrecognized_confirmed =
            now_us - arm_unrecognized_since_us >= ARM_UNRECOGNIZED_CONFIRM_US;
    } else {
        arm_unrecognized_since_us = 0;
    }

    bool arm_run = position == ARM_POSITION_RUN;
    bool healthy_throttle = rc_input_is_valid(RC_CHANNEL_THROTTLE) &&
                            !throttle_matches_failsafe();
    bool esc_calibration_active = system_state == SYSTEM_ESC_CAL_ARMED ||
                                  system_state == SYSTEM_ESC_CAL_MAX ||
                                  system_state == SYSTEM_ESC_CAL_MIN ||
                                  system_state == SYSTEM_ESC_CAL_MANUAL;

    if (esc_calibration_active && arm_cal.loaded_from_nvs &&
        (!arm_signal_valid || arm_run || arm_unrecognized)) {
        powertrain_cal_cancel();
        printf("\nWARN: ESC calibration stopped because the receiver shutdown "
               "channel is not in STOP\n> ");
        fflush(stdout);
        arm_run_candidate_since_us = 0;
        return;
    }

    if (!arm_signal_valid) {
        arm_run_candidate_since_us = 0;
        if (system_state == SYSTEM_DRIVE_ARMED) {
            rc_channel_sample_t throttle_sample = {0};
            rc_channel_sample_t steering_sample = {0};
            rc_channel_sample_t tv_sample = {0};
            rc_input_get(RC_CHANNEL_THROTTLE, &throttle_sample);
            rc_input_get(RC_CHANNEL_STEERING, &steering_sample);
            rc_input_get(RC_CHANNEL_TV_MODE, &tv_sample);

            char warning[256];
            snprintf(warning, sizeof(warning),
                     "WARN: receiver arm signal lost; CH4 last=%u us age=%lu ms "
                     "level=%d, CH2 age=%lu ms, CH1 age=%lu ms, CH5 age=%lu ms; "
                     "drive disarmed",
                     arm_sample.pulse_us,
                     (unsigned long)rc_sample_age_ms(&arm_sample, now_us),
                     gpio_get_level(PIN_RC_ARM_INPUT),
                     (unsigned long)rc_sample_age_ms(&throttle_sample, now_us),
                     (unsigned long)rc_sample_age_ms(&steering_sample, now_us),
                     (unsigned long)rc_sample_age_ms(&tv_sample, now_us));
            disarm_to_safe(warning, true);
        }
        return;
    }

    if (arm_unrecognized) {
        arm_run_candidate_since_us = 0;
        if (arm_unrecognized_confirmed && system_state == SYSTEM_DRIVE_ARMED) {
            char warning[192];
            snprintf(warning, sizeof(warning),
                     "WARN: receiver arm channel stayed unrecognized at %u us "
                     "(RUN=%u, STOP1=%u, STOP2=%u); drive disarmed",
                     arm_unrecognized_pulse_us, arm_cal.run_us,
                     arm_cal.stop_1_us, arm_cal.stop_2_us);
            disarm_to_safe(warning, true);
        }
        return;
    }

    if (!arm_run) {
        arm_run_candidate_since_us = 0;
        if (system_state == SYSTEM_DRIVE_ARMED) {
            disarm_to_safe("OK: receiver arm switch moved to STOP; drive disarmed", false);
        }
        if (healthy_throttle && !calibration_in_progress) {
            arm_cycle_ready = true;
        }
    } else if (arm_cycle_ready &&
               system_state == SYSTEM_DISARMED && !calibration_in_progress) {
        if (!healthy_throttle || !throttle_is_neutral()) {
            arm_run_candidate_since_us = 0;
        } else if (arm_run_candidate_since_us == 0) {
            arm_run_candidate_since_us = now_us;
        } else if (now_us - arm_run_candidate_since_us >= ARM_RUN_STABLE_US) {
            if (drive_arm_allowed()) {
                arm_drive();
            } else {
                arm_run_candidate_since_us = now_us;
            }
        }
    }
}

static bool calibration_allowed(bool require_neutral, bool require_saved_shutdown)
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

    if (require_saved_shutdown && !arm_cal.loaded_from_nvs) {
        printf("ERR: run 'cal arm' before ESC calibration or manual relay\n");
        return false;
    }
    if (arm_cal.loaded_from_nvs && !shutdown_channel_is_safe()) {
        printf("ERR: receiver arm switch must be in STOP for calibration\n");
        return false;
    }
    return true;
}

static bool begin_interactive_calibration(bool require_neutral,
                                          bool require_saved_shutdown)
{
    controller_lock();
    bool allowed = calibration_allowed(require_neutral, require_saved_shutdown);
    if (allowed) {
        calibration_in_progress = true;
    }
    controller_unlock();
    return allowed;
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
    if (!begin_interactive_calibration(false, false)) return;
    printf("Receiver throttle calibration started.\n");
    uint16_t neutral = capture_channel(RC_CHANNEL_THROTTLE, "Hold neutral throttle.");
    uint16_t full = neutral ? capture_channel(RC_CHANNEL_THROTTLE, "Hold full throttle.") : 0;
    uint16_t reverse = full ? capture_channel(RC_CHANNEL_THROTTLE, "Hold full reverse.") : 0;

    controller_lock();
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
    controller_unlock();
}

void powertrain_calibrate_steering(void)
{
    if (!begin_interactive_calibration(true, false)) return;
    printf("Steering calibration started. The servo continues following valid CH1 input.\n");
    uint16_t center = capture_channel(RC_CHANNEL_STEERING, "Hold steering centered.");
    uint16_t left = center ? capture_channel(RC_CHANNEL_STEERING, "Hold full left steering.") : 0;
    uint16_t right = left ? capture_channel(RC_CHANNEL_STEERING, "Hold full right steering.") : 0;

    controller_lock();
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
        if (!steering_trim_fits_endpoints(drive_config.steering_trim_tenths_deg)) {
            drive_config.steering_trim_tenths_deg = 0;
            esp_err_t trim_err = config_store_save_drive(&drive_config);
            printf(trim_err == ESP_OK
                       ? "WARN: steering trim reset because it did not fit the new endpoints\n"
                       : "WARN: steering trim reset in RAM but failed to save: %s\n",
                   trim_err == ESP_OK ? "" : esp_err_to_name(trim_err));
        }
        steering_filtered_q16 = (int32_t)trimmed_steering_center_us() << 16;
        steering_input_filter_reset(&steering_input_filter);
        servo_output_set_pulse(trimmed_steering_center_us());
    } else if (center && left && right) {
        printf("ERR: steering endpoints must be at least 150 us from opposite sides of center\n");
    }
    calibration_in_progress = false;
    controller_unlock();
}

void powertrain_calibrate_arm(void)
{
    if (!begin_interactive_calibration(true, false)) return;
    printf("Receiver three-position shutdown calibration started.\n");
    uint16_t run = capture_channel(RC_CHANNEL_ARM, "Put the shutdown switch in RUN/ON.");
    uint16_t stop_1 = run ? capture_channel(RC_CHANNEL_ARM,
                                            "Put the shutdown switch in the first STOP/OFF position.") : 0;
    uint16_t stop_2 = stop_1 ? capture_channel(RC_CHANNEL_ARM,
                                               "Put the shutdown switch in the second STOP/OFF position.") : 0;

    controller_lock();
    bool distinct = run && stop_1 && stop_2 &&
                    abs((int32_t)run - (int32_t)stop_1) >= 250 &&
                    abs((int32_t)run - (int32_t)stop_2) >= 250 &&
                    abs((int32_t)stop_1 - (int32_t)stop_2) >= 250;
    if (distinct) {
        arm_cal = (arm_calibration_t){
            .run_us = run,
            .stop_1_us = stop_1,
            .stop_2_us = stop_2,
            .loaded_from_nvs = true,
        };
        esp_err_t err = config_store_save_arm(&arm_cal);
        printf(err == ESP_OK
                   ? "OK: receiver arm-channel calibration saved\n"
                   : "ERR: failed to save arm calibration: %s\n",
               err == ESP_OK ? "" : esp_err_to_name(err));
    } else if (run && stop_1 && stop_2) {
        printf("ERR: RUN, STOP 1, and STOP 2 must each differ by at least 250 us\n");
    }
    arm_cycle_ready = false;
    calibration_in_progress = false;
    controller_unlock();
}

void powertrain_calibrate_tv_mode(void)
{
    if (!begin_interactive_calibration(true, false)) return;
    printf("CH5 torque-vectoring selector calibration started.\n");
    uint16_t off = capture_channel(RC_CHANNEL_TV_MODE, "Put CH5 in OFF.");
    uint16_t straight = off ? capture_channel(RC_CHANNEL_TV_MODE, "Put CH5 in STRAIGHT.") : 0;
    uint16_t full = straight ? capture_channel(RC_CHANNEL_TV_MODE, "Put CH5 in FULL.") : 0;

    controller_lock();
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
    controller_unlock();
}

void powertrain_calibrate_imu(void)
{
    if (!begin_interactive_calibration(true, false)) return;
    if (!imu_initialized) {
        printf("ERR: ISM330DHCX is not available\n");
        controller_lock();
        calibration_in_progress = false;
        controller_unlock();
        return;
    }

    wait_for_enter("Place the car level and completely still.");
    printf("Sampling IMU gyro bias for 2 seconds...\n");
    bool success = imu_sensor_calibrate_bias(2000);
    printf(success
               ? "OK: IMU yaw-rate bias calibrated for this boot\n"
               : "ERR: IMU calibration failed; keep the car still and level, then retry\n");
    controller_lock();
    calibration_in_progress = false;
    controller_unlock();
}

void powertrain_cal_esc_arm(void)
{
    controller_lock();
    if (!calibration_allowed(true, true)) {
        controller_unlock();
        return;
    }
    system_state = SYSTEM_ESC_CAL_ARMED;
    esc_cal_last_command_us = esp_timer_get_time();
    esc_output_set_safe();
    printf("OK: ESC calibration armed. Keep ESC battery disconnected, then send 'cal esc max'.\n");
    controller_unlock();
}

void powertrain_cal_esc_max(void)
{
    controller_lock();
    if (system_state != SYSTEM_ESC_CAL_ARMED &&
        system_state != SYSTEM_ESC_CAL_MAX &&
        system_state != SYSTEM_ESC_CAL_MIN) {
        printf("ERR: send 'cal esc arm' before 'cal esc max'\n");
        controller_unlock();
        return;
    }
    if (!throttle_is_neutral()) {
        printf("ERR: receiver throttle is not neutral\n");
        controller_unlock();
        return;
    }
    if (!shutdown_channel_is_safe()) {
        printf("ERR: receiver arm switch must be in STOP for calibration\n");
        controller_unlock();
        return;
    }
    system_state = SYSTEM_ESC_CAL_MAX;
    esc_cal_last_command_us = esp_timer_get_time();
    esc_output_set_all(esc_limits.throttle_max_us, esc_limits.reverse_low_us);
    printf("OK: maximum throttle active. Power ESCs, wait for endpoint beeps, then send 'cal esc min'.\n");
    controller_unlock();
}

void powertrain_cal_esc_min(void)
{
    controller_lock();
    if (system_state != SYSTEM_ESC_CAL_MAX) {
        printf("ERR: send 'cal esc max' before 'cal esc min'\n");
        controller_unlock();
        return;
    }
    if (!throttle_is_neutral()) {
        printf("ERR: receiver throttle is not neutral\n");
        controller_unlock();
        return;
    }
    if (!shutdown_channel_is_safe()) {
        printf("ERR: receiver arm switch must be in STOP for calibration\n");
        controller_unlock();
        return;
    }
    system_state = SYSTEM_ESC_CAL_MIN;
    esc_cal_last_command_us = esp_timer_get_time();
    esc_output_set_all(esc_limits.throttle_min_us, esc_limits.reverse_low_us);
    printf("OK: minimum throttle active. Wait for ready beeps, then send 'cal cancel'.\n");
    controller_unlock();
}

void powertrain_cal_manual(void)
{
    controller_lock();
    if (!calibration_allowed(false, true)) {
        controller_unlock();
        return;
    }
    system_state = SYSTEM_ESC_CAL_MANUAL;
    esc_cal_last_command_us = esp_timer_get_time();
    printf("OK: direct throttle relay active for 30 seconds; all reverse outputs remain low.\n");
    controller_unlock();
}

void powertrain_cal_cancel(void)
{
    controller_lock();
    system_state = SYSTEM_DISARMED;
    calibration_in_progress = false;
    arm_cycle_ready = false;
    reset_drive_state();
    esc_output_set_safe();
    printf("OK: calibration canceled; safe outputs restored\n");
    controller_unlock();
}

void powertrain_arm_from_cli(void)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED) {
        printf("ERR: controller must be DISARMED\n");
        controller_unlock();
        return;
    }
    if (!arm_cycle_ready) {
        printf("ERR: cycle the receiver shutdown switch through STOP before arming\n");
        controller_unlock();
        return;
    }
    if (drive_arm_allowed()) {
        arm_drive();
    }
    controller_unlock();
}

void powertrain_disarm(void)
{
    controller_lock();
    disarm_to_safe("OK: drive disarmed; safe outputs restored", true);
    if (system_state == SYSTEM_DISARMED) {
        esc_output_set_safe();
    }
    controller_unlock();
}

static esp_err_t save_drive_config_or_report(const char *success_message)
{
    esp_err_t err = config_store_save_drive(&drive_config);
    if (err == ESP_OK) {
        drive_config.loaded_from_nvs = true;
        printf("%s\n", success_message);
    } else {
        printf("ERR: failed to save drive configuration: %s\n", esp_err_to_name(err));
    }
    return err;
}

void powertrain_set_reverse_limit(uint8_t percent)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED || percent > 100) {
        printf("ERR: reverse limit must be 0-100 and changed while DISARMED\n");
        controller_unlock();
        return;
    }
    drive_config.reverse_limit_percent = percent;
    char message[64];
    snprintf(message, sizeof(message), "OK: reverse limit set to %u%%", percent);
    save_drive_config_or_report(message);
    controller_unlock();
}

void powertrain_set_failsafe(uint16_t pulse_us, uint16_t window_us)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED || pulse_us < 800 || pulse_us > 2200 ||
        window_us < 1 || window_us > 100) {
        printf("ERR: failsafe pulse must be 800-2200 us, window 1-100 us, while DISARMED\n");
        controller_unlock();
        return;
    }
    drive_config.receiver_failsafe_us = pulse_us;
    drive_config.receiver_failsafe_window_us = window_us;
    drive_config.receiver_failsafe_enabled = true;
    failsafe_candidate_since_us = 0;
    char message[96];
    snprintf(message, sizeof(message), "OK: throttle failsafe set to %u +/- %u us",
             pulse_us, window_us);
    save_drive_config_or_report(message);
    controller_unlock();
}

void powertrain_set_failsafe_enabled(bool enabled)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED) {
        printf("ERR: throttle-pulse failsafe can only be changed while DISARMED\n");
        controller_unlock();
        return;
    }
    drive_config.receiver_failsafe_enabled = enabled;
    failsafe_candidate_since_us = 0;
    save_drive_config_or_report(enabled
                                    ? "OK: throttle-pulse failsafe enabled"
                                    : "OK: throttle-pulse failsafe disabled");
    controller_unlock();
}

void powertrain_set_motor_poles(uint8_t poles)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED || poles < 2 || poles > 60 || (poles % 2) != 0) {
        printf("ERR: motor pole count must be even, 2-60, and changed while DISARMED\n");
        controller_unlock();
        return;
    }
    drive_config.motor_poles = poles;
    drive_config.rpm_pulses_per_revolution = poles / 2;
    if (rpm_initialized) {
        rpm_sensor_set_pulses_per_revolution(drive_config.rpm_pulses_per_revolution);
    }
    char message[160];
    snprintf(message, sizeof(message),
             "OK: motor poles=%u and RPM PPR=%u (legacy one-pulse-per-electrical-revolution assumption)",
             poles, drive_config.rpm_pulses_per_revolution);
    save_drive_config_or_report(message);
    controller_unlock();
}

void powertrain_set_rpm_pulses_per_revolution(uint16_t pulses_per_revolution)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED || pulses_per_revolution < 1 ||
        pulses_per_revolution > 120) {
        printf("ERR: RPM pulses per mechanical revolution must be 1-120 while DISARMED\n");
        controller_unlock();
        return;
    }
    drive_config.rpm_pulses_per_revolution = pulses_per_revolution;
    if (rpm_initialized) {
        rpm_sensor_set_pulses_per_revolution(pulses_per_revolution);
    }
    char message[96];
    snprintf(message, sizeof(message),
             "OK: RPM conversion set to %u pulses per mechanical revolution",
             pulses_per_revolution);
    save_drive_config_or_report(message);
    controller_unlock();
}

bool powertrain_set_steering_trim(float trim_degrees)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED || !isfinite(trim_degrees) ||
        trim_degrees < -15.0f || trim_degrees > 15.0f) {
        printf("ERR: steering trim must be finite, -15 to +15 degrees, and remain inside calibrated endpoints while DISARMED\n");
        controller_unlock();
        return false;
    }
    int16_t trim_tenths_deg = (int16_t)lroundf(trim_degrees * 10.0f);
    if (!steering_trim_fits_endpoints(trim_tenths_deg)) {
        printf("ERR: steering trim would move center outside calibrated endpoints\n");
        controller_unlock();
        return false;
    }
    int16_t previous_trim_tenths_deg = drive_config.steering_trim_tenths_deg;
    drive_config.steering_trim_tenths_deg = trim_tenths_deg;
    char message[96];
    snprintf(message, sizeof(message), "OK: steering command-space trim saved to NVS at %+.1f degrees",
             (double)drive_config.steering_trim_tenths_deg / 10.0);
    esp_err_t err = save_drive_config_or_report(message);
    if (err != ESP_OK) {
        drive_config.steering_trim_tenths_deg = previous_trim_tenths_deg;
    }
    controller_unlock();
    return err == ESP_OK;
}

void powertrain_set_steering_smoothing(uint16_t smoothing_ms)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED || smoothing_ms > 500) {
        printf("ERR: steering smoothing must be 0-500 ms while DISARMED\n");
        controller_unlock();
        return;
    }
    drive_config.steering_smoothing_ms = smoothing_ms;
    char message[96];
    snprintf(message, sizeof(message), "OK: steering smoothing time constant set to %u ms",
             smoothing_ms);
    save_drive_config_or_report(message);
    controller_unlock();
}

void powertrain_set_tv_enabled(bool enabled)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED) {
        printf("ERR: torque vectoring can only be configured while DISARMED\n");
        controller_unlock();
        return;
    }
    if (enabled && (!steering_cal.loaded_from_nvs || !tv_mode_cal.loaded_from_nvs)) {
        printf("ERR: run 'cal steering' and 'cal tv' before enabling torque vectoring\n");
        controller_unlock();
        return;
    }
    if (enabled) {
        if (!sensor_task_started) {
            printf("ERR: sensor task is unavailable; torque vectoring cannot be enabled\n");
            controller_unlock();
            return;
        }
        imu_snapshot_t imu = {0};
        if (imu_initialized) {
            imu_sensor_get_snapshot(&imu);
        }
        if (!imu.valid || !imu.bias_calibrated) {
            printf("ERR: run 'cal imu' successfully before enabling torque vectoring\n");
            controller_unlock();
            return;
        }
        for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
            if (!rpm_seen[wheel]) {
                printf("ERR: RPM input %s has not produced a valid pulse this boot; "
                       "use 'monitor rpm' and spin each wheel\n",
                       wheel_names[wheel]);
                controller_unlock();
                return;
            }
        }
    }
    drive_config.torque_vectoring_enabled = enabled;
    save_drive_config_or_report(enabled
                                    ? "OK: torque vectoring enabled; CH5 still selects its mode"
                                    : "OK: torque vectoring disabled");
    controller_unlock();
}

void powertrain_set_tv_authority(uint8_t percent)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED || percent > 25) {
        printf("ERR: vectoring authority must be 0-25 percent while DISARMED\n");
        controller_unlock();
        return;
    }
    drive_config.tv_authority_percent = percent;
    char message[80];
    snprintf(message, sizeof(message), "OK: vectoring authority set to +/- %u%%", percent);
    save_drive_config_or_report(message);
    controller_unlock();
}

void powertrain_set_tv_gains(float yaw_gain_dps,
                             float turn_rpm_gain,
                             float yaw_kp,
                             float yaw_ki,
                             float rpm_kp)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED ||
        !isfinite(yaw_gain_dps) || !isfinite(turn_rpm_gain) ||
        !isfinite(yaw_kp) || !isfinite(yaw_ki) || !isfinite(rpm_kp) ||
        yaw_gain_dps < 0.0f || yaw_gain_dps > 500.0f ||
        turn_rpm_gain < 0.0f || turn_rpm_gain > 1.0f ||
        yaw_kp < 0.0f || yaw_kp > 0.01f ||
        yaw_ki < 0.0f || yaw_ki > 0.01f ||
        rpm_kp < 0.0f || rpm_kp > 5.0f) {
        printf("ERR: invalid TV gains or controller is not DISARMED\n");
        controller_unlock();
        return;
    }
    drive_config.tv_turn_yaw_gain_dps = yaw_gain_dps;
    drive_config.tv_turn_rpm_gain = turn_rpm_gain;
    drive_config.tv_yaw_kp = yaw_kp;
    drive_config.tv_yaw_ki = yaw_ki;
    drive_config.tv_rpm_kp = rpm_kp;
    save_drive_config_or_report("OK: torque-vectoring gains saved");
    controller_unlock();
}

void powertrain_set_imu_yaw_sign(int8_t sign)
{
    controller_lock();
    if (system_state != SYSTEM_DISARMED || (sign != -1 && sign != 1)) {
        printf("ERR: IMU yaw sign must be -1 or 1 while DISARMED\n");
        controller_unlock();
        return;
    }
    drive_config.imu_yaw_sign = sign;
    if (imu_initialized) imu_sensor_set_yaw_sign(sign);
    save_drive_config_or_report("OK: IMU yaw sign saved");
    controller_unlock();
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
    throttle_calibration_t throttle_snapshot;
    steering_calibration_t steering_snapshot;
    arm_calibration_t arm_snapshot;
    tv_mode_calibration_t tv_cal_snapshot;
    drive_config_t config_snapshot;
    torque_vectoring_output_t vector_snapshot;
    bool rpm_seen_snapshot[POWERTRAIN_WHEEL_COUNT];
    system_state_t state_snapshot;
    bool arm_cycle_snapshot;
    uint16_t servo_pulse_us;
    steering_curve_sample_t steering_geometry;
    tv_mode_t requested_mode;
    bool arm_run = false;
    bool arm_valid;
    steering_input_filter_status_t steering_filter_status;
    uint16_t steering_filter_output_us;
    uint32_t steering_rejected_spikes;

    controller_lock();
    throttle_snapshot = throttle_cal;
    steering_snapshot = steering_cal;
    arm_snapshot = arm_cal;
    tv_cal_snapshot = tv_mode_cal;
    config_snapshot = drive_config;
    vector_snapshot = vectoring_output;
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        rpm_seen_snapshot[wheel] = rpm_seen[wheel];
    }
    state_snapshot = system_state;
    arm_cycle_snapshot = arm_cycle_ready;
    servo_pulse_us = servo_output_get_pulse();
    steering_geometry = steering_geometry_sample();
    requested_mode = requested_tv_mode();
    arm_valid = arm_channel_state(&arm_run);
    steering_filter_status = steering_input_filter_status(&steering_input_filter);
    steering_filter_output_us = steering_input_filter.output_us;
    steering_rejected_spikes = steering_input_filter.rejected_spike_count;
    esc_output_get(&outputs);
    if (rpm_initialized) rpm_sensor_get_snapshot(&rpm);
    if (imu_initialized) imu_sensor_get_snapshot(&imu);
    controller_unlock();

    uint32_t sensor_overruns;
    uint32_t powertrain_overruns;
    uint32_t sensor_max_us;
    uint32_t powertrain_max_us;
    UBaseType_t sensor_stack_free;
    UBaseType_t powertrain_stack_free;
    portENTER_CRITICAL(&telemetry_lock);
    sensor_overruns = sensor_overrun_count;
    powertrain_overruns = powertrain_overrun_count;
    sensor_max_us = sensor_max_execution_us;
    powertrain_max_us = powertrain_max_execution_us;
    sensor_stack_free = sensor_stack_min_free_bytes;
    powertrain_stack_free = powertrain_stack_min_free_bytes;
    portEXIT_CRITICAL(&telemetry_lock);

    printf("\nRC car powertrain status\n");
    printf("  state: %s, rearm cycle: %s\n",
           state_name(state_snapshot), arm_cycle_snapshot ? "ready" : "required");
    print_channel("throttle", RC_CHANNEL_THROTTLE);
    print_channel("steering", RC_CHANNEL_STEERING);
    printf("  steering servo: GPIO%d, output=%u us\n",
           PIN_STEERING_SERVO_OUTPUT, servo_pulse_us);
    if (steering_filter_status == STEERING_INPUT_FILTER_ACTIVE) {
        printf("  steering input filter: active, median=%u us, "
               "rejected full-range spikes=%lu\n",
               steering_filter_output_us,
               (unsigned long)steering_rejected_spikes);
    } else {
        printf("  steering input filter: warmup, median=unavailable, "
               "rejected full-range spikes=%lu\n",
               (unsigned long)steering_rejected_spikes);
    }
    printf("  loop timing: sensor max=%lu/%u us overruns=%lu, "
           "powertrain max=%lu/%u us overruns=%lu\n",
           (unsigned long)sensor_max_us, SENSOR_UPDATE_INTERVAL_MS * 1000,
           (unsigned long)sensor_overruns,
           (unsigned long)powertrain_max_us, DRIVE_UPDATE_INTERVAL_MS * 1000,
           (unsigned long)powertrain_overruns);
    printf("  stack minimum free: sensor=%lu bytes powertrain=%lu bytes\n",
           (unsigned long)sensor_stack_free, (unsigned long)powertrain_stack_free);
    printf("  heap: free=%lu min-free=%lu largest=%lu bytes\n",
           (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
           (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    print_channel("arm/stop", RC_CHANNEL_ARM);
    print_channel("TV CH5", RC_CHANNEL_TV_MODE);
    printf("  arm request: %s\n", arm_valid ? (arm_run ? "RUN" : "STOP") : "missing");
    printf("  throttle cal: full=%u neutral=%u reverse=%u deadband=%u source=%s\n",
           throttle_snapshot.full_throttle_us, throttle_snapshot.neutral_us,
           throttle_snapshot.full_reverse_us, throttle_snapshot.deadband_us,
           throttle_snapshot.loaded_from_nvs ? "nvs" : "defaults/unarmed");
    printf("  steering cal: left=%u center=%u right=%u deadband=%u source=%s\n",
           steering_snapshot.left_us, steering_snapshot.center_us, steering_snapshot.right_us,
           steering_snapshot.deadband_us,
           steering_snapshot.loaded_from_nvs ? "nvs" : "defaults");
    printf("  arm cal: run=%u stop1=%u stop2=%u source=%s\n",
           arm_snapshot.run_us, arm_snapshot.stop_1_us, arm_snapshot.stop_2_us,
           arm_snapshot.loaded_from_nvs ? "nvs" : "defaults/unarmed");
    printf("  CH5 cal: off=%u straight=%u full=%u source=%s\n",
           tv_cal_snapshot.off_us, tv_cal_snapshot.straight_us, tv_cal_snapshot.full_us,
           tv_cal_snapshot.loaded_from_nvs ? "nvs" : "defaults");
    printf("  drive: reverse_limit=%u%%, motor=%u poles, RPM PPR=%u\n",
           config_snapshot.reverse_limit_percent, config_snapshot.motor_poles,
           config_snapshot.rpm_pulses_per_revolution);
    printf("  steering config: trim=%+.1f command degrees, smoothing=%u ms\n",
           (double)config_snapshot.steering_trim_tenths_deg / 10.0,
           config_snapshot.steering_smoothing_ms);
    printf("  steering curve: servo=%+.2f deg LF=%+.2f deg RF=%+.2f deg "
           "average=%+.2f deg normalized=%+.3f%s%s\n",
           steering_geometry.servo_command_deg,
           steering_geometry.left_wheel_deg,
           steering_geometry.right_wheel_deg,
           steering_geometry.average_wheel_deg,
           steering_geometry.normalized_average,
           steering_geometry.valid ? "" : " invalid",
           steering_geometry.saturated ? " saturated" : "");
    if (config_snapshot.receiver_failsafe_enabled) {
        printf("  throttle-pulse failsafe: %u +/- %u us\n",
               config_snapshot.receiver_failsafe_us,
               config_snapshot.receiver_failsafe_window_us);
    } else {
        printf("  throttle-pulse failsafe: disabled\n");
    }
    printf("  TV: configured=%s, CH5=%s, active=%s, authority=+/- %u%%, reason=%s\n",
           config_snapshot.torque_vectoring_enabled ? "enabled" : "disabled",
           tv_mode_name(requested_mode),
           vector_snapshot.active ? tv_mode_name(vector_snapshot.active_mode) : "no",
           config_snapshot.tv_authority_percent,
           vector_snapshot.inactive_reason ? vector_snapshot.inactive_reason : "not evaluated");
    printf("  IMU: %s, bias=%s, yaw=%+.2f dps\n",
           imu.valid ? "valid" : "unavailable",
           imu.bias_calibrated ? "calibrated" : "not calibrated",
           imu.yaw_rate_dps);
    printf("  IMU mount: +X=%s, +Y=%s\n",
           IMU_MOUNT_X_POSITIVE_DIRECTION,
           IMU_MOUNT_Y_POSITIVE_DIRECTION);
    printf("  RPM: FL=%.0f%s FR=%.0f%s RL=%.0f%s RR=%.0f%s\n",
           rpm.rpm[0], rpm.valid[0] ? "" : "?",
           rpm.rpm[1], rpm.valid[1] ? "" : "?",
           rpm.rpm[2], rpm.valid[2] ? "" : "?",
           rpm.rpm[3], rpm.valid[3] ? "" : "?");
    printf("  RPM seen this boot: FL=%s FR=%s RL=%s RR=%s\n",
           rpm_seen_snapshot[0] ? "yes" : "no", rpm_seen_snapshot[1] ? "yes" : "no",
           rpm_seen_snapshot[2] ? "yes" : "no", rpm_seen_snapshot[3] ? "yes" : "no");
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
    controller_lock();
    float trim_degrees = (float)drive_config.steering_trim_tenths_deg / 10.0f;
    controller_unlock();

    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    printf("Monitoring steering for 30 seconds with persistent trim=%+.1f degrees.\n",
           (double)trim_degrees);
    while (esp_timer_get_time() < end_at_us) {
        rc_channel_sample_t sample;
        steering_curve_sample_t geometry = steering_geometry_sample();
        controller_lock();
        steering_input_filter_status_t filter_status =
            steering_input_filter_status(&steering_input_filter);
        uint16_t filter_output_us = steering_input_filter.output_us;
        uint32_t rejected_spikes = steering_input_filter.rejected_spike_count;
        controller_unlock();
        if (rc_input_get(RC_CHANNEL_STEERING, &sample)) {
            char median_text[24];
            if (filter_status == STEERING_INPUT_FILTER_ACTIVE) {
                snprintf(median_text, sizeof(median_text),
                         "%u us", filter_output_us);
            } else {
                snprintf(median_text, sizeof(median_text), "unavailable");
            }
            printf("steering: input=%u us median=%s output=%u us servo=%+.2f deg "
                   "LF=%+.2f deg RF=%+.2f deg average=%+.2f deg normalized=%+.3f "
                   "filter=%s rejected=%lu%s%s\n",
                   sample.pulse_us, median_text, servo_output_get_pulse(),
                   geometry.servo_command_deg,
                   geometry.left_wheel_deg,
                   geometry.right_wheel_deg,
                   geometry.average_wheel_deg,
                   geometry.normalized_average,
                   steering_input_filter_status_name(filter_status),
                   (unsigned long)rejected_spikes,
                   geometry.valid ? "" : " invalid",
                   geometry.saturated ? " saturated" : "");
        } else {
            printf("steering: input=missing, output=%u us (center fallback)\n",
                   servo_output_get_pulse());
        }
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_steering_with_trim(float trim_degrees)
{
    if (!powertrain_set_steering_trim(trim_degrees)) {
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(DRIVE_UPDATE_INTERVAL_MS));
    powertrain_monitor_steering();
}

void powertrain_monitor_arm(void)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    printf("Monitoring receiver arm/stop PWM for 30 seconds.\n");
    while (esp_timer_get_time() < end_at_us) {
        rc_channel_sample_t sample;
        if (rc_input_get(RC_CHANNEL_ARM, &sample)) {
            controller_lock();
            bool rearm_ready = arm_cycle_ready;
            controller_unlock();
            printf("arm: %u us, position=%s, request=%s, rearm_cycle=%s\n",
                   sample.pulse_us,
                   arm_position_name(sample.pulse_us),
                   arm_requests_run(sample.pulse_us) ? "RUN" : "STOP",
                   rearm_ready ? "ready" : "required");
        } else {
            int64_t now_us = esp_timer_get_time();
            printf("arm: missing, last=%u us, age=%lu ms, GPIO%d level=%d\n",
                   sample.pulse_us,
                   (unsigned long)rc_sample_age_ms(&sample, now_us),
                   PIN_RC_ARM_INPUT,
                   gpio_get_level(PIN_RC_ARM_INPUT));
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
        printf("RPM conversion: %u pulses/mechanical-revolution\n",
               rpm.pulses_per_revolution);
        for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
            printf("  %s: rpm=%.1f%s frequency=%.2f Hz edges=%lu window=%lu us\n",
                   wheel_names[wheel], rpm.rpm[wheel],
                   rpm.valid[wheel] ? "" : " missing",
                   rpm.frequency_hz[wheel],
                   (unsigned long)rpm.edge_count[wheel],
                   (unsigned long)rpm.window_us[wheel]);
        }
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
        controller_lock();
        torque_vectoring_output_t output = vectoring_output;
        tv_mode_t mode = requested_tv_mode();
        steering_curve_sample_t geometry = steering_geometry_sample();
        controller_unlock();
        printf("TV: CH5=%s active=%s steer=%+.2f deg wheels=%+.2f/%+.2f deg "
               "target_yaw=%+.2f error=%+.2f rpm_error=%+.3f "
               "correction=[%+.3f %+.3f %+.3f %+.3f] reason=%s\n",
               tv_mode_name(mode),
               output.active ? "yes" : "no",
               geometry.servo_command_deg,
               geometry.left_wheel_deg,
               geometry.right_wheel_deg,
               output.target_yaw_rate_dps,
               output.yaw_error_dps,
               output.side_rpm_error,
               output.wheel_correction[0],
               output.wheel_correction[1],
               output.wheel_correction[2],
               output.wheel_correction[3],
               output.inactive_reason ? output.inactive_reason : "not evaluated");
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
    printf("  monitor steering trim <-15..15>\n");
    printf("  config reverse <0-100>\n");
    printf("  config failsafe off | <pulse_us> <window_us>\n");
    printf("  config rpm poles <even 2-60>\n");
    printf("  config rpm ppr <1-120>\n");
    printf("  config steering trim <-15..15>\n");
    printf("  config steering smoothing <0-500>\n");
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
        int64_t execution_started_us = esp_timer_get_time();
        if (imu_initialized) {
            imu_sensor_update();
        }
        elapsed_ms += SENSOR_UPDATE_INTERVAL_MS;
        if (rpm_initialized && elapsed_ms >= RPM_UPDATE_INTERVAL_MS) {
            rpm_sensor_update();
            rpm_snapshot_t rpm;
            rpm_sensor_get_snapshot(&rpm);
            controller_lock();
            for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
                if (rpm.valid[wheel]) {
                    rpm_seen[wheel] = true;
                }
            }
            controller_unlock();
            elapsed_ms = 0;
        }
        uint32_t execution_us = (uint32_t)(esp_timer_get_time() - execution_started_us);
        portENTER_CRITICAL(&telemetry_lock);
        if (execution_us > sensor_max_execution_us) {
            sensor_max_execution_us = execution_us;
        }
        sensor_stack_min_free_bytes = uxTaskGetStackHighWaterMark(NULL);
        portEXIT_CRITICAL(&telemetry_lock);
        wait_for_next_period(&last_wake,
                             pdMS_TO_TICKS(SENSOR_UPDATE_INTERVAL_MS),
                             &sensor_overrun_count);
    }
}

static void powertrain_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        int64_t execution_started_us = esp_timer_get_time();
        controller_lock();
        update_steering_servo();
        handle_receiver_arm_control();

        if (system_state == SYSTEM_DRIVE_ARMED) {
            update_drive_outputs();
        } else if (system_state == SYSTEM_ESC_CAL_MANUAL) {
            rc_channel_sample_t throttle;
            if (throttle_matches_failsafe() || !throttle_sample(&throttle)) {
                powertrain_cal_cancel();
                printf("\nWARN: manual ESC relay stopped due to receiver signal loss or failsafe\n> ");
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
        controller_unlock();

        uint32_t execution_us = (uint32_t)(esp_timer_get_time() - execution_started_us);
        portENTER_CRITICAL(&telemetry_lock);
        if (execution_us > powertrain_max_execution_us) {
            powertrain_max_execution_us = execution_us;
        }
        powertrain_stack_min_free_bytes = uxTaskGetStackHighWaterMark(NULL);
        portEXIT_CRITICAL(&telemetry_lock);

        wait_for_next_period(&last_wake,
                             pdMS_TO_TICKS(DRIVE_UPDATE_INTERVAL_MS),
                             &powertrain_overrun_count);
    }
}

esp_err_t powertrain_controller_init(void)
{
    esp_err_t err = esc_output_init(&esc_limits);
    if (err != ESP_OK) return err;

    controller_mutex = xSemaphoreCreateRecursiveMutex();
    if (controller_mutex == NULL) {
        esc_output_set_safe();
        return ESP_ERR_NO_MEM;
    }

    err = config_store_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS unavailable; compiled defaults remain unarmed: %s",
                 esp_err_to_name(err));
    }

    config_store_load_throttle(&throttle_cal);
    config_store_load_steering(&steering_cal);
    config_store_load_arm(&arm_cal);
    config_store_load_tv_mode(&tv_mode_cal);
    config_store_load_drive(&drive_config);
    steering_input_filter_init(&steering_input_filter);
    if (!steering_trim_fits_endpoints(drive_config.steering_trim_tenths_deg)) {
        ESP_LOGW(TAG, "Stored steering trim does not fit calibrated endpoints; using zero trim");
        drive_config.steering_trim_tenths_deg = 0;
    }

    err = servo_output_init(trimmed_steering_center_us());
    if (err != ESP_OK) return err;
    err = rc_input_init();
    if (err != ESP_OK) return err;

    err = rpm_sensor_init(drive_config.rpm_pulses_per_revolution);
    if (err == ESP_OK) {
        rpm_initialized = true;
    } else {
        ESP_LOGW(TAG, "RPM capture unavailable: %s", esp_err_to_name(err));
    }

    err = imu_sensor_init(drive_config.imu_yaw_sign);
    if (err == ESP_OK) {
        imu_initialized = true;
        ESP_LOGI(TAG, "Keep the car stationary: calibrating IMU yaw bias for %u seconds",
                 IMU_STARTUP_CALIBRATION_MS / 1000);
        if (imu_sensor_calibrate_bias(IMU_STARTUP_CALIBRATION_MS)) {
            ESP_LOGI(TAG, "IMU startup yaw-bias calibration complete");
        } else {
            ESP_LOGW(TAG, "IMU startup calibration failed; torque vectoring remains unavailable until 'cal imu' succeeds");
        }
    } else {
        ESP_LOGW(TAG, "ISM330DHCX unavailable: %s", esp_err_to_name(err));
    }

    reset_drive_state();
    esc_output_set_safe();
    servo_output_set_neutral();
    steering_filtered_q16 = (int32_t)servo_output_get_pulse() << 16;
    steering_filter_initialized = true;
    return ESP_OK;
}

esp_err_t powertrain_controller_start(void)
{
    BaseType_t created = xTaskCreatePinnedToCore(powertrain_task,
                                                 "powertrain",
                                                 6144,
                                                 NULL,
                                                 5,
                                                 &powertrain_task_handle,
                                                 1);
    if (created != pdPASS) {
        esc_output_set_safe();
        return ESP_ERR_NO_MEM;
    }

    created = xTaskCreate(sensor_task,
                          "sensors",
                          4096,
                          NULL,
                          6,
                          &sensor_task_handle);
    if (created == pdPASS) {
        sensor_task_started = true;
    } else {
        ESP_LOGW(TAG, "sensor task could not start; torque vectoring remains unavailable");
    }
    return ESP_OK;
}
