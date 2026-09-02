#include "powertrain_controller.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "config_store.h"
#include "cornering_control.h"
#include "default_config.h"
#include "dragy_sensor.h"
#include "driver/gpio.h"
#include "drivetrain_control.h"
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
#define ARM_UNRECOGNIZED_CONFIRM_US 60000
#define ARM_RUN_STABLE_US 250000
#define ESC_CAL_TIMEOUT_US 30000000
#define REMOTE_CAL_TIMEOUT_US 120000000
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

typedef struct {
    bool active;
    bool sampling;
    powertrain_calibration_kind_t kind;
    uint8_t step;
    uint8_t total_steps;
    uint16_t values_us[3];
    int64_t last_action_us;
    char prompt[POWERTRAIN_CALIBRATION_PROMPT_MAX];
    char message[POWERTRAIN_CALIBRATION_MESSAGE_MAX];
} remote_calibration_state_t;

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
static uint16_t current_base_throttle_us = ESC_THROTTLE_MIN_US;
static drive_direction_t active_direction = DRIVE_DIRECTION_FORWARD;
static drive_direction_t pending_direction = DRIVE_DIRECTION_FORWARD;
static int64_t direction_hold_until_us;
static bool arm_cycle_ready;
static uint32_t drive_inhibit_flags;
static uint32_t drive_inhibit_count;
static const char *last_drive_inhibit_reason = "none";
static bool drive_recovery_report_pending;
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
static remote_calibration_state_t remote_calibration = {
    .message = "No browser calibration has been started",
};

#define DRIVE_INHIBIT_CH4 (1U << 0)
#define DRIVE_INHIBIT_THROTTLE_LOST (1U << 1)
#define DRIVE_INHIBIT_THROTTLE_FAILSAFE (1U << 2)
#define DRIVE_INHIBIT_CLI (1U << 3)

typedef struct {
    bool speed_valid;
    bool steering_limited;
    float vehicle_speed_mps;
    float requested_average_wheel_deg;
    float maximum_average_wheel_deg;
} steering_limit_telemetry_t;

static steering_limit_telemetry_t steering_limit_telemetry;

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

static float steering_pulse_to_servo_command_deg(uint16_t pulse_us)
{
    uint16_t center_us = trimmed_steering_center_us();
    int32_t delta = (int32_t)pulse_us - center_us;
    if (abs(delta) <= 1) {
        return 0.0f;
    }

    bool right = pulse_is_on_endpoint_side(pulse_us,
                                           steering_cal.right_us,
                                           center_us);
    float command_deg = (float)abs(delta) *
                        (90.0f / SERVO_NOMINAL_US_PER_90_DEG);
    return right ? command_deg : -command_deg;
}

static uint16_t steering_servo_command_deg_to_pulse(float command_deg)
{
    if (!isfinite(command_deg)) {
        return trimmed_steering_center_us();
    }
    if (command_deg > 45.0f) command_deg = 45.0f;
    if (command_deg < -45.0f) command_deg = -45.0f;

    uint16_t center_us = trimmed_steering_center_us();
    uint16_t endpoint_us = command_deg >= 0.0f
                               ? steering_cal.right_us
                               : steering_cal.left_us;
    int32_t direction = endpoint_us >= center_us ? 1 : -1;
    int32_t delta_us = (int32_t)lroundf(fabsf(command_deg) *
                                        (SERVO_NOMINAL_US_PER_90_DEG / 90.0f));
    return clamp_steering_pulse((int32_t)center_us + direction * delta_us);
}

static bool update_vehicle_speed_estimate(float *speed_mps)
{
    rpm_snapshot_t rpm = {0};
    float measured_speed_mps = 0.0f;
    bool measured = false;
    if (rpm_initialized) {
        rpm_sensor_get_snapshot(&rpm);
        measured = cornering_vehicle_speed_from_rear_rpm(
            &rpm, DEFAULT_WHEEL_DIAMETER_M, &measured_speed_mps);
    }

    if (measured) {
        steering_limit_telemetry.speed_valid = true;
        steering_limit_telemetry.vehicle_speed_mps = measured_speed_mps;
    } else if (system_state != SYSTEM_DRIVE_ARMED) {
        steering_limit_telemetry.speed_valid = false;
        steering_limit_telemetry.vehicle_speed_mps = 0.0f;
    }

    *speed_mps = steering_limit_telemetry.vehicle_speed_mps;
    return steering_limit_telemetry.speed_valid;
}

static uint16_t apply_speed_steering_limit(uint16_t requested_us)
{
    float requested_servo_deg = steering_pulse_to_servo_command_deg(requested_us);
    steering_curve_sample_t requested_geometry = {0};
    if (!steering_curve_sample(requested_servo_deg, &requested_geometry)) {
        steering_limit_telemetry.steering_limited = false;
        steering_limit_telemetry.requested_average_wheel_deg = 0.0f;
        steering_limit_telemetry.maximum_average_wheel_deg = 0.0f;
        return requested_us;
    }

    float speed_mps = 0.0f;
    bool speed_valid = update_vehicle_speed_estimate(&speed_mps);
    steering_curve_sample_t endpoint_geometry = {0};
    float endpoint_servo_deg = requested_geometry.average_wheel_deg < 0.0f
                                   ? -45.0f
                                   : 45.0f;
    steering_curve_sample(endpoint_servo_deg, &endpoint_geometry);
    float physical_max_angle_deg = fabsf(endpoint_geometry.average_wheel_deg);
    float maximum_angle_deg = physical_max_angle_deg;

    if (drive_config.steering_speed_limit_enabled && speed_valid) {
        maximum_angle_deg = cornering_max_average_wheel_angle_deg(
            speed_mps,
            DEFAULT_WHEELBASE_M,
            drive_config.steering_lateral_accel_g,
            physical_max_angle_deg);
    }

    steering_limit_telemetry.requested_average_wheel_deg =
        requested_geometry.average_wheel_deg;
    steering_limit_telemetry.maximum_average_wheel_deg = maximum_angle_deg;
    steering_limit_telemetry.steering_limited =
        drive_config.steering_speed_limit_enabled && speed_valid &&
        fabsf(requested_geometry.average_wheel_deg) > maximum_angle_deg;
    if (!steering_limit_telemetry.steering_limited) {
        return requested_us;
    }

    float limited_average_deg = copysignf(maximum_angle_deg,
                                          requested_geometry.average_wheel_deg);
    float limited_servo_deg = 0.0f;
    if (!steering_curve_servo_for_average(limited_average_deg,
                                          &limited_servo_deg)) {
        return requested_us;
    }
    return steering_servo_command_deg_to_pulse(limited_servo_deg);
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
        return false;
    }

    rc_channel_sample_t sample;
    if (!throttle_sample(&sample)) {
        return false;
    }

    return abs((int32_t)sample.pulse_us -
               (int32_t)drive_config.receiver_failsafe_us) <=
           drive_config.receiver_failsafe_window_us;
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
    target_us = apply_speed_steering_limit(target_us);
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

    float servo_command_deg = steering_pulse_to_servo_command_deg(
        servo_output_get_pulse());
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

static const char *drive_inhibit_reason(uint32_t flags)
{
    if ((flags & DRIVE_INHIBIT_CLI) != 0) return "CLI safe-output hold";
    if ((flags & DRIVE_INHIBIT_CH4) != 0) return "CH4 is not valid RUN";
    if ((flags & DRIVE_INHIBIT_THROTTLE_LOST) != 0) return "throttle signal missing";
    if ((flags & DRIVE_INHIBIT_THROTTLE_FAILSAFE) != 0) {
        return "configured throttle failsafe pulse";
    }
    return "none";
}

static void hold_drive_outputs_safe(const char *reason)
{
    current_base_throttle_us = esc_limits.throttle_min_us;
    active_direction = DRIVE_DIRECTION_FORWARD;
    pending_direction = DRIVE_DIRECTION_FORWARD;
    direction_hold_until_us = 0;
    torque_vectoring_reset(&vectoring_state);
    vectoring_output = (torque_vectoring_output_t){
        .inactive_reason = reason,
    };
    esc_output_set_safe();
}

static void set_drive_inhibit(uint32_t flag, const char *message)
{
    bool newly_inhibited = (drive_inhibit_flags & flag) == 0;
    drive_inhibit_flags |= flag;
    const char *reason = drive_inhibit_reason(drive_inhibit_flags);
    hold_drive_outputs_safe(reason);
    if (newly_inhibited) {
        drive_inhibit_count++;
        last_drive_inhibit_reason = reason;
    }
    drive_recovery_report_pending = false;
    if (newly_inhibited && message != NULL) {
        printf("\n%s; armed latch retained\n> ", message);
        fflush(stdout);
    }
}

static void clear_drive_inhibit(uint32_t flag)
{
    bool was_inhibited = drive_inhibit_flags != 0;
    drive_inhibit_flags &= ~flag;
    if (drive_inhibit_flags != 0) {
        hold_drive_outputs_safe(drive_inhibit_reason(drive_inhibit_flags));
    } else if (was_inhibited && system_state == SYSTEM_DRIVE_ARMED) {
        drive_recovery_report_pending = true;
    }
}

static void reset_drive_state(void)
{
    current_base_throttle_us = esc_limits.throttle_min_us;
    active_direction = DRIVE_DIRECTION_FORWARD;
    pending_direction = DRIVE_DIRECTION_FORWARD;
    direction_hold_until_us = 0;
    arm_run_candidate_since_us = 0;
    drive_inhibit_flags = 0;
    drive_inhibit_count = 0;
    last_drive_inhibit_reason = "none";
    drive_recovery_report_pending = false;
    torque_vectoring_reset(&vectoring_state);
    vectoring_output = (torque_vectoring_output_t){
        .inactive_reason = "drive is disarmed",
    };
}

static void return_to_disarmed(const char *message, bool require_arm_cycle)
{
    system_state = SYSTEM_DISARMED;
    reset_drive_state();
    esc_output_set_safe();
    if (require_arm_cycle) {
        arm_cycle_ready = false;
    }
    if (message != NULL) {
        printf("\n%s\n> ", message);
        fflush(stdout);
    }
}

static bool drive_arm_allowed(void)
{
    rc_channel_sample_t throttle;
    bool arm_run = false;

    if (calibration_in_progress) {
        printf("ERR: calibration is active; cancel or complete it before arming\n");
        return false;
    }
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
    printf(drive_config.permanent_arm_latch_enabled
               ? "OK: drive mode armed and latched until controller restart\n"
               : "OK: drive mode armed; STOP or receiver safety loss will disarm\n");
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
    steering_curve_sample_t geometry = steering_geometry_sample();

    torque_vectoring_input_t input = {
        .requested_mode = requested_tv_mode(),
        .direction = active_direction,
        .base_throttle = base_throttle,
        .steering = steering_normalized(),
        .vehicle_speed_mps = steering_limit_telemetry.speed_valid
                                 ? steering_limit_telemetry.vehicle_speed_mps
                                 : 0.0f,
        .average_wheel_angle_deg = geometry.valid
                                       ? geometry.average_wheel_deg
                                       : 0.0f,
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
    if (!drivetrain_apply_output_mask(drive_config.drivetrain_mode,
                                      &esc_limits,
                                      &command)) {
        torque_vectoring_reset(&vectoring_state);
        vectoring_output = (torque_vectoring_output_t){
            .inactive_reason = "invalid drivetrain mode",
        };
    }
    esc_output_set_wheels(&command);
}

static void update_drive_outputs(void)
{
    rc_channel_sample_t throttle;
    if (!throttle_sample(&throttle)) {
        if (drive_config.permanent_arm_latch_enabled) {
            set_drive_inhibit(DRIVE_INHIBIT_THROTTLE_LOST,
                              "WARN: throttle signal lost; safe outputs active");
        } else {
            return_to_disarmed("WARN: throttle signal lost; drive disarmed", true);
        }
        return;
    }
    clear_drive_inhibit(DRIVE_INHIBIT_THROTTLE_LOST);

    if (throttle_matches_failsafe()) {
        if (drive_config.permanent_arm_latch_enabled) {
            set_drive_inhibit(DRIVE_INHIBIT_THROTTLE_FAILSAFE,
                              "WARN: receiver failsafe pulse detected; safe outputs active");
        } else {
            return_to_disarmed(
                "WARN: receiver failsafe pulse detected; drive disarmed", true);
        }
        return;
    }
    clear_drive_inhibit(DRIVE_INHIBIT_THROTTLE_FAILSAFE);

    if (drive_inhibit_flags != 0) {
        hold_drive_outputs_safe(drive_inhibit_reason(drive_inhibit_flags));
        return;
    }
    if (drive_recovery_report_pending) {
        drive_recovery_report_pending = false;
        printf("\nOK: receiver controls recovered; drive remains armed\n> ");
        fflush(stdout);
    }

    drive_command_t drive_command = read_drive_command();
    if (!drive_command.moving) {
        pending_direction = active_direction;
        direction_hold_until_us = 0;
        current_base_throttle_us = ramp_toward(current_base_throttle_us,
                                               esc_limits.throttle_min_us,
                                               drivetrain_ramp_step_us(
                                                   DRIVE_DECEL_STEP_US,
                                                   drive_config.drive_smoothing_percent));
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
                                               drivetrain_ramp_step_us(
                                                   DRIVE_DECEL_STEP_US,
                                                   drive_config.drive_smoothing_percent));
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
    uint16_t smoothest_step_us =
        drive_command.throttle_us > current_base_throttle_us
            ? DRIVE_ACCEL_STEP_US
            : DRIVE_DECEL_STEP_US;
    uint16_t step = drivetrain_ramp_step_us(
        smoothest_step_us, drive_config.drive_smoothing_percent);
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

    if (system_state == SYSTEM_DRIVE_ARMED) {
        if (!arm_signal_valid) {
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
                     "safe outputs active",
                     arm_sample.pulse_us,
                     (unsigned long)rc_sample_age_ms(&arm_sample, now_us),
                     gpio_get_level(PIN_RC_ARM_INPUT),
                     (unsigned long)rc_sample_age_ms(&throttle_sample, now_us),
                     (unsigned long)rc_sample_age_ms(&steering_sample, now_us),
                     (unsigned long)rc_sample_age_ms(&tv_sample, now_us));
            if (drive_config.permanent_arm_latch_enabled) {
                set_drive_inhibit(DRIVE_INHIBIT_CH4, warning);
            } else {
                return_to_disarmed(warning, true);
            }
            return;
        }

        if (arm_unrecognized) {
            if (arm_unrecognized_confirmed) {
                char warning[192];
                snprintf(warning, sizeof(warning),
                         "WARN: receiver arm channel stayed unrecognized at %u us "
                         "(RUN=%u, STOP1=%u, STOP2=%u); safe outputs active",
                         arm_unrecognized_pulse_us, arm_cal.run_us,
                         arm_cal.stop_1_us, arm_cal.stop_2_us);
                if (drive_config.permanent_arm_latch_enabled) {
                    set_drive_inhibit(DRIVE_INHIBIT_CH4, warning);
                } else {
                    return_to_disarmed(warning, true);
                }
            }
            return;
        }

        if (!arm_run) {
            if (drive_config.permanent_arm_latch_enabled) {
                set_drive_inhibit(
                    DRIVE_INHIBIT_CH4,
                    "OK: receiver arm switch moved to STOP; safe outputs active");
            } else {
                return_to_disarmed(
                    "OK: receiver arm switch moved to STOP; drive disarmed", true);
            }
            return;
        }

        clear_drive_inhibit(DRIVE_INHIBIT_CH4);
        return;
    }

    if (!arm_signal_valid) {
        arm_run_candidate_since_us = 0;
        return;
    }

    if (arm_unrecognized) {
        arm_run_candidate_since_us = 0;
        return;
    }

    if (!arm_run) {
        arm_run_candidate_since_us = 0;
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
    if (calibration_in_progress) {
        printf("ERR: another calibration is already active\n");
        return false;
    }
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

/* Controller mutex must be held while applying any completed calibration. */
static bool apply_throttle_calibration(uint16_t neutral,
                                       uint16_t full,
                                       uint16_t reverse)
{
    if (!neutral || !full || !reverse ||
        !endpoints_opposite(full, neutral, reverse)) {
        printf("ERR: throttle endpoints must be at least 150 us from opposite sides of neutral\n");
        return false;
    }

    throttle_calibration_t previous = throttle_cal;
    throttle_cal = (throttle_calibration_t){
        .full_throttle_us = full,
        .neutral_us = neutral,
        .full_reverse_us = reverse,
        .deadband_us = 80,
        .loaded_from_nvs = true,
    };
    esp_err_t err = config_store_save_throttle(&throttle_cal);
    if (err != ESP_OK) throttle_cal = previous;
    printf(err == ESP_OK
               ? "OK: throttle calibration saved\n"
               : "ERR: failed to save throttle calibration: %s\n",
           err == ESP_OK ? "" : esp_err_to_name(err));
    return err == ESP_OK;
}

static bool apply_steering_calibration(uint16_t center,
                                       uint16_t left,
                                       uint16_t right)
{
    if (!center || !left || !right ||
        !endpoints_opposite(left, center, right)) {
        printf("ERR: steering endpoints must be at least 150 us from opposite sides of center\n");
        return false;
    }

    steering_calibration_t previous = steering_cal;
    steering_cal = (steering_calibration_t){
        .left_us = left,
        .center_us = center,
        .right_us = right,
        .deadband_us = 30,
        .loaded_from_nvs = true,
    };
    esp_err_t err = config_store_save_steering(&steering_cal);
    if (err != ESP_OK) {
        steering_cal = previous;
        printf("ERR: failed to save steering calibration: %s\n",
               esp_err_to_name(err));
        return false;
    }
    printf("OK: steering calibration saved\n");
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
    return true;
}

static bool apply_arm_calibration(uint16_t run,
                                  uint16_t stop_1,
                                  uint16_t stop_2)
{
    bool distinct = run && stop_1 && stop_2 &&
                    abs((int32_t)run - (int32_t)stop_1) >= 250 &&
                    abs((int32_t)run - (int32_t)stop_2) >= 250 &&
                    abs((int32_t)stop_1 - (int32_t)stop_2) >= 250;
    if (!distinct) {
        printf("ERR: RUN, STOP 1, and STOP 2 must each differ by at least 250 us\n");
        return false;
    }

    arm_calibration_t previous = arm_cal;
    arm_cal = (arm_calibration_t){
        .run_us = run,
        .stop_1_us = stop_1,
        .stop_2_us = stop_2,
        .loaded_from_nvs = true,
    };
    esp_err_t err = config_store_save_arm(&arm_cal);
    if (err != ESP_OK) arm_cal = previous;
    printf(err == ESP_OK
               ? "OK: receiver arm-channel calibration saved\n"
               : "ERR: failed to save arm calibration: %s\n",
           err == ESP_OK ? "" : esp_err_to_name(err));
    arm_cycle_ready = false;
    return err == ESP_OK;
}

static bool apply_tv_calibration(uint16_t off,
                                 uint16_t straight,
                                 uint16_t full)
{
    bool distinct = off && straight && full &&
                    abs((int32_t)off - (int32_t)straight) >= 150 &&
                    abs((int32_t)straight - (int32_t)full) >= 150 &&
                    abs((int32_t)off - (int32_t)full) >= 300;
    if (!distinct) {
        printf("ERR: CH5 positions are not sufficiently distinct\n");
        return false;
    }

    tv_mode_calibration_t previous = tv_mode_cal;
    tv_mode_cal = (tv_mode_calibration_t){
        .off_us = off,
        .straight_us = straight,
        .full_us = full,
        .loaded_from_nvs = true,
    };
    esp_err_t err = config_store_save_tv_mode(&tv_mode_cal);
    if (err != ESP_OK) tv_mode_cal = previous;
    printf(err == ESP_OK
               ? "OK: CH5 mode calibration saved\n"
               : "ERR: failed to save CH5 calibration: %s\n",
           err == ESP_OK ? "" : esp_err_to_name(err));
    return err == ESP_OK;
}

static const char *remote_calibration_kind_name(powertrain_calibration_kind_t kind)
{
    switch (kind) {
    case POWERTRAIN_CALIBRATION_THROTTLE: return "receiver throttle";
    case POWERTRAIN_CALIBRATION_STEERING: return "steering";
    case POWERTRAIN_CALIBRATION_ARM: return "CH4 arm/output inhibit";
    case POWERTRAIN_CALIBRATION_TV: return "CH5 torque-vectoring selector";
    case POWERTRAIN_CALIBRATION_IMU: return "IMU yaw bias";
    case POWERTRAIN_CALIBRATION_ESC: return "ESC endpoints/manual relay";
    default: return "none";
    }
}

static const char *remote_calibration_prompt(powertrain_calibration_kind_t kind,
                                             uint8_t step)
{
    static const char *const throttle_prompts[] = {
        "Hold neutral throttle, then capture",
        "Hold full forward throttle, then capture",
        "Hold full reverse throttle, then capture",
    };
    static const char *const steering_prompts[] = {
        "Hold steering centered, then capture",
        "Hold full left steering, then capture",
        "Hold full right steering, then capture",
    };
    static const char *const arm_prompts[] = {
        "Put CH4 in RUN/ON, then capture",
        "Put CH4 in the first STOP/OFF position, then capture",
        "Put CH4 in the second STOP/OFF position, then capture",
    };
    static const char *const tv_prompts[] = {
        "Put CH5 in OFF, then capture",
        "Put CH5 in STRAIGHT, then capture",
        "Put CH5 in FULL, then capture",
    };

    if (step >= 3) return "Calibration capture is complete";
    switch (kind) {
    case POWERTRAIN_CALIBRATION_THROTTLE: return throttle_prompts[step];
    case POWERTRAIN_CALIBRATION_STEERING: return steering_prompts[step];
    case POWERTRAIN_CALIBRATION_ARM: return arm_prompts[step];
    case POWERTRAIN_CALIBRATION_TV: return tv_prompts[step];
    case POWERTRAIN_CALIBRATION_IMU:
        return "Place the car level and completely still, then capture";
    default: return "No calibration is active";
    }
}

static void remote_calibration_finish(const char *message)
{
    remote_calibration.active = false;
    remote_calibration.sampling = false;
    remote_calibration.kind = POWERTRAIN_CALIBRATION_NONE;
    remote_calibration.step = 0;
    remote_calibration.total_steps = 0;
    memset(remote_calibration.values_us, 0, sizeof(remote_calibration.values_us));
    remote_calibration.prompt[0] = '\0';
    remote_calibration.last_action_us = esp_timer_get_time();
    snprintf(remote_calibration.message, sizeof(remote_calibration.message),
             "%s", message);
    calibration_in_progress = false;
}

bool powertrain_remote_calibration_start(powertrain_calibration_kind_t kind)
{
    if (kind < POWERTRAIN_CALIBRATION_THROTTLE ||
        kind > POWERTRAIN_CALIBRATION_IMU) {
        return false;
    }

    controller_lock();
    bool require_neutral = kind != POWERTRAIN_CALIBRATION_THROTTLE;
    if (!calibration_allowed(require_neutral, false)) {
        controller_unlock();
        return false;
    }
    if (kind == POWERTRAIN_CALIBRATION_IMU && !imu_initialized) {
        printf("ERR: ISM330DHCX is not available\n");
        controller_unlock();
        return false;
    }

    memset(&remote_calibration, 0, sizeof(remote_calibration));
    remote_calibration.active = true;
    remote_calibration.kind = kind;
    remote_calibration.total_steps =
        kind == POWERTRAIN_CALIBRATION_IMU ? 1U : 3U;
    remote_calibration.last_action_us = esp_timer_get_time();
    snprintf(remote_calibration.prompt, sizeof(remote_calibration.prompt), "%s",
             remote_calibration_prompt(kind, 0));
    snprintf(remote_calibration.message, sizeof(remote_calibration.message),
             "%s calibration started; follow the displayed capture steps",
             remote_calibration_kind_name(kind));
    calibration_in_progress = true;
    arm_run_candidate_since_us = 0;
    controller_unlock();
    return true;
}

bool powertrain_remote_calibration_capture(void)
{
    controller_lock();
    if (!remote_calibration.active || remote_calibration.sampling) {
        printf("ERR: no browser calibration step is ready to capture\n");
        controller_unlock();
        return false;
    }

    powertrain_calibration_kind_t kind = remote_calibration.kind;
    uint8_t step = remote_calibration.step;
    remote_calibration.sampling = true;
    remote_calibration.last_action_us = esp_timer_get_time();
    snprintf(remote_calibration.message, sizeof(remote_calibration.message),
             kind == POWERTRAIN_CALIBRATION_IMU
                 ? "Sampling IMU gyro bias for 2 seconds"
                 : "Sampling receiver PWM for 1500 ms");
    controller_unlock();

    bool imu_success = false;
    uint16_t pulse_us = 0;
    if (kind == POWERTRAIN_CALIBRATION_IMU) {
        imu_success = imu_sensor_calibrate_bias(2000);
    } else {
        rc_channel_id_t channel = kind == POWERTRAIN_CALIBRATION_THROTTLE
                                      ? RC_CHANNEL_THROTTLE
                                  : kind == POWERTRAIN_CALIBRATION_STEERING
                                      ? RC_CHANNEL_STEERING
                                  : kind == POWERTRAIN_CALIBRATION_ARM
                                      ? RC_CHANNEL_ARM
                                      : RC_CHANNEL_TV_MODE;
        pulse_us = average_channel(channel, 1500);
    }

    controller_lock();
    if (!remote_calibration.active || remote_calibration.kind != kind ||
        remote_calibration.step != step) {
        controller_unlock();
        return false;
    }
    remote_calibration.sampling = false;
    remote_calibration.last_action_us = esp_timer_get_time();

    if (kind == POWERTRAIN_CALIBRATION_IMU) {
        if (!imu_success) {
            snprintf(remote_calibration.message,
                     sizeof(remote_calibration.message),
                     "IMU calibration failed; keep the car still and level, then capture again");
            controller_unlock();
            return false;
        }
        remote_calibration_finish(
            "IMU yaw-rate bias calibrated successfully for this boot");
        controller_unlock();
        return true;
    }

    if (pulse_us == 0) {
        snprintf(remote_calibration.message, sizeof(remote_calibration.message),
                 "Receiver signal was lost during sampling; restore it and capture again");
        controller_unlock();
        return false;
    }

    remote_calibration.values_us[step] = pulse_us;
    remote_calibration.step++;
    if (remote_calibration.step < remote_calibration.total_steps) {
        snprintf(remote_calibration.prompt, sizeof(remote_calibration.prompt), "%s",
                 remote_calibration_prompt(kind, remote_calibration.step));
        snprintf(remote_calibration.message, sizeof(remote_calibration.message),
                 "Captured step %u at %u us",
                 (unsigned)(step + 1U), pulse_us);
        controller_unlock();
        return true;
    }

    bool applied = false;
    switch (kind) {
    case POWERTRAIN_CALIBRATION_THROTTLE:
        applied = apply_throttle_calibration(remote_calibration.values_us[0],
                                             remote_calibration.values_us[1],
                                             remote_calibration.values_us[2]);
        break;
    case POWERTRAIN_CALIBRATION_STEERING:
        applied = apply_steering_calibration(remote_calibration.values_us[0],
                                             remote_calibration.values_us[1],
                                             remote_calibration.values_us[2]);
        break;
    case POWERTRAIN_CALIBRATION_ARM:
        applied = apply_arm_calibration(remote_calibration.values_us[0],
                                        remote_calibration.values_us[1],
                                        remote_calibration.values_us[2]);
        break;
    case POWERTRAIN_CALIBRATION_TV:
        applied = apply_tv_calibration(remote_calibration.values_us[0],
                                       remote_calibration.values_us[1],
                                       remote_calibration.values_us[2]);
        break;
    default:
        break;
    }

    char result[POWERTRAIN_CALIBRATION_MESSAGE_MAX];
    snprintf(result, sizeof(result),
             applied ? "%s calibration saved to NVS"
                     : "%s calibration was rejected; restart it and verify every position",
             remote_calibration_kind_name(kind));
    remote_calibration_finish(result);
    controller_unlock();
    return applied;
}

void powertrain_calibrate_throttle(void)
{
    if (!begin_interactive_calibration(false, false)) return;
    printf("Receiver throttle calibration started.\n");
    uint16_t neutral = capture_channel(RC_CHANNEL_THROTTLE, "Hold neutral throttle.");
    uint16_t full = neutral ? capture_channel(RC_CHANNEL_THROTTLE, "Hold full throttle.") : 0;
    uint16_t reverse = full ? capture_channel(RC_CHANNEL_THROTTLE, "Hold full reverse.") : 0;

    controller_lock();
    if (neutral && full && reverse) {
        apply_throttle_calibration(neutral, full, reverse);
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
    if (center && left && right) apply_steering_calibration(center, left, right);
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
    if (run && stop_1 && stop_2) apply_arm_calibration(run, stop_1, stop_2);
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
    if (off && straight && full) apply_tv_calibration(off, straight, full);
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

bool powertrain_cal_esc_arm(void)
{
    controller_lock();
    if (!calibration_allowed(true, true)) {
        controller_unlock();
        return false;
    }
    system_state = SYSTEM_ESC_CAL_ARMED;
    esc_cal_last_command_us = esp_timer_get_time();
    esc_output_set_safe();
    printf("OK: ESC calibration armed. Keep ESC battery disconnected, then send 'cal esc max'.\n");
    controller_unlock();
    return true;
}

bool powertrain_cal_esc_max(void)
{
    controller_lock();
    if (system_state != SYSTEM_ESC_CAL_ARMED &&
        system_state != SYSTEM_ESC_CAL_MAX &&
        system_state != SYSTEM_ESC_CAL_MIN) {
        printf("ERR: send 'cal esc arm' before 'cal esc max'\n");
        controller_unlock();
        return false;
    }
    if (!throttle_is_neutral()) {
        printf("ERR: receiver throttle is not neutral\n");
        controller_unlock();
        return false;
    }
    if (!shutdown_channel_is_safe()) {
        printf("ERR: receiver arm switch must be in STOP for calibration\n");
        controller_unlock();
        return false;
    }
    system_state = SYSTEM_ESC_CAL_MAX;
    esc_cal_last_command_us = esp_timer_get_time();
    esc_output_set_all(esc_limits.throttle_max_us, esc_limits.reverse_low_us);
    printf("OK: maximum throttle active. Power ESCs, wait for endpoint beeps, then send 'cal esc min'.\n");
    controller_unlock();
    return true;
}

bool powertrain_cal_esc_min(void)
{
    controller_lock();
    if (system_state != SYSTEM_ESC_CAL_MAX) {
        printf("ERR: send 'cal esc max' before 'cal esc min'\n");
        controller_unlock();
        return false;
    }
    if (!throttle_is_neutral()) {
        printf("ERR: receiver throttle is not neutral\n");
        controller_unlock();
        return false;
    }
    if (!shutdown_channel_is_safe()) {
        printf("ERR: receiver arm switch must be in STOP for calibration\n");
        controller_unlock();
        return false;
    }
    system_state = SYSTEM_ESC_CAL_MIN;
    esc_cal_last_command_us = esp_timer_get_time();
    esc_output_set_all(esc_limits.throttle_min_us, esc_limits.reverse_low_us);
    printf("OK: minimum throttle active. Wait for ready beeps, then send 'cal cancel'.\n");
    controller_unlock();
    return true;
}

bool powertrain_cal_manual(void)
{
    controller_lock();
    if (!calibration_allowed(false, true)) {
        controller_unlock();
        return false;
    }
    system_state = SYSTEM_ESC_CAL_MANUAL;
    esc_cal_last_command_us = esp_timer_get_time();
    printf("OK: direct throttle relay active for 30 seconds; all reverse outputs remain low.\n");
    controller_unlock();
    return true;
}

bool powertrain_cal_cancel(void)
{
    controller_lock();
    if (system_state == SYSTEM_DRIVE_ARMED) {
        printf("ERR: drive is armed; no calibration is active\n");
        controller_unlock();
        return false;
    }
    if (calibration_in_progress && !remote_calibration.active) {
        printf("ERR: serial interactive calibration cannot be canceled from another task\n");
        controller_unlock();
        return false;
    }
    system_state = SYSTEM_DISARMED;
    remote_calibration_finish("Calibration canceled; safe outputs restored");
    arm_cycle_ready = false;
    reset_drive_state();
    esc_output_set_safe();
    printf("OK: calibration canceled; safe outputs restored\n");
    controller_unlock();
    return true;
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
    if (system_state == SYSTEM_DRIVE_ARMED &&
        drive_config.permanent_arm_latch_enabled) {
        set_drive_inhibit(DRIVE_INHIBIT_CLI,
                          "OK: CLI safe-output hold active until controller restart");
    } else {
        return_to_disarmed("OK: drive disarmed; safe outputs restored", true);
    }
    controller_unlock();
}

bool powertrain_disarm_for_configuration(void)
{
    controller_lock();
    if (calibration_in_progress ||
        (system_state != SYSTEM_DISARMED && system_state != SYSTEM_DRIVE_ARMED)) {
        printf("ERR: configuration disarm is unavailable during calibration\n");
        controller_unlock();
        return false;
    }
    if (system_state == SYSTEM_DISARMED) {
        controller_unlock();
        return true;
    }
    return_to_disarmed(
        "OK: drive disarmed for configuration; physical STOP-to-RUN cycle required",
        true);
    controller_unlock();
    return true;
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

static bool configuration_is_allowed(void)
{
    return system_state == SYSTEM_DISARMED && !calibration_in_progress;
}

bool powertrain_set_drivetrain_mode(drivetrain_mode_t mode)
{
    controller_lock();
    if (!configuration_is_allowed() || !drivetrain_mode_valid(mode)) {
        printf("ERR: drivetrain mode must be AWD, FWD, or RWD while DISARMED\n");
        controller_unlock();
        return false;
    }

    drivetrain_mode_t previous_mode = drive_config.drivetrain_mode;
    drive_config.drivetrain_mode = mode;
    torque_vectoring_reset(&vectoring_state);
    vectoring_output = (torque_vectoring_output_t){
        .inactive_reason = "drive is disarmed",
    };
    char message[64];
    snprintf(message, sizeof(message), "OK: drivetrain mode set to %s",
             drivetrain_mode_name(mode));
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    if (!saved) {
        drive_config.drivetrain_mode = previous_mode;
    }
    controller_unlock();
    return saved;
}

bool powertrain_set_reverse_limit(uint8_t percent)
{
    controller_lock();
    if (!configuration_is_allowed() || percent > 100) {
        printf("ERR: reverse limit must be 0-100 and changed while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.reverse_limit_percent = percent;
    char message[64];
    snprintf(message, sizeof(message), "OK: reverse limit set to %u%%", percent);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_drive_smoothing(uint8_t percent)
{
    controller_lock();
    if (!configuration_is_allowed() || percent > 100) {
        printf("ERR: drive smoothing must be 0-100 and changed while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.drive_smoothing_percent = percent;
    char message[96];
    if (percent == 0) {
        snprintf(message, sizeof(message),
                 "OK: acceleration/deceleration ramp disabled");
    } else {
        snprintf(message, sizeof(message),
                 "OK: drive smoothing set to %u%%", percent);
    }
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_failsafe(uint16_t pulse_us, uint16_t window_us)
{
    controller_lock();
    if (!configuration_is_allowed() || pulse_us < 800 || pulse_us > 2200 ||
        window_us < 1 || window_us > 100) {
        printf("ERR: failsafe pulse must be 800-2200 us, window 1-100 us, while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.receiver_failsafe_us = pulse_us;
    drive_config.receiver_failsafe_window_us = window_us;
    drive_config.receiver_failsafe_enabled = true;
    char message[96];
    snprintf(message, sizeof(message), "OK: throttle failsafe set to %u +/- %u us",
             pulse_us, window_us);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_failsafe_enabled(bool enabled)
{
    controller_lock();
    if (!configuration_is_allowed()) {
        printf("ERR: throttle-pulse failsafe can only be changed while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.receiver_failsafe_enabled = enabled;
    bool saved = save_drive_config_or_report(
                     enabled ? "OK: throttle-pulse failsafe enabled"
                             : "OK: throttle-pulse failsafe disabled") == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_motor_poles(uint8_t poles)
{
    controller_lock();
    if (!configuration_is_allowed() || poles < 2 || poles > 60 || (poles % 2) != 0) {
        printf("ERR: motor pole count must be even, 2-60, and changed while DISARMED\n");
        controller_unlock();
        return false;
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
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_rpm_pulses_per_revolution(uint16_t pulses_per_revolution)
{
    controller_lock();
    if (!configuration_is_allowed() || pulses_per_revolution < 1 ||
        pulses_per_revolution > 120) {
        printf("ERR: RPM pulses per mechanical revolution must be 1-120 while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.rpm_pulses_per_revolution = pulses_per_revolution;
    if (rpm_initialized) {
        rpm_sensor_set_pulses_per_revolution(pulses_per_revolution);
    }
    char message[96];
    snprintf(message, sizeof(message),
             "OK: RPM conversion set to %u pulses per mechanical revolution",
             pulses_per_revolution);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_steering_trim(float trim_degrees)
{
    controller_lock();
    if (!configuration_is_allowed() || !isfinite(trim_degrees) ||
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

bool powertrain_set_steering_smoothing(uint16_t smoothing_ms)
{
    controller_lock();
    if (!configuration_is_allowed() || smoothing_ms > 500) {
        printf("ERR: steering smoothing must be 0-500 ms while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.steering_smoothing_ms = smoothing_ms;
    char message[96];
    snprintf(message, sizeof(message), "OK: steering smoothing time constant set to %u ms",
             smoothing_ms);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_steering_speed_limit_enabled(bool enabled)
{
    controller_lock();
    if (!configuration_is_allowed()) {
        printf("ERR: steering speed limiting can only be changed while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.steering_speed_limit_enabled = enabled;
    bool saved = save_drive_config_or_report(
                     enabled ? "OK: speed-sensitive steering limit enabled"
                             : "OK: speed-sensitive steering limit disabled") == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_steering_lateral_accel(float lateral_accel_g)
{
    controller_lock();
    if (!configuration_is_allowed() || !isfinite(lateral_accel_g) ||
        lateral_accel_g < 0.2f || lateral_accel_g > 3.0f) {
        printf("ERR: steering lateral acceleration must be 0.2-3.0 g while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.steering_lateral_accel_g = lateral_accel_g;
    char message[96];
    snprintf(message, sizeof(message),
             "OK: steering lateral-acceleration ceiling set to %.2f g",
             (double)lateral_accel_g);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_tv_enabled(bool enabled)
{
    controller_lock();
    if (!configuration_is_allowed()) {
        printf("ERR: torque vectoring can only be configured while DISARMED\n");
        controller_unlock();
        return false;
    }
    if (enabled && (!steering_cal.loaded_from_nvs || !tv_mode_cal.loaded_from_nvs)) {
        printf("ERR: run 'cal steering' and 'cal tv' before enabling torque vectoring\n");
        controller_unlock();
        return false;
    }
    if (enabled) {
        if (!sensor_task_started) {
            printf("ERR: sensor task is unavailable; torque vectoring cannot be enabled\n");
            controller_unlock();
            return false;
        }
        imu_snapshot_t imu = {0};
        if (imu_initialized) {
            imu_sensor_get_snapshot(&imu);
        }
        if (!imu.valid || !imu.bias_calibrated) {
            printf("ERR: run 'cal imu' successfully before enabling torque vectoring\n");
            controller_unlock();
            return false;
        }
        for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
            if (!drivetrain_wheel_is_driven(drive_config.drivetrain_mode,
                                            (wheel_id_t)wheel)) {
                continue;
            }
            if (!rpm_seen[wheel]) {
                printf("ERR: RPM input %s has not produced a valid pulse this boot; "
                       "use 'monitor rpm' and spin each wheel\n",
                       wheel_names[wheel]);
                controller_unlock();
                return false;
            }
        }
    }
    drive_config.torque_vectoring_enabled = enabled;
    bool saved = save_drive_config_or_report(
                     enabled ? "OK: torque vectoring enabled; CH5 still selects its mode"
                             : "OK: torque vectoring disabled") == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_tv_authority(uint8_t percent)
{
    controller_lock();
    if (!configuration_is_allowed() || percent > 25) {
        printf("ERR: vectoring authority must be 0-25 percent while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.tv_authority_percent = percent;
    char message[80];
    snprintf(message, sizeof(message), "OK: vectoring authority set to +/- %u%%", percent);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_tv_front_relief(uint8_t percent)
{
    controller_lock();
    if (!configuration_is_allowed() || percent > 50) {
        printf("ERR: front torque relief must be 0-50 percent while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.tv_front_relief_percent = percent;
    char message[96];
    snprintf(message, sizeof(message),
             "OK: FULL-mode front torque relief set to %u%%", percent);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_tv_gains(float yaw_gain_dps,
                             float turn_rpm_gain,
                             float yaw_kp,
                             float yaw_ki,
                             float rpm_kp)
{
    controller_lock();
    if (!configuration_is_allowed() ||
        !isfinite(yaw_gain_dps) || !isfinite(turn_rpm_gain) ||
        !isfinite(yaw_kp) || !isfinite(yaw_ki) || !isfinite(rpm_kp) ||
        yaw_gain_dps < 0.0f || yaw_gain_dps > 500.0f ||
        turn_rpm_gain < 0.0f || turn_rpm_gain > 1.0f ||
        yaw_kp < 0.0f || yaw_kp > 0.01f ||
        yaw_ki < 0.0f || yaw_ki > 0.01f ||
        rpm_kp < 0.0f || rpm_kp > 5.0f) {
        printf("ERR: invalid TV gains or controller is not DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.tv_turn_yaw_gain_dps = yaw_gain_dps;
    drive_config.tv_turn_rpm_gain = turn_rpm_gain;
    drive_config.tv_yaw_kp = yaw_kp;
    drive_config.tv_yaw_ki = yaw_ki;
    drive_config.tv_rpm_kp = rpm_kp;
    bool saved = save_drive_config_or_report("OK: torque-vectoring gains saved") == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_imu_yaw_sign(int8_t sign)
{
    controller_lock();
    if (!configuration_is_allowed() || (sign != -1 && sign != 1)) {
        printf("ERR: IMU yaw sign must be -1 or 1 while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.imu_yaw_sign = sign;
    if (imu_initialized) imu_sensor_set_yaw_sign(sign);
    bool saved = save_drive_config_or_report("OK: IMU yaw sign saved") == ESP_OK;
    controller_unlock();
    return saved;
}

bool powertrain_set_permanent_arm_latch_enabled(bool enabled)
{
    controller_lock();
    if (!configuration_is_allowed()) {
        printf("ERR: permanent arm latch can only be changed while DISARMED\n");
        controller_unlock();
        return false;
    }
    bool previous = drive_config.permanent_arm_latch_enabled;
    drive_config.permanent_arm_latch_enabled = enabled;
    bool saved = save_drive_config_or_report(
                     enabled
                         ? "OK: permanent arm latch enabled"
                         : "OK: permanent arm latch disabled; safety events will disarm") ==
                 ESP_OK;
    if (!saved) {
        drive_config.permanent_arm_latch_enabled = previous;
    }
    controller_unlock();
    return saved;
}

bool powertrain_set_logging_rate_hz(uint8_t rate_hz)
{
    controller_lock();
    if (!configuration_is_allowed() || rate_hz < 1 || rate_hz > 50) {
        printf("ERR: logging rate must be 1-50 Hz while DISARMED\n");
        controller_unlock();
        return false;
    }
    drive_config.telemetry_log_rate_hz = rate_hz;
    char message[80];
    snprintf(message, sizeof(message), "OK: telemetry logging rate set to %u Hz",
             rate_hz);
    bool saved = save_drive_config_or_report(message) == ESP_OK;
    controller_unlock();
    return saved;
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

bool powertrain_get_remote_snapshot(powertrain_remote_snapshot_t *snapshot)
{
    if (snapshot == NULL || controller_mutex == NULL) {
        return false;
    }

    powertrain_remote_snapshot_t result = {0};
    controller_lock();
    result.captured_at_us = esp_timer_get_time();
    result.state = system_state;
    result.arm_cycle_ready = arm_cycle_ready;
    result.drive_inhibit_flags = drive_inhibit_flags;
    result.drive_inhibit_count = drive_inhibit_count;
    bool esc_calibration_active = system_state == SYSTEM_ESC_CAL_ARMED ||
                                  system_state == SYSTEM_ESC_CAL_MAX ||
                                  system_state == SYSTEM_ESC_CAL_MIN ||
                                  system_state == SYSTEM_ESC_CAL_MANUAL;
    result.calibration_active = remote_calibration.active ||
                                calibration_in_progress ||
                                esc_calibration_active;
    result.calibration_sampling = remote_calibration.sampling;
    result.calibration_kind = remote_calibration.kind;
    result.calibration_step = remote_calibration.step;
    result.calibration_total_steps = remote_calibration.total_steps;
    memcpy(result.calibration_values_us, remote_calibration.values_us,
           sizeof(result.calibration_values_us));
    snprintf(result.calibration_prompt, sizeof(result.calibration_prompt), "%s",
             remote_calibration.prompt);
    snprintf(result.calibration_message, sizeof(result.calibration_message), "%s",
             remote_calibration.message);
    if (esc_calibration_active) {
        result.calibration_kind = POWERTRAIN_CALIBRATION_ESC;
        result.calibration_sampling = false;
        result.calibration_total_steps =
            system_state == SYSTEM_ESC_CAL_MANUAL ? 1U : 3U;
        result.calibration_step = system_state == SYSTEM_ESC_CAL_ARMED ? 0U
                                  : system_state == SYSTEM_ESC_CAL_MAX ? 1U
                                  : system_state == SYSTEM_ESC_CAL_MIN ? 2U
                                                                       : 0U;
        const char *prompt = system_state == SYSTEM_ESC_CAL_ARMED
                                 ? "Keep traction power disconnected, then select ESC maximum"
                             : system_state == SYSTEM_ESC_CAL_MAX
                                 ? "After maximum-endpoint beeps, select ESC minimum"
                             : system_state == SYSTEM_ESC_CAL_MIN
                                 ? "After ready beeps, cancel to restore safe output"
                                 : "Manual receiver-throttle relay is active for at most 30 seconds";
        snprintf(result.calibration_prompt, sizeof(result.calibration_prompt),
                 "%s", prompt);
        snprintf(result.calibration_message, sizeof(result.calibration_message),
                 "ESC calibration/manual output mode is active");
    } else if (calibration_in_progress && !remote_calibration.active) {
        snprintf(result.calibration_message, sizeof(result.calibration_message),
                 "A USB serial interactive calibration is active");
    }
    rc_input_get(RC_CHANNEL_THROTTLE, &result.throttle_input);
    rc_input_get(RC_CHANNEL_STEERING, &result.steering_input);
    rc_input_get(RC_CHANNEL_ARM, &result.arm_input);
    rc_input_get(RC_CHANNEL_TV_MODE, &result.tv_input);
    result.steering_servo_us = servo_output_get_pulse();
    result.steering_filter_us = steering_input_filter.output_us;
    result.steering_filter_active =
        steering_input_filter_status(&steering_input_filter) ==
        STEERING_INPUT_FILTER_ACTIVE;
    result.steering_rejected_spikes = steering_input_filter.rejected_spike_count;
    result.vehicle_speed_mps = steering_limit_telemetry.vehicle_speed_mps;
    result.vehicle_speed_valid = steering_limit_telemetry.speed_valid;
    result.steering_limited = steering_limit_telemetry.steering_limited;
    result.steering_requested_deg =
        steering_limit_telemetry.requested_average_wheel_deg;
    result.steering_maximum_deg = steering_limit_telemetry.maximum_average_wheel_deg;

    steering_curve_sample_t steering_geometry = steering_geometry_sample();
    result.steering_servo_command_deg = steering_geometry.servo_command_deg;
    result.steering_left_wheel_deg = steering_geometry.left_wheel_deg;
    result.steering_right_wheel_deg = steering_geometry.right_wheel_deg;
    result.steering_average_wheel_deg = steering_geometry.average_wheel_deg;
    result.config = drive_config;
    esc_output_get(&result.outputs);
    if (rpm_initialized) {
        rpm_sensor_get_snapshot(&result.rpm);
    }
    if (imu_initialized) {
        imu_sensor_get_snapshot(&result.imu);
    }
    result.requested_tv_mode = requested_tv_mode();
    result.active_tv_mode = vectoring_output.active_mode;
    result.vectoring_active = vectoring_output.active;
    result.target_yaw_rate_dps = vectoring_output.target_yaw_rate_dps;
    result.yaw_error_dps = vectoring_output.yaw_error_dps;
    result.side_rpm_error = vectoring_output.side_rpm_error;
    result.predicted_lateral_accel_mps2 =
        vectoring_output.predicted_lateral_accel_mps2;
    result.lateral_demand = vectoring_output.lateral_demand;
    result.front_relief = vectoring_output.front_relief;
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        result.wheel_correction[wheel] = vectoring_output.wheel_correction[wheel];
    }
    snprintf(result.vectoring_reason, sizeof(result.vectoring_reason), "%s",
             vectoring_output.inactive_reason != NULL
                 ? vectoring_output.inactive_reason
                 : "unavailable");
    controller_unlock();

    *snapshot = result;
    return true;
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
    uint32_t drive_inhibit_snapshot;
    uint32_t drive_inhibit_count_snapshot;
    const char *last_drive_inhibit_reason_snapshot;
    uint16_t servo_pulse_us;
    steering_curve_sample_t steering_geometry;
    tv_mode_t requested_mode;
    bool arm_run = false;
    bool arm_valid;
    steering_input_filter_status_t steering_filter_status;
    uint16_t steering_filter_output_us;
    uint32_t steering_rejected_spikes;
    steering_limit_telemetry_t steering_limit_snapshot;

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
    drive_inhibit_snapshot = drive_inhibit_flags;
    drive_inhibit_count_snapshot = drive_inhibit_count;
    last_drive_inhibit_reason_snapshot = last_drive_inhibit_reason;
    servo_pulse_us = servo_output_get_pulse();
    steering_geometry = steering_geometry_sample();
    requested_mode = requested_tv_mode();
    arm_valid = arm_channel_state(&arm_run);
    steering_filter_status = steering_input_filter_status(&steering_input_filter);
    steering_filter_output_us = steering_input_filter.output_us;
    steering_rejected_spikes = steering_input_filter.rejected_spike_count;
    steering_limit_snapshot = steering_limit_telemetry;
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
    if (state_snapshot == SYSTEM_DRIVE_ARMED) {
        printf("  state: %s, arm policy: %s, drive outputs: %s",
               state_name(state_snapshot),
               config_snapshot.permanent_arm_latch_enabled
                   ? "latched until restart"
                   : "STOP-to-disarm",
               drive_inhibit_snapshot == 0 ? "active" : "safe/inhibited");
        if (drive_inhibit_snapshot != 0) {
            printf(" (%s)", drive_inhibit_reason(drive_inhibit_snapshot));
        }
        printf("\n");
        printf("  drive inhibit events: %lu, last=%s\n",
               (unsigned long)drive_inhibit_count_snapshot,
               last_drive_inhibit_reason_snapshot);
    } else {
        printf("  state: %s, initial arm cycle: %s\n",
               state_name(state_snapshot), arm_cycle_snapshot ? "ready" : "required");
    }
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
    printf("  drive: mode=%s, reverse_limit=%u%%, smoothing=%u%%, "
           "motor=%u poles, RPM PPR=%u\n",
           drivetrain_mode_name(config_snapshot.drivetrain_mode),
           config_snapshot.reverse_limit_percent,
           config_snapshot.drive_smoothing_percent, config_snapshot.motor_poles,
           config_snapshot.rpm_pulses_per_revolution);
    if (config_snapshot.drive_smoothing_percent == 0) {
        printf("  drive ramp: disabled; throttle changes apply immediately\n");
    } else {
        printf("  drive ramp: acceleration=%u us/tick, deceleration=%u us/tick\n",
               drivetrain_ramp_step_us(
                   DRIVE_ACCEL_STEP_US,
                   config_snapshot.drive_smoothing_percent),
               drivetrain_ramp_step_us(
                   DRIVE_DECEL_STEP_US,
                   config_snapshot.drive_smoothing_percent));
    }
    printf("  geometry: wheel=%.0f mm wheelbase=%.0f mm track=%.0f mm\n",
           (double)(DEFAULT_WHEEL_DIAMETER_M * 1000.0f),
           (double)(DEFAULT_WHEELBASE_M * 1000.0f),
           (double)(DEFAULT_TRACK_WIDTH_M * 1000.0f));
    printf("  steering config: trim=%+.1f command degrees, smoothing=%u ms\n",
           (double)config_snapshot.steering_trim_tenths_deg / 10.0,
           config_snapshot.steering_smoothing_ms);
    printf("  telemetry logging: %u Hz\n", config_snapshot.telemetry_log_rate_hz);
    printf("  permanent arm latch: %s\n",
           config_snapshot.permanent_arm_latch_enabled ? "enabled" : "disabled");
    printf("  steering speed limit: %s, ceiling=%.2f g, speed=%.1f km/h%s, "
           "requested/maximum average angle=%+.2f/%.2f deg%s\n",
           config_snapshot.steering_speed_limit_enabled ? "enabled" : "disabled",
           (double)config_snapshot.steering_lateral_accel_g,
           (double)(steering_limit_snapshot.vehicle_speed_mps * 3.6f),
           steering_limit_snapshot.speed_valid ? "" : " unavailable",
           (double)steering_limit_snapshot.requested_average_wheel_deg,
           (double)steering_limit_snapshot.maximum_average_wheel_deg,
           steering_limit_snapshot.steering_limited ? " LIMITED" : "");
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
    printf("  TV: configured=%s, CH5=%s, active=%s, authority=+/- %u%%, "
           "front_relief_max=%u%% current=%.1f%%, reason=%s\n",
           config_snapshot.torque_vectoring_enabled ? "enabled" : "disabled",
           tv_mode_name(requested_mode),
           vector_snapshot.active ? tv_mode_name(vector_snapshot.active_mode) : "no",
           config_snapshot.tv_authority_percent,
           config_snapshot.tv_front_relief_percent,
           (double)(vector_snapshot.front_relief * 100.0f),
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
        steering_limit_telemetry_t limit = steering_limit_telemetry;
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
                   "speed=%.1f km/h%s max=%.2f deg limited=%s "
                   "filter=%s rejected=%lu%s%s\n",
                   sample.pulse_us, median_text, servo_output_get_pulse(),
                   geometry.servo_command_deg,
                   geometry.left_wheel_deg,
                   geometry.right_wheel_deg,
                   geometry.average_wheel_deg,
                   geometry.normalized_average,
                   (double)(limit.vehicle_speed_mps * 3.6f),
                   limit.speed_valid ? "" : " unavailable",
                   (double)limit.maximum_average_wheel_deg,
                   limit.steering_limited ? "yes" : "no",
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
            bool drive_armed = system_state == SYSTEM_DRIVE_ARMED;
            bool permanent_latch = drive_config.permanent_arm_latch_enabled;
            uint32_t inhibit_flags = drive_inhibit_flags;
            controller_unlock();
            printf("arm: %u us, position=%s, request=%s, "
                   "armed=%s, policy=%s, outputs=%s, initial_cycle=%s\n",
                   sample.pulse_us,
                   arm_position_name(sample.pulse_us),
                   arm_requests_run(sample.pulse_us) ? "RUN" : "STOP",
                   drive_armed ? "yes" : "no",
                   permanent_latch ? "permanent" : "STOP-to-disarm",
                   inhibit_flags == 0 ? "active" : drive_inhibit_reason(inhibit_flags),
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
    printf("Monitoring IMU for 30 seconds; monitoring does not start calibration.\n");
    while (esp_timer_get_time() < end_at_us) {
        imu_snapshot_t imu = {0};
        if (imu_initialized) imu_sensor_get_snapshot(&imu);
        printf("IMU: %s bias=%s yaw=%+.2f dps gyro=[%+.2f %+.2f %+.2f] "
               "accel=[%+.2f %+.2f %+.2f]\n",
               imu.valid ? "valid" : "missing",
               imu.bias_calibrated ? "calibrated" : "not-calibrated",
               imu.yaw_rate_dps,
               imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2],
               imu.accel_mps2[0], imu.accel_mps2[1], imu.accel_mps2[2]);
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_gps(void)
{
    int64_t end_at_us = esp_timer_get_time() + MONITOR_DURATION_US;
    printf("Monitoring Dragy GPS for 30 seconds. Course is GPS course over ground.\n");
    while (esp_timer_get_time() < end_at_us) {
        dragy_sensor_snapshot_t gps = {0};
        dragy_sensor_get_snapshot(&gps);
        int64_t now_us = esp_timer_get_time();
        int64_t age_ms = gps.updated_at_us > 0 && now_us >= gps.updated_at_us
                             ? (now_us - gps.updated_at_us) / 1000
                             : -1;
        printf("GPS: uart=%s baud=%lu bytes=%lu nmea=%s fix=%s age=%lld ms "
               "lat=%.7f lon=%.7f speed=%.2f km/h course=%.2f deg "
               "alt=%.2f m sats=%u hdop=%.2f sentences=%lu checksum=%lu parse=%lu\n",
               gps.initialized ? "ready" : "unavailable",
               (unsigned long)gps.baud_rate,
               (unsigned long)gps.uart_bytes,
               gps.nmea_recent ? "recent" : "stale",
               gps.fix_valid ? "valid" : "invalid",
               (long long)age_ms,
               gps.latitude_deg,
               gps.longitude_deg,
               gps.speed_mps * 3.6f,
               gps.course_deg,
               gps.altitude_m,
               gps.satellites,
               gps.hdop,
               (unsigned long)gps.valid_sentence_count,
               (unsigned long)gps.checksum_error_count,
               (unsigned long)gps.parse_error_count);
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

void powertrain_monitor_gps_raw(void)
{
    dragy_sensor_snapshot_t gps = {0};
    dragy_sensor_get_snapshot(&gps);
    if (!gps.initialized) {
        printf("ERR: Dragy UART is unavailable\n");
        return;
    }

    printf("Capturing raw Dragy UART for 5 seconds at %lu baud; "
           "normal GPS parsing continues.\n",
           (unsigned long)gps.baud_rate);
    dragy_sensor_reset_raw_capture();
    vTaskDelay(pdMS_TO_TICKS(5000));

    dragy_raw_capture_t capture = {0};
    dragy_sensor_get_raw_capture(&capture);
    printf("Raw Dragy UART: captured=%u received=%lu overwritten=%lu\n",
           (unsigned)capture.length,
           (unsigned long)capture.total_received,
           (unsigned long)capture.overwritten);
    if (capture.length == 0) {
        printf("No UART bytes arrived during the capture window.\n");
        return;
    }

    for (size_t offset = 0; offset < capture.length; offset += 16U) {
        size_t line_length = capture.length - offset;
        if (line_length > 16U) line_length = 16U;

        printf("%04x  ", (unsigned)offset);
        for (size_t column = 0; column < 16U; column++) {
            if (column < line_length) {
                printf("%02x ", capture.bytes[offset + column]);
            } else {
                printf("   ");
            }
            if (column == 7U) printf(" ");
        }
        printf(" |");
        for (size_t column = 0; column < line_length; column++) {
            uint8_t byte = capture.bytes[offset + column];
            putchar(byte >= 0x20U && byte <= 0x7eU ? (int)byte : '.');
        }
        printf("|\n");
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
               "lateral=%.2f m/s2 demand=%.2f front_relief=%.3f "
               "correction=[%+.3f %+.3f %+.3f %+.3f] reason=%s\n",
               tv_mode_name(mode),
               output.active ? "yes" : "no",
               geometry.servo_command_deg,
               geometry.left_wheel_deg,
               geometry.right_wheel_deg,
               output.target_yaw_rate_dps,
               output.yaw_error_dps,
               output.side_rpm_error,
               output.predicted_lateral_accel_mps2,
               output.lateral_demand,
               output.front_relief,
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
    printf("    disarm holds safe outputs until restart only when arm-latch is on\n");
    printf("    disarm config clears the latch and requires a physical rearm cycle\n");
    printf("  cal receiver | cal steering | cal arm | cal tv | cal imu\n");
    printf("  cal esc arm | cal esc max | cal esc min | cal manual | cal cancel\n");
    printf("  monitor throttle | steering | arm | tv | rpm | imu | gps | vector\n");
    printf("  monitor gps raw\n");
    printf("  monitor steering trim <-15..15>\n");
    printf("  config drivetrain <awd|fwd|rwd>\n");
    printf("  config reverse <0-100>\n");
    printf("  config drive smoothing <0-100> (0=off, 100=smoothest)\n");
    printf("  config failsafe off | <pulse_us> <window_us>\n");
    printf("  config rpm poles <even 2-60>\n");
    printf("  config rpm ppr <1-120>\n");
    printf("  config steering trim <-15..15>\n");
    printf("  config steering smoothing <0-500>\n");
    printf("  config steering speed-limit <on|off>\n");
    printf("  config steering lateral-g <0.2-3.0>\n");
    printf("  config tv authority <0-25>\n");
    printf("  config tv front-relief <0-50>\n");
    printf("  config tv gains <yaw_gain_dps> <turn_rpm_gain> <yaw_kp> <yaw_ki> <rpm_kp>\n");
    printf("  config imu yaw-sign <-1|1>\n");
    printf("  config arm-latch <on|off>\n");
    printf("  config logging rate <1-50>\n");
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
        int64_t now_us = esp_timer_get_time();
        if (remote_calibration.active && !remote_calibration.sampling &&
            now_us - remote_calibration.last_action_us > REMOTE_CAL_TIMEOUT_US) {
            remote_calibration_finish(
                "Browser calibration timed out after 120 seconds; safe DISARMED state retained");
            arm_cycle_ready = false;
            reset_drive_state();
            esc_output_set_safe();
            printf("\nWARN: browser calibration timed out; safe outputs retained\n> ");
        }
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
