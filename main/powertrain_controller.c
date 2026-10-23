#include "powertrain_internal.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "config_store.h"
#include "default_config.h"
#include "driver/gpio.h"
#include "drivetrain_control.h"
#include "esc_output.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pin_config.h"
#include "powertrain_types.h"
#include "rc_input.h"
#include "rpm_sensor.h"
#include "servo_output.h"
#include "independent_steering.h"

#define ESC_THROTTLE_MIN_US 1100
#define ESC_THROTTLE_MAX_US 1940
#define ESC_REVERSE_LOW_US 1100
#define ESC_REVERSE_HIGH_US 1940
#define SENSOR_UPDATE_INTERVAL_MS 20
#define DIRECTION_CHANGE_HOLD_US 120000
#define DRIVE_ACCEL_STEP_US 12
#define DRIVE_DECEL_STEP_US 100
#define ARM_UNRECOGNIZED_CONFIRM_US 60000
#define ARM_RUN_STABLE_US 250000

_Static_assert(pdMS_TO_TICKS(SENSOR_UPDATE_INTERVAL_MS) > 0,
               "SENSOR_UPDATE_INTERVAL_MS must convert to at least one FreeRTOS tick");

typedef struct {
    uint16_t throttle_us;
    drive_direction_t direction;
    bool moving;
} drive_command_t;

static const char *TAG = "powertrain";

powertrain_context_t controller = {.system_state = SYSTEM_DISARMED};

const esc_limits_t esc_limits = {
    .throttle_min_us = ESC_THROTTLE_MIN_US,
    .throttle_max_us = ESC_THROTTLE_MAX_US,
    .reverse_low_us = ESC_REVERSE_LOW_US,
    .reverse_high_us = ESC_REVERSE_HIGH_US,
};

static uint16_t current_base_throttle_us = ESC_THROTTLE_MIN_US;
static drive_direction_t active_direction = DRIVE_DIRECTION_FORWARD;
static drive_direction_t pending_direction = DRIVE_DIRECTION_FORWARD;
static int64_t direction_hold_until_us;
static bool drive_recovery_report_pending;
static int64_t arm_unrecognized_since_us;
static uint16_t arm_unrecognized_pulse_us;
static uint32_t sensor_overrun_count;
static uint32_t powertrain_overrun_count;
static SemaphoreHandle_t controller_mutex;
static portMUX_TYPE telemetry_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t sensor_max_execution_us;
static uint32_t powertrain_max_execution_us;
static UBaseType_t sensor_stack_min_free_bytes;
static UBaseType_t powertrain_stack_min_free_bytes;

#define DRIVE_INHIBIT_CH4 (1U << 0)
#define DRIVE_INHIBIT_THROTTLE_LOST (1U << 1)
#define DRIVE_INHIBIT_THROTTLE_FAILSAFE (1U << 2)
#define DRIVE_INHIBIT_CLI (1U << 3)

void controller_lock(void)
{
    configASSERT(controller_mutex != NULL);
    configASSERT(xSemaphoreTakeRecursive(controller_mutex, portMAX_DELAY) == pdTRUE);
}

void controller_unlock(void)
{
    configASSERT(xSemaphoreGiveRecursive(controller_mutex) == pdTRUE);
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

bool pulse_is_on_endpoint_side(uint16_t pulse_us,
                               uint16_t endpoint_us,
                               uint16_t center_us)
{
    int32_t pulse_delta = (int32_t)pulse_us - center_us;
    int32_t endpoint_delta = (int32_t)endpoint_us - center_us;
    return pulse_delta != 0 && endpoint_delta != 0 &&
           ((pulse_delta > 0) == (endpoint_delta > 0));
}

bool throttle_sample(rc_channel_sample_t *sample)
{
    return rc_input_get(RC_CHANNEL_THROTTLE, sample);
}

bool throttle_is_neutral(void)
{
    rc_channel_sample_t sample;
    if (!throttle_sample(&sample)) {
        return false;
    }
    return abs((int32_t)sample.pulse_us - (int32_t)controller.throttle_cal.neutral_us) <=
           controller.throttle_cal.deadband_us;
}

bool throttle_matches_failsafe(void)
{
    if (!controller.drive_config.receiver_failsafe_enabled) {
        return false;
    }

    rc_channel_sample_t sample;
    if (!throttle_sample(&sample)) {
        return false;
    }

    return abs((int32_t)sample.pulse_us -
               (int32_t)controller.drive_config.receiver_failsafe_us) <=
           controller.drive_config.receiver_failsafe_window_us;
}

typedef enum {
    ARM_POSITION_UNKNOWN = 0,
    ARM_POSITION_RUN,
    ARM_POSITION_STOP_1,
    ARM_POSITION_STOP_2,
} arm_position_t;

static arm_position_t arm_position(uint16_t pulse_us)
{
    unsigned int run_distance = (unsigned int)abs((int32_t)pulse_us - (int32_t)controller.arm_cal.run_us);
    unsigned int stop_1_distance = (unsigned int)abs((int32_t)pulse_us - (int32_t)controller.arm_cal.stop_1_us);
    unsigned int stop_2_distance = (unsigned int)abs((int32_t)pulse_us - (int32_t)controller.arm_cal.stop_2_us);

    if (stop_1_distance <= DEFAULT_RC_ARM_POSITION_WINDOW_US) return ARM_POSITION_STOP_1;
    if (stop_2_distance <= DEFAULT_RC_ARM_POSITION_WINDOW_US) return ARM_POSITION_STOP_2;
    if (run_distance <= DEFAULT_RC_ARM_POSITION_WINDOW_US) return ARM_POSITION_RUN;
    return ARM_POSITION_UNKNOWN;
}

const char *arm_position_name(uint16_t pulse_us)
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

bool arm_requests_run(uint16_t pulse_us)
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

uint32_t rc_sample_age_ms(const rc_channel_sample_t *sample, int64_t now_us)
{
    if (sample->updated_at_us <= 0 || now_us <= sample->updated_at_us) {
        return 0;
    }

    int64_t age_ms = (now_us - sample->updated_at_us) / 1000;
    return age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
}

bool shutdown_channel_is_safe(void)
{
    if (!controller.arm_cal.loaded_from_nvs) {
        return false;
    }

    bool arm_run = false;
    return arm_channel_state(&arm_run) && !arm_run;
}

const char *drive_inhibit_reason(uint32_t flags)
{
    if ((flags & DRIVE_INHIBIT_CLI) != 0) return "CLI safe-output hold";
    if ((flags & DRIVE_INHIBIT_CH4) != 0) return "CH4 is not valid RUN";
    if ((flags & DRIVE_INHIBIT_THROTTLE_LOST) != 0) return "throttle signal missing";
    if ((flags & DRIVE_INHIBIT_THROTTLE_FAILSAFE) != 0) {
        return "configured throttle failsafe pulse";
    }
    return "none";
}

static void hold_drive_outputs_safe(void)
{
    current_base_throttle_us = esc_limits.throttle_min_us;
    active_direction = DRIVE_DIRECTION_FORWARD;
    pending_direction = DRIVE_DIRECTION_FORWARD;
    direction_hold_until_us = 0;
    esc_output_set_safe();
}

static void set_drive_inhibit(uint32_t flag, const char *message)
{
    bool newly_inhibited = (controller.drive_inhibit_flags & flag) == 0;
    controller.drive_inhibit_flags |= flag;
    hold_drive_outputs_safe();
    if (newly_inhibited) {
        controller.drive_inhibit_count++;
    }
    drive_recovery_report_pending = false;
    if (newly_inhibited && message != NULL) {
        printf("\n%s; armed latch retained\n> ", message);
        fflush(stdout);
    }
}

static void clear_drive_inhibit(uint32_t flag)
{
    bool was_inhibited = controller.drive_inhibit_flags != 0;
    controller.drive_inhibit_flags &= ~flag;
    if (controller.drive_inhibit_flags != 0) {
        hold_drive_outputs_safe();
    } else if (was_inhibited && controller.system_state == SYSTEM_DRIVE_ARMED) {
        drive_recovery_report_pending = true;
    }
}

void reset_drive_state(void)
{
    current_base_throttle_us = esc_limits.throttle_min_us;
    active_direction = DRIVE_DIRECTION_FORWARD;
    pending_direction = DRIVE_DIRECTION_FORWARD;
    direction_hold_until_us = 0;
    controller.arm_run_candidate_since_us = 0;
    controller.drive_inhibit_flags = 0;
    controller.drive_inhibit_count = 0;
    drive_recovery_report_pending = false;
}

static void return_to_disarmed(const char *message, bool require_arm_cycle)
{
    controller.system_state = SYSTEM_DISARMED;
    reset_drive_state();
    esc_output_set_safe();
    if (require_arm_cycle) {
        controller.arm_cycle_ready = false;
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

    if (controller.calibration_in_progress) {
        printf("ERR: calibration is active; cancel or complete it before arming\n");
        return false;
    }
    if (!controller.throttle_cal.loaded_from_nvs) {
        printf("ERR: run 'cal receiver' before arming\n");
        return false;
    }
    if (!controller.arm_cal.loaded_from_nvs) {
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
               throttle.pulse_us, controller.throttle_cal.neutral_us, controller.throttle_cal.deadband_us);
        return false;
    }
    return true;
}

static void arm_drive(void)
{
    reset_drive_state();
    esc_output_set_safe();
    controller.system_state = SYSTEM_DRIVE_ARMED;
    controller.arm_cycle_ready = false;
    printf(controller.drive_config.permanent_arm_latch_enabled
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

    int32_t neutral_delta = (int32_t)sample.pulse_us - controller.throttle_cal.neutral_us;
    if (abs(neutral_delta) <= controller.throttle_cal.deadband_us) {
        return command;
    }

    uint16_t endpoint = 0;
    if (pulse_is_on_endpoint_side(sample.pulse_us,
                                  controller.throttle_cal.full_throttle_us,
                                  controller.throttle_cal.neutral_us)) {
        endpoint = controller.throttle_cal.full_throttle_us;
        command.direction = DRIVE_DIRECTION_FORWARD;
    } else if (pulse_is_on_endpoint_side(sample.pulse_us,
                                         controller.throttle_cal.full_reverse_us,
                                         controller.throttle_cal.neutral_us)) {
        endpoint = controller.throttle_cal.full_reverse_us;
        command.direction = DRIVE_DIRECTION_REVERSE;
    } else {
        return command;
    }

    uint32_t input_span = abs((int32_t)endpoint - controller.throttle_cal.neutral_us);
    uint32_t input_delta = abs((int32_t)sample.pulse_us - controller.throttle_cal.neutral_us);
    if (input_span == 0) {
        return command;
    }
    if (input_delta > input_span) {
        input_delta = input_span;
    }

    float normalized = (float)input_delta / input_span;
    normalized *= normalized;
    if (command.direction == DRIVE_DIRECTION_REVERSE) {
        normalized *= (float)controller.drive_config.reverse_limit_percent / 100.0f;
    }

    uint16_t output_span = esc_limits.throttle_max_us - esc_limits.throttle_min_us;
    command.throttle_us = esc_limits.throttle_min_us +
                          (uint16_t)lroundf(normalized * output_span);
    command.moving = command.throttle_us > esc_limits.throttle_min_us;
    return command;
}

static void set_drive_outputs(void)
{
    wheel_output_command_t command;
    for (size_t wheel = 0; wheel < POWERTRAIN_WHEEL_COUNT; wheel++) {
        command.throttle_us[wheel] = current_base_throttle_us;
        command.reverse_us[wheel] = reverse_pulse_for_direction(active_direction);
    }
    drivetrain_apply_output_mask(controller.drive_config.drivetrain_mode, &esc_limits, &command);
    esc_output_set_wheels(&command);
}

static void update_drive_outputs(void)
{
    rc_channel_sample_t throttle;
    if (!throttle_sample(&throttle)) {
        if (controller.drive_config.permanent_arm_latch_enabled) {
            set_drive_inhibit(DRIVE_INHIBIT_THROTTLE_LOST,
                              "WARN: throttle signal lost; safe outputs active");
        } else {
            return_to_disarmed("WARN: throttle signal lost; drive disarmed", true);
        }
        return;
    }
    clear_drive_inhibit(DRIVE_INHIBIT_THROTTLE_LOST);

    if (throttle_matches_failsafe()) {
        if (controller.drive_config.permanent_arm_latch_enabled) {
            set_drive_inhibit(DRIVE_INHIBIT_THROTTLE_FAILSAFE,
                              "WARN: receiver failsafe pulse detected; safe outputs active");
        } else {
            return_to_disarmed(
                "WARN: receiver failsafe pulse detected; drive disarmed", true);
        }
        return;
    }
    clear_drive_inhibit(DRIVE_INHIBIT_THROTTLE_FAILSAFE);

    if (controller.drive_inhibit_flags != 0) {
        hold_drive_outputs_safe();
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
                                                   controller.drive_config.drive_smoothing_percent));
        set_drive_outputs();
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
                                                   controller.drive_config.drive_smoothing_percent));
        set_drive_outputs();
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
        smoothest_step_us, controller.drive_config.drive_smoothing_percent);
    current_base_throttle_us = ramp_toward(current_base_throttle_us,
                                           drive_command.throttle_us,
                                           step);
    set_drive_outputs();
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
    bool esc_calibration_active = controller.system_state == SYSTEM_ESC_CAL_ARMED ||
                                  controller.system_state == SYSTEM_ESC_CAL_MAX ||
                                  controller.system_state == SYSTEM_ESC_CAL_MIN ||
                                  controller.system_state == SYSTEM_ESC_CAL_MANUAL;

    if (esc_calibration_active && controller.arm_cal.loaded_from_nvs &&
        (!arm_signal_valid || arm_run || arm_unrecognized)) {
        powertrain_cal_cancel();
        printf("\nWARN: ESC calibration stopped because the receiver shutdown "
               "channel is not in STOP\n> ");
        fflush(stdout);
        controller.arm_run_candidate_since_us = 0;
        return;
    }

    if (controller.system_state == SYSTEM_DRIVE_ARMED) {
        if (!arm_signal_valid) {
            rc_channel_sample_t throttle_sample = {0};
            rc_channel_sample_t steering_sample = {0};
            rc_input_get(RC_CHANNEL_THROTTLE, &throttle_sample);
            rc_input_get(RC_CHANNEL_STEERING, &steering_sample);

            char warning[256];
            snprintf(warning, sizeof(warning),
                     "WARN: receiver arm signal lost; CH4 last=%u us age=%lu ms "
                     "level=%d, CH2 age=%lu ms, CH1 age=%lu ms; "
                     "safe outputs active",
                     arm_sample.pulse_us,
                     (unsigned long)rc_sample_age_ms(&arm_sample, now_us),
                     gpio_get_level(PIN_RC_ARM_INPUT),
                     (unsigned long)rc_sample_age_ms(&throttle_sample, now_us),
                     (unsigned long)rc_sample_age_ms(&steering_sample, now_us));
            if (controller.drive_config.permanent_arm_latch_enabled) {
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
                         arm_unrecognized_pulse_us, controller.arm_cal.run_us,
                         controller.arm_cal.stop_1_us, controller.arm_cal.stop_2_us);
                if (controller.drive_config.permanent_arm_latch_enabled) {
                    set_drive_inhibit(DRIVE_INHIBIT_CH4, warning);
                } else {
                    return_to_disarmed(warning, true);
                }
            }
            return;
        }

        if (!arm_run) {
            if (controller.drive_config.permanent_arm_latch_enabled) {
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
        controller.arm_run_candidate_since_us = 0;
        return;
    }

    if (arm_unrecognized) {
        controller.arm_run_candidate_since_us = 0;
        return;
    }

    if (!arm_run) {
        controller.arm_run_candidate_since_us = 0;
        if (healthy_throttle && !controller.calibration_in_progress) {
            controller.arm_cycle_ready = true;
        }
    } else if (controller.arm_cycle_ready &&
               controller.system_state == SYSTEM_DISARMED && !controller.calibration_in_progress) {
        if (!healthy_throttle || !throttle_is_neutral()) {
            controller.arm_run_candidate_since_us = 0;
        } else if (controller.arm_run_candidate_since_us == 0) {
            controller.arm_run_candidate_since_us = now_us;
        } else if (now_us - controller.arm_run_candidate_since_us >= ARM_RUN_STABLE_US) {
            if (drive_arm_allowed()) {
                arm_drive();
            } else {
                controller.arm_run_candidate_since_us = now_us;
            }
        }
    }
}

void powertrain_arm_from_cli(void)
{
    controller_lock();
    if (controller.system_state != SYSTEM_DISARMED) {
        printf("ERR: controller must be DISARMED\n");
        controller_unlock();
        return;
    }
    if (!controller.arm_cycle_ready) {
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
    if (controller.system_state == SYSTEM_DRIVE_ARMED &&
        controller.drive_config.permanent_arm_latch_enabled) {
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
    if (controller.calibration_in_progress ||
        (controller.system_state != SYSTEM_DISARMED && controller.system_state != SYSTEM_DRIVE_ARMED)) {
        printf("ERR: configuration disarm is unavailable during calibration\n");
        controller_unlock();
        return false;
    }
    if (controller.system_state == SYSTEM_DISARMED) {
        controller_unlock();
        return true;
    }
    return_to_disarmed(
        "OK: drive disarmed for configuration; physical STOP-to-RUN cycle required",
        true);
    controller_unlock();
    return true;
}

bool powertrain_get_remote_snapshot(powertrain_remote_snapshot_t *snapshot)
{
    if (snapshot == NULL || controller_mutex == NULL) {
        return false;
    }

    powertrain_remote_snapshot_t result = {0};
    controller_lock();
    result.captured_at_us = esp_timer_get_time();
    result.state = controller.system_state;
    result.arm_cycle_ready = controller.arm_cycle_ready;
    result.drive_inhibit_flags = controller.drive_inhibit_flags;
    result.drive_inhibit_count = controller.drive_inhibit_count;
    powertrain_calibration_snapshot(&result);
    rc_input_get(RC_CHANNEL_THROTTLE, &result.throttle_input);
    rc_input_get(RC_CHANNEL_STEERING, &result.steering_input);
    rc_input_get(RC_CHANNEL_ARM, &result.arm_input);
    powertrain_steering_snapshot(&result);
    result.config = controller.drive_config;
    esc_output_get(&result.outputs);
    if (controller.rpm_initialized) {
        rpm_sensor_get_snapshot(&result.rpm);
    }
    controller_unlock();

    *snapshot = result;
    return true;
}

static void sensor_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        int64_t execution_started_us = esp_timer_get_time();
        if (controller.rpm_initialized) rpm_sensor_update();
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
        powertrain_steering_update();
        powertrain_calibration_check_timeout();
        handle_receiver_arm_control();

        if (controller.system_state == SYSTEM_DRIVE_ARMED) {
            update_drive_outputs();
        }
        powertrain_calibration_update();
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

    config_store_load_throttle(&controller.throttle_cal);
    config_store_load_steering(&controller.steering_cal);
    config_store_load_arm(&controller.arm_cal);
    config_store_load_drive(&controller.drive_config);
    powertrain_steering_init();
    uint16_t centers[STEERING_SERVO_COUNT];
    independent_steering_mix(&controller.drive_config.steering, 0.0f, 0.0f, centers, NULL);
    err = servo_output_init(centers);
    if (err != ESP_OK) return err;
    err = rc_input_init();
    if (err != ESP_OK) return err;

    err = rpm_sensor_init(controller.drive_config.rpm_pulses_per_revolution);
    if (err == ESP_OK) {
        controller.rpm_initialized = true;
    } else {
        ESP_LOGW(TAG, "RPM capture unavailable: %s", esp_err_to_name(err));
    }

    reset_drive_state();
    esc_output_set_safe();
    return ESP_OK;
}

esp_err_t powertrain_controller_start(void)
{
    BaseType_t created = xTaskCreatePinnedToCore(powertrain_task,
                                                 "powertrain",
                                                 6144,
                                                 NULL,
                                                 5,
                                                 NULL,
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
                          NULL);
    if (created != pdPASS) {
        ESP_LOGW(TAG, "sensor task could not start; rear speed remains unavailable");
    }
    return ESP_OK;
}

void powertrain_get_timing_snapshot(powertrain_timing_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&telemetry_lock);
    *snapshot = (powertrain_timing_snapshot_t){
        .sensor_overruns = sensor_overrun_count, .drive_overruns = powertrain_overrun_count,
        .sensor_max = sensor_max_execution_us, .drive_max = powertrain_max_execution_us,
        .sensor_stack = sensor_stack_min_free_bytes, .drive_stack = powertrain_stack_min_free_bytes,
    };
    portEXIT_CRITICAL(&telemetry_lock);
}
