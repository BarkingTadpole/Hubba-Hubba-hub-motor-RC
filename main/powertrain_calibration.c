#include "powertrain_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "calibration_validation.h"
#include "config_store.h"
#include "esc_output.h"
#include "esp_timer.h"
#include "servo_output.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ESC_CAL_TIMEOUT_US 30000000
#define REMOTE_CAL_TIMEOUT_US 120000000

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

static remote_calibration_state_t remote_calibration = {
    .message = "No browser calibration has been started",
};
static int64_t esc_cal_last_command_us;
static uint32_t wheel_capture_sequence;

int powertrain_calibration_wheel_side(void)
{
    if (!remote_calibration.active) return -1;
    if (remote_calibration.kind == POWERTRAIN_CALIBRATION_WHEEL_LEFT) return STEERING_LEFT;
    if (remote_calibration.kind == POWERTRAIN_CALIBRATION_WHEEL_RIGHT) return STEERING_RIGHT;
    return -1;
}

static bool wheel_measurement_inputs_safe(void)
{
    return controller.system_state == SYSTEM_DISARMED && controller.arm_cal.loaded_from_nvs &&
           shutdown_channel_is_safe() && throttle_is_neutral() && !throttle_matches_failsafe() &&
           rc_input_is_valid(RC_CHANNEL_STEERING);
}

static bool calibration_allowed(bool require_neutral, bool require_saved_shutdown)
{
    if (controller.calibration_in_progress) {
        printf("ERR: another calibration is already active\n");
        return false;
    }
    if (controller.system_state != SYSTEM_DISARMED) {
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

    if (require_saved_shutdown && !controller.arm_cal.loaded_from_nvs) {
        printf("ERR: run 'cal arm' before ESC calibration or manual relay\n");
        return false;
    }
    if (controller.arm_cal.loaded_from_nvs && !shutdown_channel_is_safe()) {
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
        controller.calibration_in_progress = true;
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

/* Controller mutex must be held while applying any completed calibration. */
static bool apply_throttle_calibration(uint16_t neutral,
                                       uint16_t full,
                                       uint16_t reverse)
{
    if (!calibration_endpoints_valid(full, neutral, reverse)) {
        printf("ERR: throttle endpoints must be at least 150 us from opposite sides of neutral\n");
        return false;
    }

    throttle_calibration_t previous = controller.throttle_cal;
    controller.throttle_cal = (throttle_calibration_t){
        .full_throttle_us = full,
        .neutral_us = neutral,
        .full_reverse_us = reverse,
        .deadband_us = 80,
        .loaded_from_nvs = true,
    };
    esp_err_t err = config_store_save_throttle(&controller.throttle_cal);
    if (err != ESP_OK) controller.throttle_cal = previous;
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
    if (!calibration_steering_valid(left, center, right, controller.steering_cal.deadband_us)) {
        printf("ERR: steering endpoints must bracket center by at least 150 us and exceed the saved deadband\n");
        return false;
    }

    steering_calibration_t previous = controller.steering_cal;
    controller.steering_cal = (steering_calibration_t){
        .left_us = left,
        .center_us = center,
        .right_us = right,
        .deadband_us = previous.deadband_us,
        .loaded_from_nvs = true,
    };
    esp_err_t err = config_store_save_steering(&controller.steering_cal);
    if (err != ESP_OK) {
        controller.steering_cal = previous;
        printf("ERR: failed to save steering calibration: %s\n",
               esp_err_to_name(err));
        return false;
    }
    printf("OK: steering calibration saved\n");
    powertrain_steering_recenter();
    return true;
}

static bool apply_arm_calibration(uint16_t run,
                                  uint16_t stop_1,
                                  uint16_t stop_2)
{
    if (!calibration_arm_positions_valid(run, stop_1, stop_2)) {
        printf("ERR: RUN, STOP 1, and STOP 2 must each differ by at least 250 us\n");
        return false;
    }

    arm_calibration_t previous = controller.arm_cal;
    controller.arm_cal = (arm_calibration_t){
        .run_us = run,
        .stop_1_us = stop_1,
        .stop_2_us = stop_2,
        .loaded_from_nvs = true,
    };
    esp_err_t err = config_store_save_arm(&controller.arm_cal);
    if (err != ESP_OK) controller.arm_cal = previous;
    printf(err == ESP_OK
               ? "OK: receiver arm-channel calibration saved\n"
               : "ERR: failed to save arm calibration: %s\n",
           err == ESP_OK ? "" : esp_err_to_name(err));
    controller.arm_cycle_ready = false;
    return err == ESP_OK;
}

static const char *remote_calibration_kind_name(powertrain_calibration_kind_t kind)
{
    switch (kind) {
    case POWERTRAIN_CALIBRATION_THROTTLE: return "receiver throttle";
    case POWERTRAIN_CALIBRATION_STEERING: return "steering";
    case POWERTRAIN_CALIBRATION_ARM: return "CH4 arm/output inhibit";
    case POWERTRAIN_CALIBRATION_ESC: return "ESC endpoints/manual relay";
    case POWERTRAIN_CALIBRATION_WHEEL_LEFT: return "left wheel angle";
    case POWERTRAIN_CALIBRATION_WHEEL_RIGHT: return "right wheel angle";
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

    if (step >= 3) return "Calibration capture is complete";
    switch (kind) {
    case POWERTRAIN_CALIBRATION_THROTTLE: return throttle_prompts[step];
    case POWERTRAIN_CALIBRATION_STEERING: return steering_prompts[step];
    case POWERTRAIN_CALIBRATION_ARM: return arm_prompts[step];
    case POWERTRAIN_CALIBRATION_WHEEL_LEFT:
    case POWERTRAIN_CALIBRATION_WHEEL_RIGHT:
        return "Use CH1 to position this wheel; capture PWM, measure road-wheel angle, then enter it below";
    default: return "No calibration is active";
    }
}

static void remote_calibration_finish(const char *message)
{
    bool was_wheel = powertrain_calibration_wheel_side() >= 0;
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
    controller.calibration_in_progress = false;
    if (was_wheel) powertrain_steering_recenter();
}

bool powertrain_remote_calibration_start(powertrain_calibration_kind_t kind)
{
    bool wheel = kind == POWERTRAIN_CALIBRATION_WHEEL_LEFT || kind == POWERTRAIN_CALIBRATION_WHEEL_RIGHT;
    if (!wheel && (kind < POWERTRAIN_CALIBRATION_THROTTLE || kind > POWERTRAIN_CALIBRATION_ARM)) {
        return false;
    }

    controller_lock();
    bool require_neutral = kind != POWERTRAIN_CALIBRATION_THROTTLE;
    if (!calibration_allowed(require_neutral, false)) {
        controller_unlock();
        return false;
    }
    rc_channel_sample_t steering;
    if (wheel && (!wheel_measurement_inputs_safe() || controller.drive_config.steering.toe_tenths_deg != 0 ||
        !rc_input_get(RC_CHANNEL_STEERING, &steering) ||
        abs((int)steering.pulse_us - controller.steering_cal.center_us) > controller.steering_cal.deadband_us)) {
        printf("ERR: wheel measurement needs saved CH4 STOP, neutral CH1/CH2, healthy receiver and zero toe\n");
        controller_unlock();
        return false;
    }

    memset(&remote_calibration, 0, sizeof(remote_calibration));
    remote_calibration.active = true;
    remote_calibration.kind = kind;
    remote_calibration.total_steps = wheel ? 0U : 3U;
    remote_calibration.last_action_us = esp_timer_get_time();
    snprintf(remote_calibration.prompt, sizeof(remote_calibration.prompt), "%s",
             remote_calibration_prompt(kind, 0));
    snprintf(remote_calibration.message, sizeof(remote_calibration.message),
             "%s calibration started; follow the displayed capture steps",
             remote_calibration_kind_name(kind));
    controller.calibration_in_progress = true;
    controller.arm_run_candidate_since_us = 0;
    if (wheel) {
        controller.arm_cycle_ready = false;
        reset_drive_state();
        esc_output_set_safe();
        powertrain_steering_recenter();
    }
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
    int wheel_side = powertrain_calibration_wheel_side();
    if (wheel_side >= 0) {
        if (!wheel_measurement_inputs_safe()) { controller_unlock(); return false; }
        uint16_t pulses[STEERING_SERVO_COUNT];
        servo_output_get_pulses(pulses);
        remote_calibration.values_us[0] = pulses[wheel_side];
        remote_calibration.last_action_us = esp_timer_get_time();
        wheel_capture_sequence++;
        snprintf(remote_calibration.message, sizeof(remote_calibration.message),
                 "Captured commanded servo PWM %u us; enter the measured road-wheel angle", pulses[wheel_side]);
        controller_unlock();
        return true;
    }

    powertrain_calibration_kind_t kind = remote_calibration.kind;
    uint8_t step = remote_calibration.step;
    remote_calibration.sampling = true;
    remote_calibration.last_action_us = esp_timer_get_time();
    snprintf(remote_calibration.message, sizeof(remote_calibration.message),
             "Sampling receiver PWM for 1500 ms");
    controller_unlock();

    rc_channel_id_t channel = kind == POWERTRAIN_CALIBRATION_THROTTLE
                                 ? RC_CHANNEL_THROTTLE
                             : kind == POWERTRAIN_CALIBRATION_STEERING
                                 ? RC_CHANNEL_STEERING : RC_CHANNEL_ARM;
    uint16_t pulse_us = average_channel(channel, 1500);

    controller_lock();
    if (!remote_calibration.active || remote_calibration.kind != kind ||
        remote_calibration.step != step) {
        controller_unlock();
        return false;
    }
    remote_calibration.sampling = false;
    remote_calibration.last_action_us = esp_timer_get_time();

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
    controller.calibration_in_progress = false;
    controller_unlock();
}

void powertrain_calibrate_steering(void)
{
    if (!begin_interactive_calibration(true, false)) return;
    printf("Steering calibration started. Both servos continue following valid CH1 input.\n");
    uint16_t center = capture_channel(RC_CHANNEL_STEERING, "Hold steering centered.");
    uint16_t left = center ? capture_channel(RC_CHANNEL_STEERING, "Hold full left steering.") : 0;
    uint16_t right = left ? capture_channel(RC_CHANNEL_STEERING, "Hold full right steering.") : 0;

    controller_lock();
    if (center && left && right) apply_steering_calibration(center, left, right);
    controller.calibration_in_progress = false;
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
    controller.calibration_in_progress = false;
    controller_unlock();
}

bool powertrain_cal_esc_arm(void)
{
    controller_lock();
    if (!calibration_allowed(true, true)) {
        controller_unlock();
        return false;
    }
    controller.system_state = SYSTEM_ESC_CAL_ARMED;
    esc_cal_last_command_us = esp_timer_get_time();
    esc_output_set_safe();
    printf("OK: ESC calibration armed. Keep ESC battery disconnected, then send 'cal esc max'.\n");
    controller_unlock();
    return true;
}

bool powertrain_cal_esc_max(void)
{
    controller_lock();
    if (controller.system_state != SYSTEM_ESC_CAL_ARMED &&
        controller.system_state != SYSTEM_ESC_CAL_MAX &&
        controller.system_state != SYSTEM_ESC_CAL_MIN) {
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
    controller.system_state = SYSTEM_ESC_CAL_MAX;
    esc_cal_last_command_us = esp_timer_get_time();
    esc_output_set_all(esc_limits.throttle_max_us, esc_limits.reverse_low_us);
    printf("OK: maximum throttle active. Power ESCs, wait for endpoint beeps, then send 'cal esc min'.\n");
    controller_unlock();
    return true;
}

bool powertrain_cal_esc_min(void)
{
    controller_lock();
    if (controller.system_state != SYSTEM_ESC_CAL_MAX) {
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
    controller.system_state = SYSTEM_ESC_CAL_MIN;
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
    controller.system_state = SYSTEM_ESC_CAL_MANUAL;
    esc_cal_last_command_us = esp_timer_get_time();
    printf("OK: direct throttle relay active for 30 seconds; all reverse outputs remain low.\n");
    controller_unlock();
    return true;
}

bool powertrain_cal_cancel(void)
{
    controller_lock();
    if (controller.system_state == SYSTEM_DRIVE_ARMED) {
        printf("ERR: drive is armed; no calibration is active\n");
        controller_unlock();
        return false;
    }
    if (controller.calibration_in_progress && !remote_calibration.active) {
        printf("ERR: serial interactive calibration cannot be canceled from another task\n");
        controller_unlock();
        return false;
    }
    controller.system_state = SYSTEM_DISARMED;
    remote_calibration_finish("Calibration canceled; safe outputs restored");
    controller.arm_cycle_ready = false;
    reset_drive_state();
    esc_output_set_safe();
    printf("OK: calibration canceled; safe outputs restored\n");
    controller_unlock();
    return true;
}


void powertrain_calibration_snapshot(powertrain_remote_snapshot_t *snapshot)
{
    bool esc_calibration_active = controller.system_state == SYSTEM_ESC_CAL_ARMED ||
                                  controller.system_state == SYSTEM_ESC_CAL_MAX ||
                                  controller.system_state == SYSTEM_ESC_CAL_MIN ||
                                  controller.system_state == SYSTEM_ESC_CAL_MANUAL;
    snapshot->calibration_active = remote_calibration.active ||
                                controller.calibration_in_progress ||
                                esc_calibration_active;
    snapshot->calibration_sampling = remote_calibration.sampling;
    snapshot->calibration_kind = remote_calibration.kind;
    snapshot->calibration_step = remote_calibration.step;
    snapshot->calibration_total_steps = remote_calibration.total_steps;
    snapshot->wheel_capture_sequence = wheel_capture_sequence;
    memcpy(snapshot->calibration_values_us, remote_calibration.values_us,
           sizeof(snapshot->calibration_values_us));
    snprintf(snapshot->calibration_prompt, sizeof(snapshot->calibration_prompt), "%s",
             remote_calibration.prompt);
    snprintf(snapshot->calibration_message, sizeof(snapshot->calibration_message), "%s",
             remote_calibration.message);
    if (esc_calibration_active) {
        snapshot->calibration_kind = POWERTRAIN_CALIBRATION_ESC;
        snapshot->calibration_sampling = false;
        snapshot->calibration_total_steps =
            controller.system_state == SYSTEM_ESC_CAL_MANUAL ? 1U : 3U;
        snapshot->calibration_step = controller.system_state == SYSTEM_ESC_CAL_ARMED ? 0U
                                  : controller.system_state == SYSTEM_ESC_CAL_MAX ? 1U
                                  : controller.system_state == SYSTEM_ESC_CAL_MIN ? 2U
                                                                       : 0U;
        const char *prompt = controller.system_state == SYSTEM_ESC_CAL_ARMED
                                 ? "Keep traction power disconnected, then select ESC maximum"
                             : controller.system_state == SYSTEM_ESC_CAL_MAX
                                 ? "After maximum-endpoint beeps, select ESC minimum"
                             : controller.system_state == SYSTEM_ESC_CAL_MIN
                                 ? "After ready beeps, cancel to restore safe output"
                                 : "Manual receiver-throttle relay is active for at most 30 seconds";
        snprintf(snapshot->calibration_prompt, sizeof(snapshot->calibration_prompt),
                 "%s", prompt);
        snprintf(snapshot->calibration_message, sizeof(snapshot->calibration_message),
                 "ESC calibration/manual output mode is active");
    } else if (controller.calibration_in_progress && !remote_calibration.active) {
        snprintf(snapshot->calibration_message, sizeof(snapshot->calibration_message),
                 "A USB serial interactive calibration is active");
    }
}

void powertrain_calibration_check_timeout(void)
{
    int64_t now_us = esp_timer_get_time();
    if (powertrain_calibration_wheel_side() >= 0 && !wheel_measurement_inputs_safe()) {
        remote_calibration_finish("Wheel measurement stopped: keep CH4 STOP, CH2 neutral and receiver signals healthy");
        controller.arm_cycle_ready = false;
        reset_drive_state();
        esc_output_set_safe();
    }
    if (remote_calibration.active && !remote_calibration.sampling &&
        now_us - remote_calibration.last_action_us > REMOTE_CAL_TIMEOUT_US) {
        remote_calibration_finish(
            "Browser calibration timed out after 120 seconds; safe DISARMED state retained");
        controller.arm_cycle_ready = false;
        reset_drive_state();
        esc_output_set_safe();
        printf("\nWARN: browser calibration timed out; safe outputs retained\n> ");
    }
}

void powertrain_calibration_update(void)
{
    if (controller.system_state == SYSTEM_ESC_CAL_MANUAL) {
        rc_channel_sample_t throttle;
        if (throttle_matches_failsafe() || !throttle_sample(&throttle)) {
            powertrain_cal_cancel();
            printf("\nWARN: manual ESC relay stopped due to receiver signal loss or failsafe\n> ");
        } else {
            esc_output_set_all(throttle.pulse_us, esc_limits.reverse_low_us);
        }
    }

    if (controller.system_state == SYSTEM_ESC_CAL_ARMED ||
        controller.system_state == SYSTEM_ESC_CAL_MAX ||
        controller.system_state == SYSTEM_ESC_CAL_MIN ||
        controller.system_state == SYSTEM_ESC_CAL_MANUAL) {
        if (esp_timer_get_time() - esc_cal_last_command_us > ESC_CAL_TIMEOUT_US) {
            powertrain_cal_cancel();
            printf("\nWARN: ESC calibration timed out\n> ");
        }
    }
}
