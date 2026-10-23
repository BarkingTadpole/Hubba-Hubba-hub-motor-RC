#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "powertrain_types.h"
#include "remote_command.h"

#define POWERTRAIN_CALIBRATION_PROMPT_MAX 112
#define POWERTRAIN_CALIBRATION_MESSAGE_MAX 160

typedef enum {
    POWERTRAIN_CALIBRATION_NONE = 0,
    POWERTRAIN_CALIBRATION_THROTTLE,
    POWERTRAIN_CALIBRATION_STEERING,
    POWERTRAIN_CALIBRATION_ARM,
    POWERTRAIN_CALIBRATION_ESC,
} powertrain_calibration_kind_t;

typedef struct {
    int64_t captured_at_us;
    system_state_t state;
    bool arm_cycle_ready;
    uint32_t drive_inhibit_flags;
    uint32_t drive_inhibit_count;
    bool calibration_active;
    bool calibration_sampling;
    powertrain_calibration_kind_t calibration_kind;
    uint8_t calibration_step;
    uint8_t calibration_total_steps;
    uint16_t calibration_values_us[3];
    char calibration_prompt[POWERTRAIN_CALIBRATION_PROMPT_MAX];
    char calibration_message[POWERTRAIN_CALIBRATION_MESSAGE_MAX];
    rc_channel_sample_t throttle_input;
    rc_channel_sample_t steering_input;
    rc_channel_sample_t arm_input;
    uint16_t steering_servo_us[STEERING_SERVO_COUNT];
    uint16_t steering_filter_us;
    bool steering_filter_active;
    uint32_t steering_rejected_spikes;
    float vehicle_speed_mps;
    bool vehicle_speed_valid;
    bool vehicle_speed_held;
    float steering_command;
    float ackermann_strength_percent;
    float steering_angle_deg[STEERING_SERVO_COUNT];
    steering_calibration_t steering_calibration;
    drive_config_t config;
    wheel_output_command_t outputs;
    rpm_snapshot_t rpm;
} powertrain_remote_snapshot_t;

esp_err_t powertrain_controller_init(void);
esp_err_t powertrain_controller_start(void);
bool powertrain_get_remote_snapshot(powertrain_remote_snapshot_t *snapshot);
void powertrain_print_status(void);
void powertrain_print_help(void);
void powertrain_arm_from_cli(void);
void powertrain_disarm(void);
bool powertrain_disarm_for_configuration(void);

bool powertrain_set_drivetrain_mode(drivetrain_mode_t mode);
bool powertrain_set_reverse_limit(uint8_t percent);
bool powertrain_set_drive_smoothing(uint8_t percent);
bool powertrain_set_failsafe(uint16_t pulse_us, uint16_t window_us);
bool powertrain_set_failsafe_enabled(bool enabled);
bool powertrain_set_motor_poles(uint8_t poles);
bool powertrain_set_rpm_pulses_per_revolution(uint16_t pulses_per_revolution);
bool powertrain_set_steering_smoothing(uint16_t smoothing_ms);
bool powertrain_set_steering_servo(uint8_t side, const steering_servo_config_t *servo);
bool powertrain_set_steering_geometry(const steering_geometry_t *geometry);
bool powertrain_set_steering_input(const steering_calibration_t *calibration);
bool powertrain_set_ackermann_curve(const ackermann_point_t *points, uint16_t count);
bool powertrain_set_permanent_arm_latch_enabled(bool enabled);

void powertrain_calibrate_throttle(void);
void powertrain_calibrate_steering(void);
void powertrain_calibrate_arm(void);
bool powertrain_cal_esc_arm(void);
bool powertrain_cal_esc_max(void);
bool powertrain_cal_esc_min(void);
bool powertrain_cal_manual(void);
bool powertrain_cal_cancel(void);
bool powertrain_remote_calibration_start(powertrain_calibration_kind_t kind);
bool powertrain_remote_calibration_capture(void);

void powertrain_monitor_throttle(void);
void powertrain_monitor_steering(void);
void powertrain_monitor_arm(void);
void powertrain_monitor_rpm(void);

/* Dispatch a parsed command through the controller guards. */
bool powertrain_apply_command(const remote_command_t *command);
