#pragma once

#include <stdbool.h>
#include <stdint.h>

#define POWERTRAIN_WHEEL_COUNT 4

typedef enum {
    WHEEL_FRONT_LEFT = 0,
    WHEEL_FRONT_RIGHT,
    WHEEL_REAR_LEFT,
    WHEEL_REAR_RIGHT,
} wheel_id_t;

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

typedef enum {
    TV_MODE_OFF = 0,
    TV_MODE_STRAIGHT,
    TV_MODE_FULL,
} tv_mode_t;

typedef struct {
    uint16_t pulse_us;
    int64_t updated_at_us;
    bool valid;
} rc_channel_sample_t;

typedef struct {
    uint16_t full_throttle_us;
    uint16_t neutral_us;
    uint16_t full_reverse_us;
    uint16_t deadband_us;
    bool loaded_from_nvs;
} throttle_calibration_t;

typedef struct {
    uint16_t left_us;
    uint16_t center_us;
    uint16_t right_us;
    uint16_t deadband_us;
    bool loaded_from_nvs;
} steering_calibration_t;

typedef struct {
    uint16_t run_us;
    uint16_t stop_1_us;
    uint16_t stop_2_us;
    bool loaded_from_nvs;
} arm_calibration_t;

typedef struct {
    uint16_t off_us;
    uint16_t straight_us;
    uint16_t full_us;
    bool loaded_from_nvs;
} tv_mode_calibration_t;

typedef struct {
    uint16_t throttle_min_us;
    uint16_t throttle_max_us;
    uint16_t reverse_low_us;
    uint16_t reverse_high_us;
} esc_limits_t;

typedef struct {
    uint8_t reverse_limit_percent;
    bool receiver_failsafe_enabled;
    uint16_t receiver_failsafe_us;
    uint16_t receiver_failsafe_window_us;
    uint8_t motor_poles;
    uint16_t rpm_pulses_per_revolution;
    int16_t steering_trim_tenths_deg;
    uint16_t steering_smoothing_ms;
    bool steering_speed_limit_enabled;
    float steering_lateral_accel_g;
    bool torque_vectoring_enabled;
    uint8_t tv_authority_percent;
    uint8_t tv_front_relief_percent;
    float tv_turn_yaw_gain_dps;
    float tv_turn_rpm_gain;
    float tv_yaw_kp;
    float tv_yaw_ki;
    float tv_rpm_kp;
    int8_t imu_yaw_sign;
    bool loaded_from_nvs;
} drive_config_t;

typedef struct {
    float rpm[POWERTRAIN_WHEEL_COUNT];
    float frequency_hz[POWERTRAIN_WHEEL_COUNT];
    uint32_t edge_count[POWERTRAIN_WHEEL_COUNT];
    uint32_t window_us[POWERTRAIN_WHEEL_COUNT];
    bool valid[POWERTRAIN_WHEEL_COUNT];
    int64_t updated_at_us;
    uint16_t pulses_per_revolution;
} rpm_snapshot_t;

typedef struct {
    float accel_mps2[3];
    float gyro_dps[3];
    float yaw_rate_dps;
    int64_t updated_at_us;
    bool valid;
    bool bias_calibrated;
} imu_snapshot_t;

typedef struct {
    uint16_t throttle_us[POWERTRAIN_WHEEL_COUNT];
    uint16_t reverse_us[POWERTRAIN_WHEEL_COUNT];
} wheel_output_command_t;
