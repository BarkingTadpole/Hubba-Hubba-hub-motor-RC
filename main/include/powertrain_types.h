#pragma once

#include <stdbool.h>
#include <stdint.h>

#define POWERTRAIN_WHEEL_COUNT 4
#define REAR_RPM_COUNT 2
#define RPM_REAR_LEFT 0
#define RPM_REAR_RIGHT 1
#define STEERING_SERVO_COUNT 2
#define STEERING_LEFT 0
#define STEERING_RIGHT 1
#define ACKERMANN_MAX_POINTS 8
#define INDEPENDENT_STEERING_VERSION 2

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

static inline const char *system_state_name(system_state_t state)
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

typedef enum {
    DRIVE_DIRECTION_FORWARD = 0,
    DRIVE_DIRECTION_REVERSE,
} drive_direction_t;

typedef enum {
    DRIVETRAIN_AWD = 0,
    DRIVETRAIN_FWD,
    DRIVETRAIN_RWD,
} drivetrain_mode_t;

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
    uint16_t throttle_min_us;
    uint16_t throttle_max_us;
    uint16_t reverse_low_us;
    uint16_t reverse_high_us;
} esc_limits_t;

/* Endpoints are wheel directions, so swapping left/right reverses a servo. */
typedef struct {
    uint16_t left_us;
    uint16_t center_us;
    uint16_t right_us;
    uint16_t left_tenths_deg;
    uint16_t right_tenths_deg;
} steering_servo_config_t;

typedef struct {
    uint16_t speed_tenths_kmh;
    uint16_t strength_tenths_percent;
} ackermann_point_t;

typedef struct {
    uint16_t wheelbase_tenths_mm;
    uint16_t track_tenths_mm;
    uint16_t wheel_diameter_tenths_mm;
    uint16_t max_left_tenths_deg;
    uint16_t max_right_tenths_deg;
} steering_geometry_t;

/* Versioned, fixed-width NVS blob; the entire curve is saved atomically. */
typedef struct {
    uint16_t version;
    steering_servo_config_t servo[STEERING_SERVO_COUNT];
    steering_geometry_t geometry;
    uint16_t point_count;
    ackermann_point_t points[ACKERMANN_MAX_POINTS];
} independent_steering_config_t;

typedef struct {
    drivetrain_mode_t drivetrain_mode;
    uint8_t reverse_limit_percent;
    uint8_t drive_smoothing_percent;
    bool receiver_failsafe_enabled;
    uint16_t receiver_failsafe_us;
    uint16_t receiver_failsafe_window_us;
    uint8_t motor_poles;
    uint16_t rpm_pulses_per_revolution;
    uint16_t steering_smoothing_ms;
    independent_steering_config_t steering;
    bool permanent_arm_latch_enabled;
    bool loaded_from_nvs;
} drive_config_t;

typedef struct {
    float rpm[REAR_RPM_COUNT];
    float frequency_hz[REAR_RPM_COUNT];
    uint32_t edge_count[REAR_RPM_COUNT];
    uint32_t window_us[REAR_RPM_COUNT];
    bool valid[REAR_RPM_COUNT];
    int64_t updated_at_us;
    uint16_t pulses_per_revolution;
} rpm_snapshot_t;

typedef struct {
    uint16_t throttle_us[POWERTRAIN_WHEEL_COUNT];
    uint16_t reverse_us[POWERTRAIN_WHEEL_COUNT];
} wheel_output_command_t;
