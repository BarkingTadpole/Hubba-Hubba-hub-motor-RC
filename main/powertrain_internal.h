#pragma once

/* Private to the powertrain modules. Public callers use powertrain_controller.h.
 * Shared mutable context access and steering/calibration helpers require the
 * recursive controller mutex, except initialization before tasks start.
 * Public entry points acquire the mutex themselves.
 * RPM initialization is immutable after startup. Timing counters have their
 * own short critical section and are exposed only through a copy. */
#include "powertrain_controller.h"
#include "rc_input.h"

#define DRIVE_UPDATE_INTERVAL_MS 20

typedef struct {
    throttle_calibration_t throttle_cal;
    steering_calibration_t steering_cal;
    arm_calibration_t arm_cal;
    drive_config_t drive_config;
    system_state_t system_state;
    bool calibration_in_progress;
    bool rpm_initialized;
    bool arm_cycle_ready;
    uint32_t drive_inhibit_flags;
    uint32_t drive_inhibit_count;
    int64_t arm_run_candidate_since_us;
} powertrain_context_t;

extern powertrain_context_t controller;
extern const esc_limits_t esc_limits;

typedef struct {
    uint32_t sensor_overruns, drive_overruns;
    uint32_t sensor_max, drive_max;
    uint32_t sensor_stack, drive_stack;
} powertrain_timing_snapshot_t;

void controller_lock(void);
void controller_unlock(void);
bool pulse_is_on_endpoint_side(uint16_t pulse_us,
                               uint16_t endpoint_us,
                               uint16_t center_us);
bool throttle_sample(rc_channel_sample_t *sample);
bool throttle_is_neutral(void);
bool throttle_matches_failsafe(void);
bool shutdown_channel_is_safe(void);
void reset_drive_state(void);
const char *arm_position_name(uint16_t pulse_us);
bool arm_requests_run(uint16_t pulse_us);
uint32_t rc_sample_age_ms(const rc_channel_sample_t *sample, int64_t now_us);
const char *drive_inhibit_reason(uint32_t flags);

void powertrain_get_timing_snapshot(powertrain_timing_snapshot_t *snapshot);
void powertrain_steering_init(void);
void powertrain_steering_update(void);
void powertrain_steering_recenter(void);
void powertrain_steering_snapshot(powertrain_remote_snapshot_t *snapshot);
void powertrain_calibration_check_timeout(void);
void powertrain_calibration_update(void);
void powertrain_calibration_snapshot(powertrain_remote_snapshot_t *snapshot);
