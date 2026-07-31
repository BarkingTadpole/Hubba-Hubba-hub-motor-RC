#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t powertrain_controller_init(void);
void powertrain_controller_start(void);

void powertrain_print_status(void);
void powertrain_print_help(void);
void powertrain_arm_from_cli(void);
void powertrain_disarm(void);

void powertrain_set_reverse_limit(uint8_t percent);
void powertrain_set_failsafe(uint16_t pulse_us, uint16_t window_us);
void powertrain_set_motor_poles(uint8_t poles);
void powertrain_set_tv_enabled(bool enabled);
void powertrain_set_tv_authority(uint8_t percent);
void powertrain_set_tv_gains(float yaw_gain_dps,
                             float turn_rpm_gain,
                             float yaw_kp,
                             float yaw_ki,
                             float rpm_kp);
void powertrain_set_imu_yaw_sign(int8_t sign);

void powertrain_calibrate_throttle(void);
void powertrain_calibrate_steering(void);
void powertrain_calibrate_arm(void);
void powertrain_calibrate_tv_mode(void);
void powertrain_calibrate_imu(void);
void powertrain_cal_esc_arm(void);
void powertrain_cal_esc_max(void);
void powertrain_cal_esc_min(void);
void powertrain_cal_manual(void);
void powertrain_cal_cancel(void);

void powertrain_monitor_throttle(void);
void powertrain_monitor_steering(void);
void powertrain_monitor_arm(void);
void powertrain_monitor_tv_mode(void);
void powertrain_monitor_rpm(void);
void powertrain_monitor_imu(void);
void powertrain_monitor_vectoring(void);
