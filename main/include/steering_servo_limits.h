#pragma once

/* User-supplied 180-degree positional-servo specification. These limits are
 * servo PWM, not receiver input limits or measured road-wheel travel. */
#define SERVO_PULSE_MIN_US 500
#define SERVO_PULSE_NEUTRAL_US 1500
#define SERVO_PULSE_MAX_US 2500
#define SERVO_SHAFT_TRAVEL_TENTHS_DEG 1800
#define SERVO_LINKAGE_MAX_TENTHS_DEG 900
#define SERVO_FREQUENCY_MIN_HZ 50
#define SERVO_FREQUENCY_MAX_HZ 330

/* Keep both base and trimmed neutral inside calibrated mechanical endpoints. */
#define SERVO_CENTER_CLEARANCE_US 50
#define STEERING_TRIM_MAX_US (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US - 2 * SERVO_CENTER_CLEARANCE_US)
