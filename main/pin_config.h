#pragma once

#include "driver/gpio.h"

/*
 * ESP32 DOIT DevKit V1 pin plan.
 *
 * Receiver and RPM inputs must be conditioned to 3.3 V before they reach the
 * ESP32. GPIO34-GPIO39 are input-only and do not provide internal pull-ups.
 */

/* Radiolink R7FG PWM inputs. */
#define PIN_RC_THROTTLE_INPUT GPIO_NUM_25
#define PIN_RC_STEERING_INPUT GPIO_NUM_32
#define PIN_RC_ARM_INPUT GPIO_NUM_33
#define PIN_RC_TV_MODE_INPUT GPIO_NUM_26

/* Four independent ESC throttle outputs. */
#define PIN_ESC_FL_THROTTLE GPIO_NUM_13
#define PIN_ESC_FR_THROTTLE GPIO_NUM_14
#define PIN_ESC_RL_THROTTLE GPIO_NUM_21
#define PIN_ESC_RR_THROTTLE GPIO_NUM_18

/* Four independent ESC reverse/direction outputs. */
#define PIN_ESC_FL_REVERSE GPIO_NUM_2
#define PIN_ESC_FR_REVERSE GPIO_NUM_27
#define PIN_ESC_RL_REVERSE GPIO_NUM_19
#define PIN_ESC_RR_REVERSE GPIO_NUM_17

/* Hobbywing HW86060041 RPM sensor signal inputs. */
#define PIN_RPM_FL_INPUT GPIO_NUM_34
#define PIN_RPM_FR_INPUT GPIO_NUM_35
#define PIN_RPM_RL_INPUT GPIO_NUM_36
#define PIN_RPM_RR_INPUT GPIO_NUM_39

/* Adafruit STEMMA QT ISM330DHCX I2C interface and optional interrupts. */
#define PIN_IMU_I2C_SDA GPIO_NUM_23
#define PIN_IMU_I2C_SCL GPIO_NUM_22
#define PIN_IMU_INT1 GPIO_NUM_NC
#define PIN_IMU_INT2 GPIO_NUM_NC
