#pragma once

#include "driver/gpio.h"

/*
 * ESP32 DOIT DevKit V1 pin plan.
 *
 * Avoid boot strapping pins for ESC signals. V1 mirrors one logical drive
 * command across four separate ESC throttle outputs and four separate reverse
 * outputs so later firmware can add per-wheel trim without rewiring.
 */

#define PIN_RC_THROTTLE_INPUT GPIO_NUM_25 // good

#define PIN_ESC_FL_THROTTLE GPIO_NUM_15 // good
#define PIN_ESC_FR_THROTTLE GPIO_NUM_13 // good
#define PIN_ESC_RL_THROTTLE GPIO_NUM_21 // good
#define PIN_ESC_RR_THROTTLE GPIO_NUM_27 // good

#define PIN_ESC_FL_REVERSE GPIO_NUM_2 // good
#define PIN_ESC_FR_REVERSE GPIO_NUM_12 // good
#define PIN_ESC_RL_REVERSE GPIO_NUM_19 // good
#define PIN_ESC_RR_REVERSE GPIO_NUM_14 // good

#define PIN_ARM_SWITCH GPIO_NUM_33 // active-low, internal pull-up, switch to GND
