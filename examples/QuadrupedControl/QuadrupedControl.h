/*
 * QuadrupedControl.h
 *
 *  Created on: 15.09.2019
 *      Author: Armin
 */

#ifndef QUADRUPEDCONTROL_H_
#define QUADRUPEDCONTROL_H_

#include <stdint.h>

//#define INFO // activate this to see serial info output

//#define QUADRUPED_HAS_IR_CONTROL
//#define QUADRUPED_HAS_NEOPIXEL
//#define QUADRUPED_HAS_US_DISTANCE
//#define QUADRUPED_PLAYS_RTTTL

#if defined(QUADRUPED_HAS_IR_CONTROL) && !defined(IR_RECEIVER_PIN)
#define IR_INPUT_PIN  A0
#endif

#if defined(QUADRUPED_HAS_US_DISTANCE)
#define PIN_TRIGGER_OUT     A3
#define PIN_ECHO_IN         A4
#define PIN_US_SERVO        13

#define MILLIS_BETWEEN_MEASUREMENTS 200 // 5 per second

void handleUSSensor();
#endif

#define PIN_BUZZER     3

#define VCC_STOP_THRESHOLD_MILLIVOLT 3600 // stop moving if below 3.6 volt
#define MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE 20000 // 20 seconds
#define MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE 120000 // 2 Minutes
#define MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE 60000 // 1 Minute

bool checkForLowVoltage();
bool delayAndCheck(uint16_t aDelayMillis);

#endif /* QUADRUPEDCONTROL_H_ */

#pragma once
