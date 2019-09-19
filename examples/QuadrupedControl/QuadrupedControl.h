/*
 * QuadrupedControl.h
 *
 *  Created on: 15.09.2019
 *      Author: Armin
 */

#ifndef QUADRUPEDCONTROL_H_
#define QUADRUPEDCONTROL_H_

#define PIN_SPEAKER     3
#define PIN_NEOPIXEL    4

#define PIN_TRIGGER_OUT     A3
#define PIN_ECHO_IN         A4

#define VCC_STOP_THRESHOLD_MILLIVOLT 3600 // stop moving if below 3.6 Volt
#define MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE 20000 // 20 seconds
#define MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE 120000 // 2 Minutes
#define MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE 60000 // 1 Minute

void doAutoMove();
void doTest();

void handleUSSensor();
bool checkForLowVoltage();
bool delayAndCheck(uint16_t aDelayMillis);

#endif /* QUADRUPEDCONTROL_H_ */

#pragma once
