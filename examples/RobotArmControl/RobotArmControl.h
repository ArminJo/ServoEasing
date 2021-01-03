/*
 * RobotArmControl.h
 *
 *  Created on: 01.10.2019
 *      Author: Armin
 */

#ifndef ROBOTARMCONTROL_H_
#define ROBOTARMCONTROL_H_

#include <stdint.h>

//#define ROBOT_ARM_IR_CONTROL
//#define ROBOT_ARM_RTC_CONTROL

#if defined(ROBOT_ARM_IR_CONTROL)
#define IR_INPUT_PIN  A0
#endif

#define VCC_STOP_THRESHOLD_MILLIVOLT 3500   // We have voltage drop at the connectors, so the battery voltage is assumed to be higher, than the Arduino VCC.
#define VCC_STOP_MIN_MILLIVOLT 3200         // We have voltage drop at the connectors, so the battery voltage is assumed to be higher, than the Arduino VCC.
#define VCC_CHECK_PERIOD_MILLIS 2000        // Period of VCC checks
#define VCC_STOP_PERIOD_REPETITIONS 9       // Shutdown after 9 times (18 seconds) VCC below VCC_STOP_THRESHOLD_MILLIVOLT or 1 time below VCC_STOP_MIN_MILLIVOLT

#define MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE 30000

#define MANUAL_CHECK_PERIOD_MILLIS 100        // Period of manual control checks


extern bool sVCCTooLow;

bool checkVCC();
void doRobotArmAutoMove();
bool delayAndCheckForRobotArm(uint16_t aDelayMillis);
void doSetToAutoModeForRobotArm();

#endif /* ROBOTARMCONTROL_H_ */

#pragma once
