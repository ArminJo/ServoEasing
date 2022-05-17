/*
 * RobotArmControl.h
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  ServoEasing is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef _ROBOT_ARM_CONTROL_H
#define _ROBOT_ARM_CONTROL_H

#include <stdint.h>

#define VCC_STOP_THRESHOLD_MILLIVOLT 3500   // We have voltage drop at the connectors, so the battery voltage is assumed to be higher, than the Arduino VCC.
#define VCC_STOP_MIN_MILLIVOLT 3200         // We have voltage drop at the connectors, so the battery voltage is assumed to be higher, than the Arduino VCC.
#define VCC_CHECK_PERIOD_MILLIS 2000        // Period of VCC checks
#define VCC_STOP_PERIOD_REPETITIONS 9       // Shutdown after 9 times (18 seconds) VCC below VCC_STOP_THRESHOLD_MILLIVOLT or 1 time below VCC_STOP_MIN_MILLIVOLT

#define _TIMEOUT_MILLIS_BEFORE_SWITCH_TO_AUTO_MOVE  30000
#define MILLIS_OF_INACTIVITY_BEFORE_ATTENTION       60000

#define MANUAL_CHECK_PERIOD_MILLIS 100        // Period of manual control checks

extern bool sVCCTooLow;
extern bool sDebugOutputIsEnabled;

bool checkVCC();
bool delayAndCheckForRobotArm(uint16_t aDelayMillis);
void doEnableAutoModeForRobotArm();
void doRobotArmAttention();

#endif // _ROBOT_ARM_CONTROL_H
