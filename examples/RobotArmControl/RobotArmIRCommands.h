/*
 * RobotArmIRCommands.h
 *
 *  List of functions to call by IR command
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef _ROBOT_ARM_IR_COMMANDS_H
#define _ROBOT_ARM_IR_COMMANDS_H

#if defined(ROBOT_ARM_HAS_IR_CONTROL)
#include "IRCommandDispatcher.h"
#else
#define RETURN_IF_STOP
#define DELAY_AND_RETURN_IF_STOP(aDurationMillis)
#endif

#include "ClockMovements.h"

#define ACTION_TYPE_STOP        0
#define ACTION_TYPE_DRAW_TIME   1

extern bool sInverseKinematicModeForCombinedMovementsIsActive;
extern uint8_t sActionType;

void doRobotArmAutoMove();
void doRobotArmTestMove();
void doGoCenter();
void doGoFolded();
void doGoForward();
void doGoBack();
void doTurnRight();
void doTurnLeft();
void doLiftUp();
void doLiftDown();
void doOpenClaw();
void doCloseClaw();

void doSwitchToManual();
void doInverseKinematicOff();
void doInverseKinematicOn();
void doToggleInverseKinematic();
void doTestInverseKinematic();

/*
 * Instant command functions
 */
void doStop();

void doIncreaseSpeed();
void doDecreaseSpeed();
void doSwitchEasingType();

#endif // _ROBOT_ARM_IR_COMMANDS_H
