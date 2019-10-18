/*
 * Commands.h
 *
 * list of functions to call by IR command
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef SRC_COMMANDS_H_
#define SRC_COMMANDS_H_

#include "RobotArmControl.h"

#if defined(ROBOT_ARM_IR_CONTROL)
#include "IRCommandDispatcher.h"
#else
#define RETURN_IF_STOP
#endif

#if defined(ROBOT_ARM_RTC_CONTROL)
#include "ClockMovements.h"
#endif

#define ACTION_TYPE_STOP        0
#define ACTION_TYPE_DRAW_TIME   1

extern bool sInverseKinematicModeActive;
extern uint8_t sActionType;

void doRobotArmAutoMove();
void doCenter();
void doFolded();
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

/*
 * Instant command functions
 */
void doStop();

void doIncreaseSpeed();
void doDecreaseSpeed();
void doSwitchEasingType();

#endif /* SRC_COMMANDS_H_ */

#pragma once
