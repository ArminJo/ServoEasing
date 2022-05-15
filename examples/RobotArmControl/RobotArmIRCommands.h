/*
 * RobotArmIRCommands.h
 *
 * list of functions to call by IR command
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef _ROBOT_ARM_IR_COMMANDS_H
#define _ROBOT_ARM_IR_COMMANDS_H

#if defined(ROBOT_ARM_HAS_IR_CONTROL)
#include "IRCommandDispatcher.h"
#else
#define RETURN_IF_STOP
#define DELAY_AND_RETURN_IF_STOP(aDurationMillis)
#endif

#if defined(ROBOT_ARM_HAS_RTC_CONTROL)
#include "ClockMovements.h"
#endif

#define ACTION_TYPE_STOP        0
#define ACTION_TYPE_DRAW_TIME   1

extern bool sInverseKinematicModeActive;
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
