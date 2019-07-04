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

extern bool sInverseKinematicModeActive;

void doAutoMove();
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

// from ClockMovements.cpp
void doClockTest();

void doSwitchToManual();
void doInverseKinematicOff();
void doInverseKinematicOn();
void doToggleInverseKinematic();

/*
 * Instant command functions
 */
void doStop();
// from RobotArmControl.cpp and used by  doStop()
void setToAutoMode();

void doIncreaseSpeed();
void doDecreaseSpeed();
void doSwitchEasingType();

#endif /* SRC_COMMANDS_H_ */

//Added by Sloeber 
#pragma once
