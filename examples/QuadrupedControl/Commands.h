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

#include "QuadrupedControl.h" // for doAutoMove() and doMyMovement()

// The code for the called command is available in variable sCurrentIRCode
void doBeep();

void internalAutoMove();
void doAttention();

void doDance();
void doWave();
void doTwist();
void doBow();
void doLeanLeft();
void doLeanRight();
void doLeanBack();
void doLeanFront();

void doCreepForward();
void doCreepBack();
void doTrot();

void doTurnLeft();
void doTurnRight();

void doCenterServos();

void doCalibration();

/*
 * Instant command functions
 */
void doStop();
void doSetDirectionForward();
void doSetDirectionBack();
void doSetDirectionRight();
void doSetDirectionLeft();
void doIncreaseSpeed();
void doDecreaseSpeed();
void doIncreaseHeight();
void doDecreaseHeight();
void convertBodyHeightAngleToHeight();

#endif /* SRC_COMMANDS_H_ */

#pragma once
