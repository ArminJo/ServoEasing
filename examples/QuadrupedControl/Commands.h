/*
 * Commands.h
 *
 * list of functions to call by IR command
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "QuadrupedControl.h"  //must be first
#if defined(QUADRUPED_IR_CONTROL)
#include "IRCommandDispatcher.h"
#else
#define RETURN_IF_STOP
#endif

#define ACTION_TYPE_STOP    0
#define ACTION_TYPE_CREEP   1
#define ACTION_TYPE_TROT    2
#define ACTION_TYPE_TURN    3
#define ACTION_TYPE_TWIST   4
extern uint8_t sActionType; // can be set by commands and is reset if sRequestToStopReceived is set

// The code for the called command is available in variable sCurrentIRCode
// All functions have the prefix __attribute__((weak)) in order to enable easy overwriting with own functions.


// Basic moves
void doCreepForward();
void doCreepBack();
void doTrot();
void doTurnLeft();
void doTurnRight();

// Combined moves
void doDance();
void doWave();
void doTwist();
void doBow();
void doLeanLeft();
void doLeanRight();
void doLeanBack();
void doLeanFront();

void doAutoMove();
void doAttention();

// Special commands
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

#endif /* COMMANDS_H_ */

#pragma once
