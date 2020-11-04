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
#if defined(QUADRUPED_HAS_IR_CONTROL)
#include "IRCommandDispatcher.h" // RETURN_IF_STOP is defined here
#else
#  if defined(RETURN_IF_STOP)
#undef RETURN_IF_STOP
#  endif
#define RETURN_IF_STOP // define as empty
#endif

#define ACTION_TYPE_STOP    0
#define ACTION_TYPE_CREEP   1
#define ACTION_TYPE_TROT    2
#define ACTION_TYPE_TURN    3
#define ACTION_TYPE_TWIST   4
#define ACTION_TYPE_ATTENTION   5
#define ACTION_TYPE_LEAN    6
#define ACTION_TYPE_WAVE    7
// only used by UserCommands.cpp
#define ACTION_TYPE_DANCE   8
#define ACTION_TYPE_TEST    9
#define ACTION_TYPE_AUTO_MOVE   10
extern uint8_t sActionType; // must be set by exclusive commands and is reset if sRequestToStopReceived is set
extern uint8_t sLastActionType; // do determine changes of actionType

// The code for the called command is available in variable sCurrentIRCode
// All functions have the prefix __attribute__((weak)) in order to enable easy overwriting with own functions.

// Basic moves
void doCreepForward();
void doTrot();
void doTurnLeft();
void doTurnRight();

// Combined moves
void doDance();
void doWave();
void doTwist();
void doQuadrupedAutoMove();

// not on the remote
void doBow();
void doLeanLeft();
void doLeanRight();
void doLeanBack();
void doLeanFront();
void doAttention();
void doBeep();

// Special commands
void doCenterServos();
void doCalibration();
void doTest();

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

/*
 * Functions for bigger remote controls
 */
void doUSRight();
void doUSLeft();
void doUSScan();

void doPattern1();
void doPattern2();
void doPattern3();
void doPatternFire();
void doPatternHeartbeat();
void wipeOutPatterns();

#endif /* COMMANDS_H_ */

#pragma once
