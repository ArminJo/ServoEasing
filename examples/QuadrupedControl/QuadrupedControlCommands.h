/*
 * QuadrupedControlCommands.h
 *
 * list of functions implemented, which can e.g. be called by IR remote
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef QUADRUPED_CONTROL_COMMANDS_H
#define QUADRUPED_CONTROL_COMMANDS_H

// Definition of RETURN_IF_STOP and BREAK_IF_STOP macro
#if defined(QUADRUPED_HAS_IR_CONTROL)
#include "IRCommandDispatcher.h" // RETURN_IF_STOP and BREAK_IF_STOP are defined here
#else
#  if defined(RETURN_IF_STOP)
#undef RETURN_IF_STOP
#  endif
#define RETURN_IF_STOP // define as empty
#  if defined(BREAK_IF_STOP)
#undef BREAK_IF_STOP
#  endif
#define BREAK_IF_STOP // define as empty
#endif

/*
 * Action type definitions
 * Currently used for NeoPatterns display
 */
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
extern uint8_t sCurrentlyRunningAction; // A change on this action type triggers the generation of new neopatterns
extern uint8_t sLastActionTypeForNeopatternsDisplay; // do determine changes of sCurrentlyRunningAction

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

#endif /* QUADRUPED_CONTROL_COMMANDS_H */
#pragma once
