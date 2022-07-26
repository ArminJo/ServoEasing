/*
 * QuadrupedControlCommands.h
 *
 * list of functions implemented, which can e.g. be called by IR remote
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef _QUADRUPED_CONTROL_COMMANDS_H
#define _QUADRUPED_CONTROL_COMMANDS_H

#define SERVO_MIN_SPEED       4 // degree per seconds
#define SERVO_MAX_SPEED     400 // degree per seconds

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
void doTest();

/*
 * Instant command functions
 */
void doStop();
void doPauseResume();
void doSetDirectionForward();
void doSetDirectionBack();
void doSetDirectionRight();
void doSetDirectionLeft();
void doIncreaseSpeed();
void doDecreaseSpeed();
void doIncreaseHeight();
void doDecreaseHeight();

#endif // _QUADRUPED_CONTROL_COMMANDS_H
