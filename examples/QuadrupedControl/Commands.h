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

void internalAutoMove();
void doAttention();

void doAutoMove();
void doDance();
void doWave();
void doTwist();
void doBow();
void doLeanLeft();
void doLeanRight();

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

#endif /* SRC_COMMANDS_H_ */
