/*
 * QuadrupedControlCommands.h
 *
 * list of functions implemented, which can e.g. be called by IR remote
 *
 *  Copyright (C) 2019-2026  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of QuadrupedControl https://github.com/ArminJo/QuadrupedControl.
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  QuadrupedControl and ServoEasing are free software: you can redistribute it and/or modify
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
 */

#ifndef _QUADRUPED_CONTROL_COMMANDS_H
#define _QUADRUPED_CONTROL_COMMANDS_H

#define SERVO_MIN_SPEED       4 // degree per seconds
#define SERVO_MAX_SPEED     400 // degree per seconds

// The code for the called command is available in variable sCurrentIRCode

void doTrot();
void doCreep();
void doTurn();

// Combined moves
void doDance();
void doWave();
void doTwist();
void doQuadrupedDemoMove();
void doQuadrupedAutoMove();

// Moves not on the remote
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
