/*
 * QuadrupedBasicMovements.h
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

#ifndef _QUADRUPED_BASIC_MOVEMENTS_H
#define _QUADRUPED_BASIC_MOVEMENTS_H

#include "QuadrupedServoConfiguration.h" // for MOVE_DIRECTION_FORWARD

#include <stdint.h>

extern volatile uint8_t sMovingDirection;

#if ! defined(QUADRUPED_BREAK_IF_STOP)
#define QUADRUPED_BREAK_IF_STOP     if (QUADRUPED_MOVEMENT_BREAK_FLAG) break
#endif
#if ! defined(QUADRUPED_RETURN_IF_STOP)
#define QUADRUPED_RETURN_IF_STOP     if (QUADRUPED_MOVEMENT_BREAK_FLAG) return
#endif

/*
 * Action type definitions
 * Currently used for NeoPatterns display
 */
#define ACTION_TYPE_STOP        0
#define ACTION_TYPE_CREEP       1
#define ACTION_TYPE_TROT        2
#define ACTION_TYPE_TURN        3
#define ACTION_TYPE_TWIST       4
#define ACTION_TYPE_SHUTDOWN    5
#define ACTION_TYPE_LEAN        6
// only used by UserCommands.cpp
#define ACTION_TYPE_DOWN_UP     7
#define ACTION_TYPE_TEST        8
#define ACTION_TYPE_WAVE        9
#define ACTION_TYPE_PAUSE      10
#define ACTION_TYPE_MELODY     11

#define COMBINED_ACTION_TYPE_STOP       0 // combined action
#define COMBINED_ACTION_TYPE_DANCE      1 // combined action
#define COMBINED_ACTION_TYPE_AUTO_MOVE  2 // combined action
#define COMBINED_ACTION_TYPE_DEMO_MOVE  3 // combined action



extern uint8_t sCurrentlyRunningCombinedAction;
extern uint8_t sCurrentlyRunningAction; // A change on this action type triggers the generation of new neopatterns
extern uint8_t sLastRunningAction; // do determine changes of sCurrentlyRunningAction

/*
 * Movements
 */
void basicHalfCreep(uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false);
void basicTwist(int8_t aTwistAngle, bool aTurnLeft = true);
void twist(int8_t aTwistAngle, uint8_t aNumberOfTwists);
void basicTurn(uint8_t aMoveLegIndex, bool aTurnLeft = false);
void moveTurn(uint8_t aNumberOfTurns = 0);          // 0 -> 256 turns
void moveTrot(uint8_t aNumberOfTrots = 0);      // 0 -> 256 trots
void moveCreep(uint8_t aNumberOfCreeps = 0);    // 0 -> 256 creeps
void lean(uint8_t aDirection);
void downAndUp(uint8_t aNumberOfDownAndUps);

void setActionToStop();

#endif // _QUADRUPED_BASIC_MOVEMENTS_H
