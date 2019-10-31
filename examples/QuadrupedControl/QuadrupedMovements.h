/*
 * QuadrupedMovements.h
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef SRC_QUADRUPEDMOVEMENTS_H_
#define SRC_QUADRUPEDMOVEMENTS_H_

#include "QuadrupedServoConfiguration.h" // for MOVE_DIRECTION_FORWARD

#include <stdint.h>

extern uint8_t sMovingDirection;

/*
 * Movements
 */
void basicHalfCreep(uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false);
void basicTwist(int8_t aTwistAngle, bool aTurnLeft = true);
void basicQuarterTurn(uint8_t aMoveLegIndex, bool aTurnLeft = false);
void moveTurn(uint8_t aNumberOfTurns = 0);          // 0 -> 256 turns
void moveTrot(uint8_t aNumberOfTrots = 0);      // 0 -> 256 trots
void moveCreep(uint8_t aNumberOfCreeps = 0);    // 0 -> 256 creeps

void checkIfBodyHeightHasChanged();

#endif /* SRC_QUADRUPEDMOVEMENTS_H_ */

#pragma once
