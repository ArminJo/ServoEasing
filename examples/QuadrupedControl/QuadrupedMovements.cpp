/*
 * QuadrupedMovements.cpp
 *
 * This file  contains the basic movement functions
 *
 * To run this example need to install the "ServoEasing", "IRLremote" and "PinChangeInterrupt" libraries under "Tools -> Manage Libraries..." or "Ctrl+Shift+I"
 * Use "ServoEasing", "IRLremote" and "PinChangeInterrupt" as filter string.
 *
 *  Copyright (C) 2019  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  ServoEasing is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

#include "Commands.h"

#include "QuadrupedMovements.h"
#include "QuadrupedServoControl.h"

//#define INFO // activate this to see serial info output

uint8_t sMovingDirection = MOVE_DIRECTION_FORWARD;

/*
 * Gait variations
 * 1. Creep: Move one leg forward and down, then move body with all 4 legs down, then move diagonal leg.
 * 2. Trot: Move 2 diagonal legs up and forward
 */

void moveTrot(uint8_t aNumberOfTrots) {
    sActionType = ACTION_TYPE_TROT;
    setEasingTypeForMoving();

    uint8_t tCurrentDirection = sMovingDirection;
    do {
        uint8_t LiftMaxAngle = sBodyHeightAngle + ((LIFT_HIGHEST_ANGLE - sBodyHeightAngle) / 2);
        /*
         * first move right front and left back leg up and forward
         */
        transformAndSetAllServos(TROT_BASE_ANGLE_FL_BR + TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR - TROT_MOVE_ANGLE,
        TROT_BASE_ANGLE_FL_BR - TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR + TROT_MOVE_ANGLE, sBodyHeightAngle, LiftMaxAngle,
                sBodyHeightAngle, LiftMaxAngle, tCurrentDirection);
        RETURN_IF_STOP;

        checkIfBodyHeightHasChanged();

        // and the the other legs
        transformAndSetAllServos(TROT_BASE_ANGLE_FL_BR - TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR + TROT_MOVE_ANGLE,
        TROT_BASE_ANGLE_FL_BR + TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR - TROT_MOVE_ANGLE, LiftMaxAngle, sBodyHeightAngle,
                LiftMaxAngle, sBodyHeightAngle, tCurrentDirection);
        RETURN_IF_STOP;

        checkIfBodyHeightHasChanged();

        if (sMovingDirection != tCurrentDirection) {
            tCurrentDirection = sMovingDirection;
        }
        aNumberOfTrots--;
    } while (aNumberOfTrots != 0);

    setEasingTypeToLinear();
    sActionType = ACTION_TYPE_STOP;
}

void basicTwist(int8_t aTwistAngle, bool aTurnLeft) {
    sActionType = ACTION_TYPE_TWIST;
#ifdef INFO
    Serial.print(F("Twist angle="));
    Serial.print(aTwistAngle);
    if (aTurnLeft) {
        Serial.println(F(" left"));
    } else {
        Serial.println(F(" right"));
    }
#endif
    int8_t tTwistAngle;
    aTurnLeft ? tTwistAngle = -aTwistAngle : tTwistAngle = aTwistAngle;

    setPivotServos(90 + tTwistAngle, 90 + tTwistAngle, 90 + tTwistAngle, 90 + tTwistAngle);
    sActionType = ACTION_TYPE_STOP;
}

/*
 * Must reverse direction of legs if turning right otherwise the COG is not supported.
 */
void moveTurn(uint8_t aNumberOfQuarterTurns) {
    sActionType = ACTION_TYPE_TURN;
    centerServos();
    setEasingTypeForMoving();
    uint8_t tNextLegIndex = FRONT_LEFT;

    /*
     * Move one leg out of center position, otherwise the COG is not supported at the first move
     */
    if (sMovingDirection == MOVE_DIRECTION_LEFT) {
        moveOneServoAndCheckInputAndWait(FRONT_RIGHT_PIVOT, 90 + TURN_MOVE_ANGLE);
        RETURN_IF_STOP;
    } else {
        moveOneServoAndCheckInputAndWait(BACK_LEFT_PIVOT, 90 - TURN_MOVE_ANGLE);
        RETURN_IF_STOP;
    }

    do {
        basicQuarterTurn(tNextLegIndex, sMovingDirection == MOVE_DIRECTION_LEFT);
        RETURN_IF_STOP;
        // reverse direction of NextLegIndex if moveTurn right
        sMovingDirection == MOVE_DIRECTION_LEFT ? tNextLegIndex++ : tNextLegIndex--;
        tNextLegIndex = tNextLegIndex % NUMBER_OF_LEGS;
        aNumberOfQuarterTurns--;
    } while (aNumberOfQuarterTurns != 0);

    setEasingTypeToLinear();
    sActionType = ACTION_TYPE_STOP;
}

/*
 * One quarter turn is approximately 12 degree
 * @param aMoveLegIndex from 0 FRONT_LEFT to 3 FRONT_RIGHT
 */
void basicQuarterTurn(uint8_t aMoveLegIndex, bool aTurnLeft) {
#ifdef INFO
    Serial.print(F("Turn leg="));
    Serial.print(aMoveLegIndex);
    if (aTurnLeft) {
        Serial.println(F(" left"));
    } else {
        Serial.println(F(" right"));
    }
#endif
    int8_t tServoIndex = aMoveLegIndex * SERVOS_PER_LEG;
    int8_t tMoveAngle;
    int8_t tTurnAngle;

    /*
     * Move one leg forward in moveTurn direction
     */
    if (aTurnLeft) {
        tMoveAngle = TURN_MOVE_ANGLE;
        tTurnAngle = -TURN_BODY_ANGLE;
    } else {
        tMoveAngle = -TURN_MOVE_ANGLE;
        tTurnAngle = TURN_BODY_ANGLE;
    }
    ServoEasing::ServoEasingNextPositionArray[tServoIndex] = 90 + tMoveAngle;
    ServoEasing::ServoEasingNextPositionArray[tServoIndex + LIFT_SERVO_OFFSET] = LIFT_HIGHEST_ANGLE;
    tServoIndex += SERVOS_PER_LEG;

    /*
     *
     */
    for (uint8_t i = 0; i < NUMBER_OF_LEGS - 1; ++i) {
        tServoIndex %= NUMBER_OF_SERVOS;
        ServoEasing::ServoEasingNextPositionArray[tServoIndex] = ServoEasing::ServoEasingNextPositionArray[tServoIndex] + tTurnAngle;
// reset lift values for all other legs
        ServoEasing::ServoEasingNextPositionArray[tServoIndex + LIFT_SERVO_OFFSET] = sBodyHeightAngle;
        tServoIndex += SERVOS_PER_LEG;
    }
//    printArrayPositions(&Serial);
    synchronizeMoveAllServosAndCheckInputAndWait();
}

/*
 * Y position with right legs closed and left legs open
 */
void goToYPosition(uint8_t aDirection) {
#ifdef INFO
    Serial.print(F("goToYPosition aDirection="));
    Serial.println(aDirection);
#endif
    transformAndSetPivotServos(180 - Y_POSITION_OPEN_ANGLE, Y_POSITION_OPEN_ANGLE, (180 - Y_POSITION_CLOSE_ANGLE),
    Y_POSITION_CLOSE_ANGLE, aDirection, false, false);
    setLiftServos(sBodyHeightAngle);
}

/*
 * 0 -> 256 creeps
 */
void moveCreep(uint8_t aNumberOfCreeps) {
    sActionType = ACTION_TYPE_CREEP;
    goToYPosition(sMovingDirection);
    setEasingTypeForMoving();
    uint8_t tCurrentDirection = sMovingDirection;

    do {
        basicHalfCreep(tCurrentDirection, false);
        RETURN_IF_STOP;
// now mirror movement
        basicHalfCreep(tCurrentDirection, true);
        RETURN_IF_STOP;
        /*
         * Check if we have to change direction
         */
        if (sMovingDirection != tCurrentDirection) {
            tCurrentDirection = sMovingDirection;
        }
        aNumberOfCreeps--;
    } while (aNumberOfCreeps != 0);

    setEasingTypeToLinear();
    sActionType = ACTION_TYPE_STOP;
}

/*
 * moves one leg forward and down, then moves body, then moves diagonal leg.
 */
void basicHalfCreep(uint8_t aDirection, bool doMirror) {
#ifdef INFO
    Serial.print(F("BasicHalfCreep Direction="));
    Serial.print(aDirection);
    Serial.print(F(" doMirror="));
    Serial.println(doMirror);
// 1. Move front right leg up, forward and down
    Serial.println(F("Move front leg"));
#endif
    transformAndSetAllServos(180 - Y_POSITION_OPEN_ANGLE, Y_POSITION_OPEN_ANGLE, 180 - Y_POSITION_CLOSE_ANGLE,
    Y_POSITION_FRONT_ANGLE, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, LIFT_HIGHEST_ANGLE, aDirection, doMirror);
    RETURN_IF_STOP;

// check if body height has changed
    checkIfBodyHeightHasChanged();
// reset lift value
    ServoEasing::ServoEasingNextPositionArray[transformOneServoIndex(FRONT_RIGHT_PIVOT) + LIFT_SERVO_OFFSET] = sBodyHeightAngle;

// 2. Move body forward by CREEP_BODY_MOVE_ANGLE
#ifdef INFO
    Serial.println(F("Move body"));
#endif
    transformAndSetAllServos(180 - Y_POSITION_CLOSE_ANGLE, Y_POSITION_OPEN_ANGLE + CREEP_BODY_MOVE_ANGLE,
            180 - Y_POSITION_OPEN_ANGLE, Y_POSITION_OPEN_ANGLE, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle,
            sBodyHeightAngle, aDirection, doMirror);
    RETURN_IF_STOP;
    checkIfBodyHeightHasChanged();

// 3. Move back right leg up, forward and down
#ifdef INFO
    Serial.println(F("Move back leg to close position"));
#endif
// Move to Y position with other side legs together
    transformAndSetAllServos(180 - Y_POSITION_CLOSE_ANGLE, Y_POSITION_CLOSE_ANGLE, 180 - Y_POSITION_OPEN_ANGLE,
    Y_POSITION_OPEN_ANGLE, sBodyHeightAngle, LIFT_HIGHEST_ANGLE, sBodyHeightAngle, sBodyHeightAngle, aDirection, doMirror);
    RETURN_IF_STOP;

    checkIfBodyHeightHasChanged();

// reset lift value
    ServoEasing::ServoEasingNextPositionArray[transformOneServoIndex(BACK_LEFT_PIVOT) + LIFT_SERVO_OFFSET] = sBodyHeightAngle;
}

/*
 * Just as an unused example to see the principle of movement
 */
void basicSimpleHalfCreep(uint8_t aLeftLegIndex, bool aMoveMirrored) {
#ifdef INFO
    Serial.print(F("LeftLegIndex="));
    Serial.print(aLeftLegIndex);
#endif

    uint8_t tLeftLegPivotServoIndex;

    if (aMoveMirrored) {
#ifdef INFO
        Serial.print(F(" mirrored=true"));
#endif
// get index of pivot servo of mirrored leg
        tLeftLegPivotServoIndex = ((NUMBER_OF_LEGS - 1) - aLeftLegIndex) * SERVOS_PER_LEG; // 0->6
    } else {
        tLeftLegPivotServoIndex = aLeftLegIndex * SERVOS_PER_LEG;
    }
#ifdef INFO
    Serial.println();
#endif
//    printArrayPositions(&Serial);
    uint8_t tEffectiveAngle;

// 1. Move front left leg up, forward and down
#ifdef INFO
    Serial.println(F("Move front leg"));
#endif
    moveOneServoAndCheckInputAndWait(tLeftLegPivotServoIndex + LIFT_SERVO_OFFSET, LIFT_HIGHEST_ANGLE);
    RETURN_IF_STOP;

// go CREEP_BODY_MOVE_ANGLE ahead of Y_POSITION_OPEN_ANGLE
    aMoveMirrored ?
            tEffectiveAngle = 180 - (Y_POSITION_OPEN_ANGLE - CREEP_BODY_MOVE_ANGLE) :
            tEffectiveAngle = Y_POSITION_OPEN_ANGLE - CREEP_BODY_MOVE_ANGLE;
    moveOneServoAndCheckInputAndWait(tLeftLegPivotServoIndex, tEffectiveAngle);
    RETURN_IF_STOP;

    moveOneServoAndCheckInputAndWait(tLeftLegPivotServoIndex + LIFT_SERVO_OFFSET, sBodyHeightAngle);
    RETURN_IF_STOP;

// 2. Move body forward
#ifdef INFO
    Serial.println(F("Move body"));
#endif
    uint8_t tIndex = tLeftLegPivotServoIndex;
    uint8_t tIndexDelta;
// Front left
    if (aMoveMirrored) {
        ServoEasing::ServoEasingNextPositionArray[tIndex] = 180 - Y_POSITION_OPEN_ANGLE;
        tIndexDelta = -SERVOS_PER_LEG;
    } else {
        ServoEasing::ServoEasingNextPositionArray[tIndex] = Y_POSITION_OPEN_ANGLE;
        tIndexDelta = SERVOS_PER_LEG;
    }
// Back left
    tIndex = (tIndex + tIndexDelta) % NUMBER_OF_SERVOS;
    if (aMoveMirrored) {
        ServoEasing::ServoEasingNextPositionArray[tIndex] = Y_POSITION_OPEN_ANGLE;
    } else {
        ServoEasing::ServoEasingNextPositionArray[tIndex] = 180 - Y_POSITION_OPEN_ANGLE;
    }

// Back right
    tIndex = (tIndex + tIndexDelta) % NUMBER_OF_SERVOS;
    if (aMoveMirrored) {
        ServoEasing::ServoEasingNextPositionArray[tIndex] = 180 - CREEP_BODY_MOVE_ANGLE;
    } else {
        ServoEasing::ServoEasingNextPositionArray[tIndex] = CREEP_BODY_MOVE_ANGLE;
    }

// Front right
    tIndex = (tIndex + tIndexDelta) % NUMBER_OF_SERVOS;
    if (aMoveMirrored) {
        ServoEasing::ServoEasingNextPositionArray[tIndex] = Y_POSITION_CLOSE_ANGLE;
    } else {
        ServoEasing::ServoEasingNextPositionArray[tIndex] = 180 - Y_POSITION_CLOSE_ANGLE;
    }
//    printArrayPositions(&Serial);
    synchronizeMoveAllServosAndCheckInputAndWait();

// 3. Move back right leg up, forward and down
#ifdef INFO
    Serial.println(F("Move back leg to close position"));
#endif
// Move to Y position with right legs together / 120, 60, 180, 0
    uint8_t tDiagonalIndex = (tLeftLegPivotServoIndex + DIAGONAL_SERVO_OFFSET) % NUMBER_OF_SERVOS;
    moveOneServoAndCheckInputAndWait(tDiagonalIndex + LIFT_SERVO_OFFSET, LIFT_HIGHEST_ANGLE);
    RETURN_IF_STOP;

    aMoveMirrored ? tEffectiveAngle = 180 - Y_POSITION_CLOSE_ANGLE : tEffectiveAngle = Y_POSITION_CLOSE_ANGLE;
    moveOneServoAndCheckInputAndWait(tDiagonalIndex, tEffectiveAngle);
    RETURN_IF_STOP;

    moveOneServoAndCheckInputAndWait(tDiagonalIndex + LIFT_SERVO_OFFSET, sBodyHeightAngle);
}

void checkIfBodyHeightHasChanged() {
    static uint8_t tCurrentBodyHeightAngle = 0;
    // init static variable manually
    if (tCurrentBodyHeightAngle == 0) {
        tCurrentBodyHeightAngle = sBodyHeightAngle;
    }

    if (sBodyHeightAngle != tCurrentBodyHeightAngle) {
        setLiftServosToBodyHeight();
        tCurrentBodyHeightAngle = sBodyHeightAngle;
    }
}

