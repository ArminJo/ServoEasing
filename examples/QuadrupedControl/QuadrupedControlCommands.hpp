/*
 * QuadrupedControlCommands.hpp
 *
 * Contains all the IR command functions available.
 * All functions have the prefix "do" to identify them as IR control commands.
 *
 *  Copyright (C) 2019-2026  Armin Joachimsmeyer
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

#ifndef _QUADRUPED_CONTROL_COMMANDS_HPP
#define _QUADRUPED_CONTROL_COMMANDS_HPP

#include <Arduino.h>

#include "QuadrupedBasicMovements.h"
#include "QuadrupedControlCommands.h"
#include "QuadrupedBasicMovements.hpp"

#if defined(QUADRUPED_ENABLE_RTTTL)
void doRandomMelody(); // Required for doTest()
#endif

// This block must be located after the includes of other *.hpp files
//#define LOCAL_INFO  // This enables info output only for this file
#include "LocalDebugLevelStart.h"

uint8_t sCurrentDemoMoveIndex = 0;

/******************************************
 * The Commands to execute
 ******************************************/
void doTest() {
#if defined(LOCAL_INFO)
    Serial.print(F("Test"));
    printQuadrupedServoSpeed();
#endif
#if defined(QUADRUPED_ENABLE_RTTTL)
    doRandomMelody();
#else
    twist(45, 2);
#endif
}

void doCenterServos() {
#if defined(LOCAL_INFO)
    Serial.print(F("Center"));
    printQuadrupedServoSpeed();
#endif
    centerServos();
}

void doTwist() {
#if defined(LOCAL_INFO)
    Serial.print(F("Twist"));
    printQuadrupedServoSpeed();
#endif
    twist(30, 2);
}

void doTrot() {
#if defined(LOCAL_INFO)
    Serial.print(F("Trot"));
    printQuadrupedServoSpeed();
#endif
    moveTrot();
}

void doCreep() {
#if defined(LOCAL_INFO)
    Serial.print(F("Creep"));
    printQuadrupedServoSpeed();
#endif
    moveCreep();
}

void doTurn() {
#if defined(LOCAL_INFO)
    Serial.print(F("Turn"));
    printQuadrupedServoSpeed();
#endif
    moveTurn();
}

/*
 * A movement to get attention, that quadruped may be switched off
 * Move down and up and back to starting height
 */
void doAttention() {
#if defined(LOCAL_INFO)
    Serial.println(F("Start attention"));
#endif
//    sCurrentlyRunningAction = ACTION_TYPE_ATTENTION;
    // Move down and up and back to starting height
    downAndUp(1);
}

void doWave() {
    sCurrentlyRunningAction = ACTION_TYPE_WAVE;

#if defined(LOCAL_INFO)
    Serial.print(F("Wave 3 times with right leg"));
    printQuadrupedServoSpeed();
#endif
    while (true) { // To force end handling
        uint8_t tRequestedBodyHeightAngle = sRequestedBodyHeightAngle; // sRequestedBodyHeightAngle is volatile
        // move front left and back right leg 10 degree forward to avoid falling to front if lifting the front right leg
        moveAllServosToNextPositionsAndCheckInputAndWait(80, 90, 100, 90, tRequestedBodyHeightAngle, tRequestedBodyHeightAngle,
                tRequestedBodyHeightAngle, tRequestedBodyHeightAngle);
        QUADRUPED_BREAK_IF_STOP;

        // move all legs up, except front left -> front right lifts from the ground
        moveLiftServosToNextPositionsAndCheckInputAndWait(LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE,
        LIFT_HIGHEST_ANGLE);

#if defined(APPLICATION_DELAY_AND_CHECK_AVAILABLE)
        delayAndCheckByApplication(1000);
        QUADRUPED_BREAK_IF_STOP;
#else
        delay(1000); // wait 1 second then start new move
#endif

        ServoEasing::ServoEasingArray[FRONT_RIGHT_PIVOT]->setEasingType(EASE_QUADRATIC_IN_OUT);

        // wave with the front right leg
        for (uint_fast8_t i = 0; i < 3; ++i) {
            moveOneServoAndCheckInputAndWait(FRONT_RIGHT_PIVOT, 135, sQuadrupedServoSpeed * 2);
            QUADRUPED_BREAK_IF_STOP;

            moveOneServoAndCheckInputAndWait(FRONT_RIGHT_PIVOT, 45, sQuadrupedServoSpeed * 2);
            QUADRUPED_BREAK_IF_STOP; // break to force setEasingType(EASE_LINEAR) before return
        }
        break;
    }
    ServoEasing::ServoEasingArray[FRONT_RIGHT_PIVOT]->setEasingType(EASE_LINEAR);
#if defined(APPLICATION_DELAY_AND_CHECK_AVAILABLE)
    delayAndCheckByApplication(1000);
#else
    delay(1000); // wait 1 second then start new move
#endif
    centerServos();
    setActionToStop();
}

/*
 * Dance and wave
 * 1. Center
 * 2. 2 times up and down
 * 3. 2 times lean all directions and twist
 * 4. Wave
 * 5. Center
 */
void doDanceAndWave() {
    sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_DANCE;
#if defined(LOCAL_INFO)
    Serial.print(F("Dance+Wave"));
    printQuadrupedServoSpeed();
#endif

#if defined(LOCAL_INFO)
#endif
    centerServos();
    while (true) { // To force end handling
        /*
         * Move down and up and back to current height
         */
        downAndUp(2);
        QUADRUPED_BREAK_IF_STOP;

        for (int i = 0; i < 2; ++i) {
            doOneQuadrupedDemoMove(DEMO_MOVE_LEAN);
            centerServos();
            QUADRUPED_BREAK_IF_STOP;
            twist(60, 2);
            centerServos();
            QUADRUPED_BREAK_IF_STOP;
        }
        QUADRUPED_BREAK_IF_STOP;
        doWave();
        break;
    }

    // end handling
    sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_STOP;
}

/*
 * Some small moves which show the features of a Quadruped and do not relocate it
 */
void doOneQuadrupedDemoMove(uint8_t aIndexOfDemoMove) {
    switch (aIndexOfDemoMove) {
    case DEMO_MOVE_DOWN_AND_UP:
        downAndUp(2);
        break;
    case DEMO_MOVE_TWIST:
        twist(60, 2);
        centerServos();
        break;
    case DEMO_MOVE_TURN: // Ends with Y position
        moveTurn(13, DO_RIGHT_TURN); // 13 turns -> around 180 degree
        delay(500);
        moveTurn(14, DO_LEFT_TURN); // with 13 we do not reach starting position
        break;
    case DEMO_MOVE_LEAN: // ends with back down
        centerServos();
        // 2 times left and right
        for (int i = 0; i < 2; ++i) {
            lean(MOVE_DIRECTION_LEFT);
            QUADRUPED_BREAK_IF_STOP;
            lean(MOVE_DIRECTION_RIGHT);
            QUADRUPED_BREAK_IF_STOP;
        }
        centerServos(); // Do this before returning
        QUADRUPED_BREAK_IF_STOP;
        // 2 times forward and backward
        for (int i = 0; i < 2; ++i) {
            lean(MOVE_DIRECTION_FORWARD);
            QUADRUPED_BREAK_IF_STOP;
            lean(MOVE_DIRECTION_BACKWARD);
            QUADRUPED_BREAK_IF_STOP;
        }
        break;
        case DEMO_MOVE_WAVE:
        doWave();
        break;
        default:
        break;
    }
}

/**
 * A demo move for table top / restricted areas
 * Called as demo if no command was ever sent.
 *
 *  UP and Down and Twist.
 *  Go forward, turn 90 degrees right, then go back sideways.
 *  Twist.
 *  Go forward, which was originally the right direction, then turn 90 degrees left, then go back sideways.
 *  Dance and wave.
 *
 * - Speed 140 - medium
 * - Center
 * - 2x move down and up and back to starting height
 * - 2x twist
 * - 3x creep forward turn and 3x creep back sideways
 * - Now we are back at start position
 *
 * - Center
 * - Creep forward turn forward and creep back sideways
 * - Now we are back at start position
 *
 * - Dance and wave.
 */
void doQuadrupedLongDemoMove() {
    doQuadrupedDemoMoves(true);
}
void doQuadrupedDemoMoves(bool aDoLongDemo) {
    uint16_t tOriginalSpeed = sQuadrupedServoSpeed;
#if defined(LOCAL_INFO)
    Serial.print(F("Start demo move sequence"));
#endif
    setQuadrupedServoSpeed(140);
    while (true) { // To force end handling

        if (!aDoLongDemo) {
            /*
             * Do demo move 0 to 4
             */
            doOneQuadrupedDemoMove(sCurrentDemoMoveIndex);
            sCurrentDemoMoveIndex++;
            if (sCurrentDemoMoveIndex > DEMO_MOVE_MAX_VALUE) {
                sCurrentDemoMoveIndex = 0;
            }
            break; // Force end handling
        }

        sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_DEMO_MOVE;

        centerServos();

        // Move down and up and back to starting height
        doOneQuadrupedDemoMove(DEMO_MOVE_DOWN_AND_UP);
        QUADRUPED_BREAK_IF_STOP;

        // twist
        doOneQuadrupedDemoMove(DEMO_MOVE_TWIST);
        QUADRUPED_BREAK_IF_STOP;

        /*
         * creep forward turn and creep back sideways
         */
        sMovingDirection = MOVE_DIRECTION_FORWARD;
        moveCreep(3);
        QUADRUPED_BREAK_IF_STOP;

        // turn 90 degree right, look right
        sMovingDirection = MOVE_DIRECTION_RIGHT;
        moveTurn(7);
        QUADRUPED_BREAK_IF_STOP;

        // creep right backward, look right
        sMovingDirection = MOVE_DIRECTION_RIGHT;
        moveCreep(3);
        QUADRUPED_BREAK_IF_STOP;

        /*
         * Now we are back at start position
         * Creep forward turn forward and creep back sideways
         */
        // creep forward, look right
        sMovingDirection = MOVE_DIRECTION_FORWARD;
        moveCreep(3);
        QUADRUPED_BREAK_IF_STOP;

        // turn 90 degree left, look forward
        sMovingDirection = MOVE_DIRECTION_LEFT;
        moveTurn(8);
        QUADRUPED_BREAK_IF_STOP;

        // creep left, look forward
        sMovingDirection = MOVE_DIRECTION_LEFT;
        moveCreep(3);
        QUADRUPED_BREAK_IF_STOP;

        /*
         * Now we are back at start position - dance
         */
        doDanceAndWave();
        break;
    }

    // end handling
    sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_STOP;
    // restore speed
    setQuadrupedServoSpeed(tOriginalSpeed);
}

/*
 * Feature move, needs more space than demo move.
 * lasts around 3 minutes
 *
 * UP and Down and Twist
 *
 * - Speed 90 - slow
 * - Center
 * - 4x move down and up and back to starting height, 4 times to distinguish it from demo move
 * - 2x creep forward
 *
 * - Speed 260 - fast
 * - 6x creep forward fast
 * - 8x creep right fast
 * - 4x creep left fast
 * - 1 seconds pause
 *
 * - turn right
 * - Dance fast with speed 200
 * - Center
 *
 * - Speed 160 - medium
 * - 10x trot forward
 * - 8x  trot backward
 * - 1 second pause
 *
 * - 8x trot right
 * - 2 seconds pause
 *
 * - Center
 * - Turn left
 * - Dance
 */
void doQuadrupedFeatureMove() {
    uint16_t tOriginalSpeed = sQuadrupedServoSpeed;
    sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_FEATURE_MOVE;

#if defined(LOCAL_INFO)
    Serial.println();
    Serial.print(F("Start feature move sequence"));
#endif

    setQuadrupedServoSpeed(90);

    centerServos();
    while (true) { // To force end handling

        // Move down and up and back to starting height
        downAndUp(4); // Do it 4 times to distinguish it from demo move
        QUADRUPED_BREAK_IF_STOP;

        // creep forward slow
        sMovingDirection = MOVE_DIRECTION_FORWARD;
        moveCreep(2);
        QUADRUPED_BREAK_IF_STOP;

        // creep forward fast
        setQuadrupedServoSpeed(260);
        moveCreep(6);
        QUADRUPED_BREAK_IF_STOP;

        // to avoid strange movement from one y position to the next.
        centerServos();
        QUADRUPED_BREAK_IF_STOP;

        // creep right fast
        sMovingDirection = MOVE_DIRECTION_RIGHT;
        moveCreep(8);
        QUADRUPED_BREAK_IF_STOP;

        // to avoid strange movement from one y position to the next.
        centerServos();
        QUADRUPED_BREAK_IF_STOP;

        // creep left fast
        sMovingDirection = MOVE_DIRECTION_LEFT;
        moveCreep(4);
#if defined(APPLICATION_DELAY_AND_CHECK_AVAILABLE)
        delayAndCheckByApplication(1000);
        QUADRUPED_BREAK_IF_STOP;
#else
        delay(1000); // wait 1 second then start new move
#endif

        // turn right
        sMovingDirection = MOVE_DIRECTION_RIGHT;
//    triggerNeoPatterns();
        moveTurn(11);
        QUADRUPED_BREAK_IF_STOP;

        // Dance
        setQuadrupedServoSpeed(200);
        doDanceAndWave();
        sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_FEATURE_MOVE; // we must restore it after doDanceAndWave()
        QUADRUPED_BREAK_IF_STOP;

        // trot forward
        setQuadrupedServoSpeed(160);
        sMovingDirection = MOVE_DIRECTION_FORWARD;
        centerServos();
        moveTrot(10);
        QUADRUPED_BREAK_IF_STOP;

        // trot backward
        sMovingDirection = MOVE_DIRECTION_BACKWARD;
        moveTrot(8);
#if defined(APPLICATION_DELAY_AND_CHECK_AVAILABLE)
        delayAndCheckByApplication(1000);
        QUADRUPED_BREAK_IF_STOP;
#else
        delay(1000); // wait 1 second then start new move
#endif

        // trot right
        sMovingDirection = MOVE_DIRECTION_RIGHT;
        moveTrot(8);
#if defined(APPLICATION_DELAY_AND_CHECK_AVAILABLE)
        delayAndCheckByApplication(2000);
        QUADRUPED_BREAK_IF_STOP;
#else
        delay(2000); // wait 2 seconds then start new move
#endif

        // turn left
        centerServos();
        sMovingDirection = MOVE_DIRECTION_LEFT;
        moveTurn(12);
        QUADRUPED_BREAK_IF_STOP;

        // dance
        doDanceAndWave();
        break;
    }

    // end handling
    sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_STOP;
    // restore speed
    setQuadrupedServoSpeed(tOriginalSpeed);
}

/*************************
 * Instant Commands
 *************************/
void doStop() {
    setActionToStop(); // this also stops NeoPatterns
}

void doPauseResume() {
    if (sCurrentlyRunningAction != ACTION_TYPE_STOP) {
        if (sCurrentlyRunningAction == ACTION_TYPE_PAUSE) {
#if defined(LOCAL_INFO)
            Serial.print(F("Resume with action="));
            Serial.println(sLastRunningAction);
#endif
            sCurrentlyRunningAction = sLastRunningAction; // restore also action
            resumeWithoutInterruptsAllServos();
//        resumeWithInterruptsAllServos();  // start the interrupts for NeoPatterns if disabled.
        } else {
#if defined(LOCAL_INFO)
            Serial.println(F("Pause"));
#endif
            pauseAllServos(); // this does not stop the interrupts for NeoPatterns.
//        disableServoEasingInterrupt(); // stop the interrupts for NeoPatterns.
            sLastRunningAction = sCurrentlyRunningAction;
            sCurrentlyRunningAction = ACTION_TYPE_PAUSE;
        }
    }
}

/**
 * Sets direction to MOVE_DIRECTION_FORWARD
 * If IR is enabled, starts a creep if not already in a trot movement
 * This enables direction changes of a running trot.
 */
void doSetDirectionForward() {
    sMovingDirection = MOVE_DIRECTION_FORWARD;
#if defined(LOCAL_INFO)
    Serial.print(F("Forward"));
    printQuadrupedServoSpeed();
#endif
#if defined(QUADRUPED_HAS_IR_CONTROL)
    // This enables to change also trot direction
    if (sCurrentlyRunningAction != ACTION_TYPE_TROT && sCurrentlyRunningAction != ACTION_TYPE_CREEP) {
        // If not doing trot or creep, start a creep
        IRDispatcher.setNextBlockingCommand(COMMAND_CREEP);
    }
#endif
}

void doSetDirectionBack() {
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
#if defined(QUADRUPED_HAS_IR_CONTROL)
    // This enables to change also trot direction
    if (sCurrentlyRunningAction != ACTION_TYPE_TROT && sCurrentlyRunningAction != ACTION_TYPE_CREEP) {
        // If not doing trot or creep, start a creep
        IRDispatcher.setNextBlockingCommand(COMMAND_CREEP);
    }
#endif
}

void doSetDirectionLeft() {
    sMovingDirection = MOVE_DIRECTION_LEFT;
#if defined(QUADRUPED_HAS_IR_CONTROL)
    // This enables to change also creep and trot direction
    if (sCurrentlyRunningAction != ACTION_TYPE_TROT && sCurrentlyRunningAction != ACTION_TYPE_CREEP) {
        // If not doing trot or creep, start a turn
        IRDispatcher.setNextBlockingCommand(COMMAND_TURN);
    }
#endif
}

void doSetDirectionRight() {
    sMovingDirection = MOVE_DIRECTION_RIGHT;
#if defined(QUADRUPED_HAS_IR_CONTROL)
    // This enables to change also creep and trot direction
    if (sCurrentlyRunningAction != ACTION_TYPE_TROT && sCurrentlyRunningAction != ACTION_TYPE_CREEP) {
        // If not doing trot or creep, start a turn
        IRDispatcher.setNextBlockingCommand(COMMAND_TURN);
    }
#endif
}

/*
 * Increase moving speed by 25%
 */
void doIncreaseSpeed() {
    sQuadrupedServoSpeed += sQuadrupedServoSpeed / 4;
    if (sQuadrupedServoSpeed > SERVO_MAX_SPEED) {
        sQuadrupedServoSpeed = SERVO_MAX_SPEED;
    }
    setSpeedForAllServos(sQuadrupedServoSpeed);
#if defined(LOCAL_INFO)
    printQuadrupedServoSpeed();
#endif
}

/*
 * Decrease moving speed by 25%
 */
void doDecreaseSpeed() {
    sQuadrupedServoSpeed -= sQuadrupedServoSpeed / 4;
    if (sQuadrupedServoSpeed < SERVO_MIN_SPEED) {
        sQuadrupedServoSpeed = SERVO_MIN_SPEED;
    }
    setSpeedForAllServos(sQuadrupedServoSpeed);
#if defined(LOCAL_INFO)
    printQuadrupedServoSpeed();
#endif
}

/*
 * !!! The angle is inverse to the effective height !!!
 * Take two degrees to move faster
 */
void doIncreaseHeight() {
    if (sRequestedBodyHeightAngle >= (LIFT_LOWEST_ANGLE + 2)) {
        sRequestedBodyHeightAngle -= 2;
#if defined(LOCAL_INFO)
        printBodyHeight();
#else
        setBodyHeight();
#endif
        if (sCurrentlyRunningAction == ACTION_TYPE_STOP) {
            setLiftServosToBodyHeight();
        }
    }
}

void doDecreaseHeight() {
    if (sRequestedBodyHeightAngle <= (LIFT_HIGHEST_ANGLE - 2)) {
        sRequestedBodyHeightAngle += 2;
#if defined(LOCAL_INFO)
        printBodyHeight();
#else
        setBodyHeight();
#endif
        if (sCurrentlyRunningAction == ACTION_TYPE_STOP) {
            setLiftServosToBodyHeight();
        }
    }
}

#include "LocalDebugLevelEnd.h"

#endif // _QUADRUPED_CONTROL_COMMANDS_HPP
