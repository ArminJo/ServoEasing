/*
 * QuadrupedIRCommands.cpp
 *
 * Contains all the IR command functions available.
 * All functions have the prefix in order to enable easy overwriting with own functions.
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

#ifndef _QUADRUPED_CONTROL_COMMANDS_HPP
#define _QUADRUPED_CONTROL_COMMANDS_HPP

#include <Arduino.h>

#include "QuadrupedBasicMovements.h"
#include "QuadrupedControlCommands.h"
#include "QuadrupedServoControl.hpp"
#include "QuadrupedBasicMovements.hpp"

// This block must be located after the includes of other *.hpp files
//#define LOCAL_INFO  // This enables info output only for this file
#include "LocalDebugLevelStart.h"

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

    uint8_t tRequestedBodyHeightAngle = sRequestedBodyHeightAngle; // sRequestedBodyHeightAngle is volatile
    // move front left and back right leg 10 degree forward to avoid falling to front if lifting the front right leg
    moveAllServosToNextPositionsAndCheckInputAndWait(80, 90, 100, 90, tRequestedBodyHeightAngle, tRequestedBodyHeightAngle, tRequestedBodyHeightAngle,
            tRequestedBodyHeightAngle);
    QUADRUPED_RETURN_IF_STOP;

    // move all legs up, except front left -> front right lifts from the ground
    moveLiftServosToNextPositionsAndCheckInputAndWait(LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE);

#if defined(APPLICATION_DELAY_AND_CHECK_AVAILABLE)
    delayAndCheckByApplication(1000);
    QUADRUPED_RETURN_IF_STOP;
#else
    delay(1000); // wait 2 second then start new move
#endif

    ServoEasing::ServoEasingArray[FRONT_RIGHT_PIVOT]->setEasingType(EASE_QUADRATIC_IN_OUT);

    // wave with the front right leg
    for (uint_fast8_t i = 0; i < 3; ++i) {
        moveOneServoAndCheckInputAndWait(FRONT_RIGHT_PIVOT, 135, sQuadrupedServoSpeed * 2);
        QUADRUPED_RETURN_IF_STOP;

        moveOneServoAndCheckInputAndWait(FRONT_RIGHT_PIVOT, 45, sQuadrupedServoSpeed * 2);
        QUADRUPED_RETURN_IF_STOP;
    }
    ServoEasing::ServoEasingArray[FRONT_RIGHT_PIVOT]->setEasingType(EASE_LINEAR);

#if defined(APPLICATION_DELAY_AND_CHECK_AVAILABLE)
    delayAndCheckByApplication(1000);
    QUADRUPED_RETURN_IF_STOP;
#else
    delay(1000); // wait 2 second then start new move
#endif

    centerServos();
    setActionToStop();
}

/*
 * Center, lean left and right lean all 4 directions and twist a random angle. Ends with a wave.
 */
void doDance() {
    sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_DANCE;
#if defined(LOCAL_INFO)
    Serial.print(F("Dance"));
    printQuadrupedServoSpeed();
#endif

    centerServos();
    QUADRUPED_RETURN_IF_STOP;
    /*
     * Move down and up and back to current height
     */
    downAndUp(2);
    QUADRUPED_RETURN_IF_STOP;

    for (int i = 0; i < 1; ++i) {
        lean(MOVE_DIRECTION_LEFT);
        QUADRUPED_RETURN_IF_STOP;
        lean(MOVE_DIRECTION_RIGHT);
        QUADRUPED_RETURN_IF_STOP;
    }
    for (int i = 0; i < 3; ++i) {
        lean(MOVE_DIRECTION_LEFT);
        QUADRUPED_RETURN_IF_STOP;
        // lean back
        lean(MOVE_DIRECTION_BACKWARD);
        QUADRUPED_RETURN_IF_STOP;
        twist(60, 1);
        QUADRUPED_RETURN_IF_STOP;

        lean(MOVE_DIRECTION_RIGHT);
        QUADRUPED_RETURN_IF_STOP;
        // lean front
        lean(MOVE_DIRECTION_FORWARD);
        QUADRUPED_RETURN_IF_STOP;
    }

    doWave();
    centerServos();
    sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_STOP;
}

/**
 * A demo move for tables - 2 1/2 minutes
 */
void doQuadrupedDemoMove() {
    uint16_t tOriginalSpeed = sQuadrupedServoSpeed;
    sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_DEMO_MOVE;

#if defined(LOCAL_INFO)
    Serial.print(F("Start demo move sequence"));
#endif
    setQuadrupedServoSpeed(140);
    centerServos();
    // Move down and up and back to starting height
    downAndUp(2);
    QUADRUPED_RETURN_IF_STOP;

    // twist
    twist(30, 2);
    QUADRUPED_RETURN_IF_STOP;

    /*
     * creep forward turn and creep back sideways
     */
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    moveCreep(3);
    QUADRUPED_RETURN_IF_STOP;

    // turn 90 degree right, look right
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveTurn(6);
    QUADRUPED_RETURN_IF_STOP;

    // creep right backward, look right
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveCreep(3);
    QUADRUPED_RETURN_IF_STOP;

    /*
     * Now we are back at start position
     * center and twist
     * Creep forward turn forward and creep back sideways
     */
    centerServos();
    // twist
    twist(30, 2);
    QUADRUPED_RETURN_IF_STOP;

    // creep forward, look right
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    moveCreep(3);
    QUADRUPED_RETURN_IF_STOP;

    // turn 90 degree left, look forward
    sMovingDirection = MOVE_DIRECTION_LEFT;
    moveTurn(9);
    QUADRUPED_RETURN_IF_STOP;

    // creep left, look forward
    sMovingDirection = MOVE_DIRECTION_LEFT;
    moveCreep(3);
    QUADRUPED_RETURN_IF_STOP;

    /*
     * Now we are back at start position
     * dance
     */
    doDance();

    sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_STOP;
    // restore speed
    setQuadrupedServoSpeed(tOriginalSpeed);
}

/*
 * Auto move, needs more space than demo move.
 * It runs only once 40 seconds after boot if no input was detected.
 */
void doQuadrupedAutoMove() {
    uint16_t tOriginalSpeed = sQuadrupedServoSpeed;

#if defined(LOCAL_INFO)
    Serial.println(F("Start auto move sequence"));
#endif
    sCurrentlyRunningCombinedAction = COMBINED_ACTION_TYPE_AUTO_MOVE;

    centerServos();
    QUADRUPED_RETURN_IF_STOP;

    // Move down and up and back to starting height
    downAndUp(4); // Do it 4 times to distinguish it from demo move
    QUADRUPED_RETURN_IF_STOP;

    // creep forward slow
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    moveCreep(2);
    QUADRUPED_RETURN_IF_STOP;

    // creep forward fast
    setQuadrupedServoSpeed(260);
    moveCreep(6);
    QUADRUPED_RETURN_IF_STOP;

    // creep right fast
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveCreep(8);
    QUADRUPED_RETURN_IF_STOP;

    // creep left fast
    sMovingDirection = MOVE_DIRECTION_LEFT;
    moveCreep(4);
#if defined(APPLICATION_DELAY_AND_CHECK_AVAILABLE)
    delayAndCheckByApplication(2000);
    QUADRUPED_RETURN_IF_STOP;
#else
    delay(2000); // wait 2 second then start new move
#endif

    // Dance
    setQuadrupedServoSpeed(200);
    doDance();
    QUADRUPED_RETURN_IF_STOP;

    // turn right
    sMovingDirection = MOVE_DIRECTION_RIGHT;
//    triggerNeoPatterns();
    moveTurn(11);
    QUADRUPED_RETURN_IF_STOP;

    // trot forward
    setQuadrupedServoSpeed(160);
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    centerServos();
    moveTrot(10);
    QUADRUPED_RETURN_IF_STOP;

    // trot back
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
    moveTrot(8);
#if defined(APPLICATION_DELAY_AND_CHECK_AVAILABLE)
    delayAndCheckByApplication(2000);
    QUADRUPED_RETURN_IF_STOP;
#else
    delay(2000); // wait 2 second then start new move
#endif

    // trot right
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveTrot(8);
#if defined(APPLICATION_DELAY_AND_CHECK_AVAILABLE)
    delayAndCheckByApplication(2000);
    QUADRUPED_RETURN_IF_STOP;
#else
    delay(2000); // wait 2 second then start new move
#endif

    // turn left
    centerServos();
    sMovingDirection = MOVE_DIRECTION_LEFT;
    moveTurn(12);
    QUADRUPED_RETURN_IF_STOP;

    // dance
    doDance();
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
    if (sRequestedBodyHeightAngle > (LIFT_LOWEST_ANGLE + 2)) {
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
    if (sRequestedBodyHeightAngle < (LIFT_HIGHEST_ANGLE - 2)) {
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
