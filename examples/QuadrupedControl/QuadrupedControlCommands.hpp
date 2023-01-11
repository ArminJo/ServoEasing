/*
 * QuadrupedIRCommands.cpp
 *
 * Contains all the IR command functions available.
 * All functions have the prefix __attribute__((weak)) in order to enable easy overwriting with own functions.
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of QuadrupedControl https://github.com/ArminJo/QuadrupedControl.
 *
 *  QuadrupedControl is free software: you can redistribute it and/or modify
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

//#define INFO // activate this to see serial info output

/******************************************
 * The Commands to execute
 ******************************************/
void __attribute__((weak)) doTest() {
    // to be overwritten by user function
#if defined(INFO)
    Serial.print(F("Test TableMove"));
    printQuadrupedServoSpeed();
#endif
#if defined(QUADRUPED_ENABLE_RTTTL)
    doRandomMelody();
#else
    twist(45, 2);
#endif
}

void __attribute__((weak)) doCenterServos() {
#if defined(INFO)
    Serial.print(F("Center"));
    printQuadrupedServoSpeed();
#endif
    centerServos();
}

void __attribute__((weak)) doTwist() {
#if defined(INFO)
    Serial.print(F("Twist"));
    printQuadrupedServoSpeed();
#endif
    twist(30, 2);
}

void __attribute__((weak)) doTrot() {
#if defined(INFO)
    Serial.print(F("Trot"));
    printQuadrupedServoSpeed();
#endif
    moveTrot();
}

void __attribute__((weak)) doCreep() {
#if defined(INFO)
    Serial.print(F("Creep"));
    printQuadrupedServoSpeed();
#endif
    moveCreep();
}

void __attribute__((weak)) doTurn() {
#if defined(INFO)
    Serial.print(F("Turn"));
    printQuadrupedServoSpeed();
#endif
    moveTurn();
}

/*
 * A movement to get attention, that quadruped may be switched off
 * Move down and up and back to starting height
 */
void __attribute__((weak)) doAttention() {
#if defined(INFO)
    Serial.println(F("Start attention"));
#endif
//    sCurrentlyRunningAction = ACTION_TYPE_ATTENTION;
    // Move down and up and back to starting height
    downAndUp(1);
}

void __attribute__((weak)) doWave() {
    sCurrentlyRunningAction = ACTION_TYPE_WAVE;

#if defined(INFO)
    Serial.print(F("Wave 3 times with right leg"));
    printQuadrupedServoSpeed();
#endif

    uint8_t tRequestedBodyHeightAngle = sRequestedBodyHeightAngle; // sRequestedBodyHeightAngle is volatile
    // move front left and back right leg 10 degree forward to avoid falling to front if lifting the front right leg
    setAllServos(80, 90, 100, 90, tRequestedBodyHeightAngle, tRequestedBodyHeightAngle, tRequestedBodyHeightAngle,
            tRequestedBodyHeightAngle);
    QUADRUPED_RETURN_IF_STOP;

    // move all legs up, except front left -> front right lifts from the ground
    setLiftServos(LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE);

    delayAndCheckForStopByIR(1000);
    QUADRUPED_RETURN_IF_STOP;

    ServoEasing::ServoEasingArray[FRONT_RIGHT_PIVOT]->setEasingType(EASE_QUADRATIC_IN_OUT);

    // wave with the front right leg
    for (uint_fast8_t i = 0; i < 3; ++i) {
        moveOneServoAndCheckInputAndWait(FRONT_RIGHT_PIVOT, 135, sQuadrupedServoSpeed * 2);
        QUADRUPED_RETURN_IF_STOP;

        moveOneServoAndCheckInputAndWait(FRONT_RIGHT_PIVOT, 45, sQuadrupedServoSpeed * 2);
        QUADRUPED_RETURN_IF_STOP;
    }
    ServoEasing::ServoEasingArray[FRONT_RIGHT_PIVOT]->setEasingType(EASE_LINEAR);

    delayAndCheckForStopByIR(1000);
    QUADRUPED_RETURN_IF_STOP;

    centerServos();
    setActionToStop();
}

/*
 * Center, lean left and right lean all 4 directions and twist a random angle. Ends with a wave.
 */
void __attribute__((weak)) doDance() {
    sCurrentlyRunningCombinedAction = ACTION_TYPE_DANCE;
#if defined(INFO)
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
    sCurrentlyRunningCombinedAction = ACTION_TYPE_STOP;
}

/**
 * A demo move for tables - 2 1/2 minutes
 */
void __attribute__((weak)) doQuadrupedDemoMove() {
    uint16_t tOriginalSpeed = sQuadrupedServoSpeed;

#if defined(INFO)
    Serial.println(F("Start demo move sequence"));
#endif
    sCurrentlyRunningCombinedAction = ACTION_TYPE_DEMO_MOVE;

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

    sCurrentlyRunningCombinedAction = ACTION_TYPE_STOP;
    // restore speed
    setQuadrupedServoSpeed(tOriginalSpeed);
}

void __attribute__((weak)) doQuadrupedAutoMove() {
    uint16_t tOriginalSpeed = sQuadrupedServoSpeed;

#if defined(INFO)
    Serial.println(F("Start auto move sequence"));
#endif
    sCurrentlyRunningCombinedAction = ACTION_TYPE_AUTO_MOVE;

    centerServos();
    QUADRUPED_RETURN_IF_STOP;

    // Move down and up and back to starting height
    downAndUp(2);
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
    delayAndCheckForStopByIR(2000);
    QUADRUPED_RETURN_IF_STOP;

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
    delayAndCheckForStopByIR(2000);
    QUADRUPED_RETURN_IF_STOP;

    // trot right
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveTrot(8);
    delayAndCheckForStopByIR(2000);
    QUADRUPED_RETURN_IF_STOP;

    // turn left
    centerServos();
    sMovingDirection = MOVE_DIRECTION_LEFT;
    moveTurn(12);
    QUADRUPED_RETURN_IF_STOP;

    // dance
    doDance();
    sCurrentlyRunningCombinedAction = ACTION_TYPE_STOP;

    // restore speed
    setQuadrupedServoSpeed(tOriginalSpeed);
}

/*************************
 * Instant Commands
 *************************/
void __attribute__((weak)) doStop() {
    setActionToStop(); // this also stops NeoPatterns
}

void __attribute__((weak)) doPauseResume() {
    if (sCurrentlyRunningAction != ACTION_TYPE_STOP) {
        if (sCurrentlyRunningAction == ACTION_TYPE_PAUSE) {
#if defined(INFO)
            Serial.print(F("Resume with action="));
            Serial.println(sLastRunningAction);
#endif
            sCurrentlyRunningAction = sLastRunningAction; // restore also action
            resumeWithoutInterruptsAllServos();
//        resumeWithInterruptsAllServos();  // start the interrupts for NeoPatterns if disabled.
        } else {
#if defined(INFO)
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
void __attribute__((weak)) doSetDirectionForward() {
    sMovingDirection = MOVE_DIRECTION_FORWARD;
#if defined(INFO)
    Serial.print(F("Forward"));
    printQuadrupedServoSpeed();
#endif
#if defined(QUADRUPED_HAS_IR_CONTROL)
    // This enables to change also trot direction
    if (sCurrentlyRunningAction != ACTION_TYPE_TROT) {
        // If not doing trot, start a creep
        IRDispatcher.setNextBlockingCommand(COMMAND_CREEP);
    }
#endif
}

void __attribute__((weak)) doSetDirectionBack() {
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
#if defined(QUADRUPED_HAS_IR_CONTROL)
    // This enables to change also trot direction
    if (sCurrentlyRunningAction != ACTION_TYPE_TROT) {
        // If not doing trot, start a creep
        IRDispatcher.setNextBlockingCommand(COMMAND_CREEP);
    }
#endif
}

void __attribute__((weak)) doSetDirectionLeft() {
    sMovingDirection = MOVE_DIRECTION_LEFT;
#if defined(QUADRUPED_HAS_IR_CONTROL)
    // This enables to change also creep and trot direction
    if (sCurrentlyRunningAction != ACTION_TYPE_CREEP && sCurrentlyRunningAction != ACTION_TYPE_TROT) {
        IRDispatcher.setNextBlockingCommand(COMMAND_TURN);
    }
#endif
}

void __attribute__((weak)) doSetDirectionRight() {
    sMovingDirection = MOVE_DIRECTION_RIGHT;
#if defined(QUADRUPED_HAS_IR_CONTROL)
    // This enables to change also creep and trot direction
    if (sCurrentlyRunningAction != ACTION_TYPE_CREEP && sCurrentlyRunningAction != ACTION_TYPE_TROT) {
        IRDispatcher.setNextBlockingCommand(COMMAND_TURN);
    }
#endif
}

/*
 * Increase moving speed by 25%
 */
void __attribute__((weak)) doIncreaseSpeed() {
    sQuadrupedServoSpeed += sQuadrupedServoSpeed / 4;
    if (sQuadrupedServoSpeed > SERVO_MAX_SPEED) {
        sQuadrupedServoSpeed = SERVO_MAX_SPEED;
    }
    setSpeedForAllServos(sQuadrupedServoSpeed);
#if defined(INFO)
    printQuadrupedServoSpeed();
#endif
}

/*
 * Decrease moving speed by 25%
 */
void __attribute__((weak)) doDecreaseSpeed() {
    sQuadrupedServoSpeed -= sQuadrupedServoSpeed / 4;
    if (sQuadrupedServoSpeed < SERVO_MIN_SPEED) {
        sQuadrupedServoSpeed = SERVO_MIN_SPEED;
    }
    setSpeedForAllServos(sQuadrupedServoSpeed);
#if defined(INFO)
    printQuadrupedServoSpeed();
#endif
}

/*
 * !!! The angle is inverse to the effective height !!!
 * Take two degrees to move faster
 */
void __attribute__((weak)) doIncreaseHeight() {
    if (sRequestedBodyHeightAngle > (LIFT_LOWEST_ANGLE + 2)) {
        sRequestedBodyHeightAngle -= 2;
#if defined(INFO)
        printBodyHeight();
#endif
        if (sCurrentlyRunningAction == ACTION_TYPE_STOP) {
            setLiftServosToBodyHeight();
        }
    }
}

void __attribute__((weak)) doDecreaseHeight() {
    if (sRequestedBodyHeightAngle < (LIFT_HIGHEST_ANGLE - 2)) {
        sRequestedBodyHeightAngle += 2;
#if defined(INFO)
        printBodyHeight();
#endif
        if (sCurrentlyRunningAction == ACTION_TYPE_STOP) {
            setLiftServosToBodyHeight();
        }
    }
}

#endif // _QUADRUPED_CONTROL_COMMANDS_HPP
