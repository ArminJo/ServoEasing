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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
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
    sCurrentlyRunningAction = ACTION_TYPE_TEST;
#if defined(QUADRUPED_ENABLE_RTTTL)
    doRandomMelody(); // Use melody as sample test
#else
    sCurrentlyRunningAction = ACTION_TYPE_STOP; // action = stop ends the melody
#endif
}

/*
 * Center, lean left and right lean all 4 directions and twist a random angle. Ends with a wave.
 */
void __attribute__((weak)) doDance() {
    sCurrentlyRunningAction = ACTION_TYPE_DANCE;
#if defined(INFO)
    Serial.print(F("Dance"));
    printQuadrupedServoSpeed();
#endif

    centerServos();
    QUADRUPED_RETURN_IF_STOP;
    /*
     * Move down and up and back to current height
     */
    doAttention();
    QUADRUPED_RETURN_IF_STOP;

    for (int i = 0; i < 1; ++i) {
        doLeanLeft();
        QUADRUPED_RETURN_IF_STOP;
        doLeanRight();
        QUADRUPED_RETURN_IF_STOP;
    }
    for (int i = 0; i < 3; ++i) {
        doLeanLeft();
        QUADRUPED_RETURN_IF_STOP;
        // lean back
        doLeanBack();
        QUADRUPED_RETURN_IF_STOP;
        uint8_t tTwistAngle = random(30, 60);
        basicTwist(tTwistAngle);
        QUADRUPED_RETURN_IF_STOP;
        basicTwist(tTwistAngle, false);
        QUADRUPED_RETURN_IF_STOP;

        doLeanRight();
        QUADRUPED_RETURN_IF_STOP;
        // lean front
        doLeanFront();
        QUADRUPED_RETURN_IF_STOP;
    }

    doWave();
    QUADRUPED_RETURN_IF_STOP;
    centerServos();
    sCurrentlyRunningAction = ACTION_TYPE_STOP;
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

    delayAndCheckForLowVoltageAndStop(1000);
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

    delayAndCheckForLowVoltageAndStop(1000);
    QUADRUPED_RETURN_IF_STOP;

    centerServos();
    sCurrentlyRunningAction = ACTION_TYPE_STOP;
}

void __attribute__((weak)) doCenterServos() {
#if defined(INFO)
    Serial.print(F("Center"));
    printQuadrupedServoSpeed();
#endif
    centerServos();
}

void __attribute__((weak)) doBow() {
#if defined(INFO)
    Serial.print(F("Bow"));
    printQuadrupedServoSpeed();
#endif
    centerServos();

    delayAndCheckForLowVoltageAndStop(300);
    QUADRUPED_RETURN_IF_STOP;

    // Lift front legs
    ServoEasing::ServoEasingArray[FRONT_LEFT_LIFT]->setEaseTo(LIFT_LOWEST_ANGLE, sQuadrupedServoSpeed);
    ServoEasing::ServoEasingArray[FRONT_RIGHT_LIFT]->startEaseToD(LIFT_LOWEST_ANGLE,
            ServoEasing::ServoEasingArray[FRONT_LEFT_LIFT]->mMillisForCompleteMove);
    updateAndCheckInputAndWaitForAllServosToStop();

    delayAndCheckForLowVoltageAndStop(300);
    QUADRUPED_RETURN_IF_STOP;

    centerServos();
}

void __attribute__((weak)) doTwist() {
#if defined(INFO)
    Serial.print(F("Twist"));
    printQuadrupedServoSpeed();
#endif
    basicTwist(30, true);
    QUADRUPED_RETURN_IF_STOP;
    basicTwist(30, false);
    QUADRUPED_RETURN_IF_STOP;

    centerServos();
}

void __attribute__((weak)) doLeanLeft() {
    sCurrentlyRunningAction = ACTION_TYPE_LEAN;

#if defined(INFO)
    Serial.print(F("Lean left"));
    printQuadrupedServoSpeed();
#endif
    setLiftServos(LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_LOWEST_ANGLE, LIFT_LOWEST_ANGLE);
    sCurrentlyRunningAction = ACTION_TYPE_STOP;
}

void __attribute__((weak)) doLeanRight() {
#if defined(INFO)
    Serial.print(F("Lean right"));
    printQuadrupedServoSpeed();
#endif
    setLiftServos(LIFT_LOWEST_ANGLE, LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE);
    sCurrentlyRunningAction = ACTION_TYPE_STOP;
}

void __attribute__((weak)) doLeanBack() {
#if defined(INFO)
    Serial.print(F("Lean back"));
    printQuadrupedServoSpeed();
#endif
    setLiftServos(LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_LOWEST_ANGLE);
    sCurrentlyRunningAction = ACTION_TYPE_STOP;
}

void __attribute__((weak)) doLeanFront() {
#if defined(INFO)
    Serial.print(F("Lean front"));
    printQuadrupedServoSpeed();
#endif
    setLiftServos(LIFT_HIGHEST_ANGLE, LIFT_LOWEST_ANGLE, LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE);
    sCurrentlyRunningAction = ACTION_TYPE_STOP;
}

void __attribute__((weak)) doTurnRight() {
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveTurn();
}

void __attribute__((weak)) doTurnLeft() {
    sMovingDirection = MOVE_DIRECTION_LEFT;
    moveTurn();
}

void __attribute__((weak)) doTrot() {
#if defined(INFO)
    Serial.println(F("Trot"));
    printQuadrupedServoSpeed();
#endif
    moveTrot();
}

/*
 * Set servo positions and speeds required to moveCreep forward one step
 * Start with move to Y position with right legs together
 */
void __attribute__((weak)) doCreepForward() {
#if defined(INFO)
    Serial.print(F("Creep forward"));
    printQuadrupedServoSpeed();
#endif
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    moveCreep();
}

/*
 * A movement to get attention, that quadruped may be switched off
 * Move down and up and back to starting height
 */
void __attribute__((weak)) doAttention() {
    sCurrentlyRunningAction = ACTION_TYPE_ATTENTION;
    // Move down and up and back to starting height
    setLiftServos(LIFT_HIGHEST_ANGLE);
    QUADRUPED_RETURN_IF_STOP;
    setLiftServos(LIFT_LOWEST_ANGLE);
    QUADRUPED_RETURN_IF_STOP;
    setLiftServos(sRequestedBodyHeightAngle);
    sCurrentlyRunningAction = ACTION_TYPE_STOP;
}

void __attribute__((weak)) doQuadrupedAutoMove() {
    uint16_t tOriginalSpeed = sQuadrupedServoSpeed;

#if defined(INFO)
    Serial.println(F("Start auto move sequence"));
#endif
    centerServos();
    QUADRUPED_RETURN_IF_STOP;

    // Move down and up and back to starting height
    doAttention();
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
    delayAndCheckForLowVoltageAndStop(2000);
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
    delayAndCheckForLowVoltageAndStop(2000);
    QUADRUPED_RETURN_IF_STOP;

    // trot right
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveTrot(8);
    delayAndCheckForLowVoltageAndStop(2000);
    QUADRUPED_RETURN_IF_STOP;

    // turn left
    centerServos();
    sMovingDirection = MOVE_DIRECTION_LEFT;
    moveTurn(12);
    QUADRUPED_RETURN_IF_STOP;

    // center
    centerServos();

    // restore speed
    setQuadrupedServoSpeed(tOriginalSpeed);

#if defined(INFO)
    Serial.println(F("Stop auto move sequence and wait 10 seconds"));
#endif
    delayAndCheckForLowVoltageAndStop(10000);
}
/*************************
 * Instant Commands
 *************************/
void __attribute__((weak)) doStop() {
    sCurrentlyRunningAction = ACTION_TYPE_STOP; // this also stops NeoPatterns
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

void __attribute__((weak)) doSetDirectionForward() {
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    moveCreep();
}

void __attribute__((weak)) doSetDirectionBack() {
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
    moveCreep();
}

void __attribute__((weak)) doSetDirectionLeft() {
    sMovingDirection = MOVE_DIRECTION_LEFT;
    moveTurn();
}

void __attribute__((weak)) doSetDirectionRight() {
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveTurn();
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
