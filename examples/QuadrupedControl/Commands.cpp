/*
 * Commands.cpp
 *
 * Contains all the IR command functions available.
 * All functions have the prefix __attribute__((weak)) in order to enable easy overwriting with own functions.
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

uint8_t sActionType;

/******************************************
 * The Commands to execute
 ******************************************/
void __attribute__((weak)) doTest() {
    // to be overwritten by user function
}

void __attribute__((weak)) doBeep() {
    // to be overwritten by user function
}

/*
 * Center, lean left and right lean all 4 directions and twist a random angle. Ends with a wave.
 */
void __attribute__((weak)) doDance() {
    Serial.print(F("Dance."));
    printSpeed();

    centerServos();
    RETURN_IF_STOP;
    /*
     * Move down and up and back to current height
     */
    doAttention();
    RETURN_IF_STOP;

    for (int i = 0; i < 1; ++i) {
        doLeanLeft();
        RETURN_IF_STOP;
        doLeanRight();
        RETURN_IF_STOP;
    }
    for (int i = 0; i < 3; ++i) {
        doLeanLeft();
        RETURN_IF_STOP;
        // lean back
        doLeanBack();
        RETURN_IF_STOP;
        uint8_t tTwistAngle = random(30, 60);
        basicTwist(tTwistAngle);
        RETURN_IF_STOP;
        basicTwist(tTwistAngle, false);
        RETURN_IF_STOP;

        doLeanRight();
        RETURN_IF_STOP;
        // lean front
        doLeanFront();
        RETURN_IF_STOP;
    }

    doWave();
    RETURN_IF_STOP;
    centerServos();
}

void __attribute__((weak)) doWave() {
    Serial.print(F("Wave 3 times with right leg."));
    printSpeed();

    // move front left and back right leg 10 degree forward to avoid falling to front if lifting the front right leg
    setAllServos(80, 90, 100, 90, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle);
    RETURN_IF_STOP;

    // move all legs up, except front left -> front right lifts from the ground
    setLiftServos(LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE);
    RETURN_IF_STOP;

    delayAndCheck(1000);
    RETURN_IF_STOP;

    sServoArray[FRONT_RIGHT_PIVOT]->setEasingType(EASE_QUADRATIC_IN_OUT);

    // wave with the front right leg
    for (uint8_t i = 0; i < 3; ++i) {
        moveOneServoAndCheckInputAndWait(FRONT_RIGHT_PIVOT, 135, sServoSpeed * 2);
        RETURN_IF_STOP;

        moveOneServoAndCheckInputAndWait(FRONT_RIGHT_PIVOT, 45, sServoSpeed * 2);
        RETURN_IF_STOP;
    }
    sServoArray[FRONT_RIGHT_PIVOT]->setEasingType(EASE_LINEAR);

    delayAndCheck(1000);
    RETURN_IF_STOP;

    centerServos();
}

void __attribute__((weak)) doCenterServos() {
    Serial.print(F("Center."));
    printSpeed();
    centerServos();
}

void __attribute__((weak)) doBow() {
    Serial.print(F("Bow."));
    printSpeed();
    centerServos();
    RETURN_IF_STOP;

    delayAndCheck(300);
    RETURN_IF_STOP;

    // Lift front legs
    sServoArray[FRONT_LEFT_LIFT]->setEaseTo(LIFT_LOWEST_ANGLE, sServoSpeed);
    sServoArray[FRONT_RIGHT_LIFT]->startEaseToD(LIFT_LOWEST_ANGLE, sServoArray[FRONT_LEFT_LIFT]->mMillisForCompleteMove);
    updateAndCheckInputAndWaitForAllServosToStop();
    RETURN_IF_STOP;

    delayAndCheck(300);
    RETURN_IF_STOP;

    centerServos();
}

void __attribute__((weak)) doTwist() {
    Serial.print(F("Twist."));
    printSpeed();
    basicTwist(30, true);
    RETURN_IF_STOP;
    basicTwist(30, false);
    RETURN_IF_STOP;

    centerServos();
}

void __attribute__((weak)) doLeanLeft() {
    Serial.print(F("Lean left."));
    printSpeed();
    setLiftServos(LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_LOWEST_ANGLE, LIFT_LOWEST_ANGLE);
}

void __attribute__((weak)) doLeanRight() {
    Serial.print(F("Lean right."));
    printSpeed();
    setLiftServos(LIFT_LOWEST_ANGLE, LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE);
}

void __attribute__((weak)) doLeanBack() {
    Serial.print(F("Lean back."));
    printSpeed();
    setLiftServos(LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_LOWEST_ANGLE);
}

void __attribute__((weak)) doLeanFront() {
    Serial.print(F("Lean front."));
    printSpeed();
    setLiftServos(LIFT_HIGHEST_ANGLE, LIFT_LOWEST_ANGLE, LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE);
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
    Serial.println(F("Trot."));
    printSpeed();
    moveTrot();
}

/*
 * Set servo positions and speeds needed to moveCreep forward one step
 * Start with move to Y position with right legs together
 */
void __attribute__((weak)) doCreepForward() {
    Serial.print(F("Creep forward."));
    printSpeed();
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    moveCreep();
}

/*
 * Start from same position as forward
 */
void __attribute__((weak)) doCreepBack() {
    Serial.print(F("Creep back."));
    printSpeed();
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
    moveCreep();
}

/*
 * A movement to get attention, that quadruped may be switched off
 * Move down and up and back to starting height
 */
void __attribute__((weak)) doAttention() {
    Serial.println(F("Move to get attention"));
//    doBeep();
    // Move down and up and back to starting height
    setLiftServos(LIFT_HIGHEST_ANGLE);
    RETURN_IF_STOP;
    setLiftServos(LIFT_LOWEST_ANGLE);
    RETURN_IF_STOP;
    setLiftServos(sBodyHeightAngle);
}

void __attribute__((weak)) doAutoMove() {
    uint16_t tOriginalSpeed = sServoSpeed;

    centerServos();
    RETURN_IF_STOP;

    // Move down and up and back to starting height
    doAttention();
    RETURN_IF_STOP;

    // creep forward slow
    sMovingDirection = MOVE_DIRECTION_FORWARD;
//    triggerNeoPatterns();
    moveCreep(2);
    RETURN_IF_STOP;

    // creep forward fast
    setSpeed(260);
    moveCreep(6);
    RETURN_IF_STOP;

    // creep right fast
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveCreep(8);
    RETURN_IF_STOP;

    // creep left fast
    sMovingDirection = MOVE_DIRECTION_LEFT;
    moveCreep(4);
    RETURN_IF_STOP;

    delayAndCheck(2000);

    setSpeed(200);
    doDance();
    RETURN_IF_STOP;

    // turn right
    sMovingDirection = MOVE_DIRECTION_RIGHT;
//    triggerNeoPatterns();
    moveTurn(11);
    RETURN_IF_STOP;

    // trot forward
    setSpeed(160);
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    centerServos();
    moveTrot(10);
    RETURN_IF_STOP;

    // trot back
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
    moveTrot(8);
    RETURN_IF_STOP;

    delayAndCheck(2000);

    // trot right
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveTrot(8);
    RETURN_IF_STOP;

    delayAndCheck(2000);

    // turn right
    centerServos();
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveTurn(12);
    RETURN_IF_STOP;

    // restore speed
    setSpeed(tOriginalSpeed);

    delayAndCheck(10000);
}
/*************************
 * Instant Commands
 *************************/
void __attribute__((weak)) doStop() {
#if defined(QUADRUPED_IR_CONTROL)
    sRequestToStopReceived = true;
    sActionType = ACTION_TYPE_STOP;
#endif
}

void __attribute__((weak)) doSetDirectionForward() {
    sMovingDirection = MOVE_DIRECTION_FORWARD;
}

void __attribute__((weak)) doSetDirectionBack() {
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
}

void __attribute__((weak)) doSetDirectionLeft() {
    sMovingDirection = MOVE_DIRECTION_LEFT;
}

void __attribute__((weak)) doSetDirectionRight() {
    sMovingDirection = MOVE_DIRECTION_RIGHT;
}

/*
 * Increase moving speed by 25%
 */
void __attribute__((weak)) doIncreaseSpeed() {
    sServoSpeed += sServoSpeed / 4;
    if (sServoSpeed > 400) {
        sServoSpeed = 400;
    }
    setSpeedForAllServos(sServoSpeed);
    Serial.print(sServoSpeed);
}

/*
 * Decrease moving speed by 25%
 */
void __attribute__((weak)) doDecreaseSpeed() {
    if (sServoSpeed > 2) {
        sServoSpeed -= sServoSpeed / 4;
        if (sServoSpeed < 4) {
            sServoSpeed = 4;
        }
    }
    setSpeedForAllServos(sServoSpeed);
    Serial.print(sServoSpeed);
}

/*
 * !!! The angle is inverse to the effective height !!!
 * Take two degrees to move faster
 */
void __attribute__((weak)) doIncreaseHeight() {
    if (sBodyHeightAngle > (LIFT_LOWEST_ANGLE + 2)) {
        sBodyHeightAngle -= 2;
        convertBodyHeightAngleToHeight();
#if defined(QUADRUPED_IR_CONTROL)
        if (!sExecutingMainCommand) {
            setLiftServosToBodyHeight();
        }
#else
        setLiftServosToBodyHeight();
#endif
    }
}

void __attribute__((weak)) doDecreaseHeight() {
    if (sBodyHeightAngle < (LIFT_HIGHEST_ANGLE - 2)) {
        sBodyHeightAngle += 2;
        convertBodyHeightAngleToHeight();
#if defined(QUADRUPED_IR_CONTROL)
        if (!sExecutingMainCommand) {
            setLiftServosToBodyHeight();
        }
#else
        setLiftServosToBodyHeight();
#endif
    }
}

void convertBodyHeightAngleToHeight() {
    sBodyHeight = map(sBodyHeightAngle, LIFT_HIGHEST_ANGLE, LIFT_LOWEST_ANGLE, 0, 255);
    Serial.print(F("sBodyHeight="));
    Serial.println(sBodyHeight);
}

/*
 * Special calibration command
 */

/*
 * Signals which leg is to be calibrated
 */
void signalLeg(uint8_t aPivotServoIndex) {
    sServoArray[aPivotServoIndex + LIFT_SERVO_OFFSET]->easeTo(LIFT_HIGHEST_ANGLE, 60);
    sServoArray[aPivotServoIndex]->easeTo(90, 60);
    sServoArray[aPivotServoIndex + LIFT_SERVO_OFFSET]->easeTo(90, 60);
}

#if defined(QUADRUPED_IR_CONTROL)
/*
 * includes needed only for doCalibration
 */
#include "IRCommandMapping.h" // for COMMAND_*
#include "QuadrupedServoControl.h"

/*
 * Changes the servo calibration values in EEPROM.
 * Starts with front left i.e. sServoArray[0,1] and switches to the next leg with the COMMAND_ENTER
 */
void doCalibration() {
    uint8_t tPivotServoIndex = 0; // start with front left i.e. sServoArray[0]
    bool tGotExitCommand = false;
    resetServosTo90Degree();
    delay(500);
    signalLeg(tPivotServoIndex);
    Serial.println(F("Entered calibration. Use the forward/backward right/left buttons to set the servo position to 90 degree."));
    Serial.println(F("Use enter/OK button to go to next leg. Values are stored at receiving a different button or after 4th leg."));

    while (!tGotExitCommand) {
        unsigned long tIRCode = getIRCommand(true);
        printIRCommandString(tIRCode);

        switch (tIRCode) {
        case COMMAND_RIGHT:
            sServoTrimAngles[tPivotServoIndex]++;
            sServoArray[tPivotServoIndex]->setTrim(sServoTrimAngles[tPivotServoIndex]);
            break;
        case COMMAND_LEFT:
            sServoTrimAngles[tPivotServoIndex]--;
            sServoArray[tPivotServoIndex]->setTrim(sServoTrimAngles[tPivotServoIndex]);
            break;
        case COMMAND_FORWARD:
            sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]++;
            sServoArray[tPivotServoIndex + LIFT_SERVO_OFFSET]->setTrim(sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]);
            break;
        case COMMAND_BACKWARD:
            sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]--;
            sServoArray[tPivotServoIndex + LIFT_SERVO_OFFSET]->setTrim(sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]);
            break;
        case COMMAND_ENTER:
            // show 135 and 45 degree positions
            sServoArray[tPivotServoIndex]->easeTo(135, 100);
            delay(2000);
            sServoArray[tPivotServoIndex]->easeTo(45, 100);
            delay(2000);
            sServoArray[tPivotServoIndex]->easeTo(90, 100);
            tPivotServoIndex += SERVOS_PER_LEG;
            eepromWriteServoTrim();
            if (tPivotServoIndex >= NUMBER_OF_SERVOS) {
                tGotExitCommand = true;
            } else {
                signalLeg(tPivotServoIndex);
            }
            // remove a repeat command
            getIRCommand(false);
            break;
        case COMMAND_CALIBRATE:
            // repeated command here
            break;
        default:
            eepromWriteServoTrim();
            tGotExitCommand = true;
            break;
        }
        Serial.print(F("ServoTrimAngles["));
        Serial.print(tPivotServoIndex);
        Serial.print(F("]="));
        Serial.print(sServoTrimAngles[tPivotServoIndex]);
        Serial.print(F(" ["));
        Serial.print(tPivotServoIndex + LIFT_SERVO_OFFSET);
        Serial.print(F("]="));
        Serial.println(sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]);
        sServoArray[tPivotServoIndex]->print(&Serial);
        sServoArray[tPivotServoIndex + LIFT_SERVO_OFFSET]->print(&Serial);
        delay(200);
    }
}
#endif
