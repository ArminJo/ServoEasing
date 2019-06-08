/*
 * Commands.cpp
 *
 * Contains all the IR command functions available.
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

#include "IRCommandDispatcher.h"

#include "QuadrupedServoControl.h"

/******************************************
 * The Commands to execute
 ******************************************/

/*
 * Center, lean left and right lean all 4 directions and twist. Ends with a wave.
 */
void doDance() {
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
        setLiftServos(LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MIN_ANGLE);
        RETURN_IF_STOP;
        uint8_t tTwistAngle = random(15, 40);
        basicTwist(tTwistAngle);
        RETURN_IF_STOP;
        basicTwist(tTwistAngle, false);
        RETURN_IF_STOP;

        doLeanRight();
        RETURN_IF_STOP;
        // lean front
        setLiftServos(LIFT_MAX_ANGLE, LIFT_MIN_ANGLE, LIFT_MIN_ANGLE, LIFT_MAX_ANGLE);
        RETURN_IF_STOP;
    }

    doWave();
    RETURN_IF_STOP;
    centerServos();
}

void doWave() {
    Serial.print(F("Wave 3 times with right leg."));
    printSpeed();

    setAllServos(80, 90, 100, 90, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle);
    RETURN_IF_STOP;

    setLiftServos(LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE);
    RETURN_IF_STOP;

    delayAndCheckIRInput(1000);
    RETURN_IF_STOP;

    sServoArray[FRONT_RIGHT_PIVOT]->setEasingType(EASE_QUADRATIC_IN_OUT);

    for (uint8_t i = 0; i < 3; ++i) {
        moveOneServoAndCheckInputAndWait(FRONT_RIGHT_PIVOT, 135, sServoSpeed * 2);
        RETURN_IF_STOP;

        moveOneServoAndCheckInputAndWait(FRONT_RIGHT_PIVOT, 45, sServoSpeed * 2);
        RETURN_IF_STOP;
    }
    delayAndCheckIRInput(1000);
    RETURN_IF_STOP;

    centerServos();
}

void doCenterServos() {
    Serial.print(F("Center."));
    printSpeed();
    centerServos();
}

void doBow() {
    Serial.print(F("Bow."));
    printSpeed();
    centerServos();
    RETURN_IF_STOP;

    delayAndCheckIRInput(300);
    RETURN_IF_STOP;

    // Lift front legs
    sServoArray[FRONT_LEFT_LIFT]->setEaseTo(LIFT_MIN_ANGLE, sServoSpeed);
    sServoArray[FRONT_RIGHT_LIFT]->startEaseToD(LIFT_MIN_ANGLE, sServoArray[FRONT_LEFT_LIFT]->mMillisForCompleteMove);
    updateAndCheckInputAndWaitForAllServosToStop();
    RETURN_IF_STOP;

    delayAndCheckIRInput(300);
    RETURN_IF_STOP;

    centerServos();
}

void doTwist() {
    Serial.print(F("Twist."));
    printSpeed();
    basicTwist(30, true);
    RETURN_IF_STOP;
    basicTwist(30, false);
    RETURN_IF_STOP;

    centerServos();
}

void doLeanLeft() {
    Serial.print(F("Lean left."));
    printSpeed();
    setLiftServos(LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MIN_ANGLE, LIFT_MIN_ANGLE);
}

void doLeanRight() {
    Serial.print(F("Lean right."));
    printSpeed();
    setLiftServos(LIFT_MIN_ANGLE, LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE);
}

void doTurnRight() {
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveTurn();
}

void doTurnLeft() {
    sMovingDirection = MOVE_DIRECTION_LEFT;
    moveTurn();
}

void doTrot() {
    Serial.println(F("Trot."));
    printSpeed();
    moveTrot();
}

/*
 * Set servo positions and speeds needed to moveCreep forward one step
 * Start with move to Y position with right legs together
 */
void doCreepForward() {
    Serial.print(F("Creep forward."));
    printSpeed();
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    moveCreep();
}

/*
 * Start from same position as forward
 */
void doCreepBack() {
    Serial.print(F("Creep back."));
    printSpeed();
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
    moveCreep();
}

/*
 * A movement to get attention, that quadruped may be switched off
 * Move down and up and back to starting height
 */
void doAttention() {
    Serial.println(F("Move to get attention"));
    // Move down and up and back to starting height
    setLiftServos(LIFT_MAX_ANGLE);
    RETURN_IF_STOP;
    setLiftServos(LIFT_MIN_ANGLE);
    RETURN_IF_STOP;
    setLiftServos(sBodyHeightAngle);
}

void internalAutoMove() {
    centerServos();
    RETURN_IF_STOP;

    // Move down and up and back to starting height
    doAttention();
    RETURN_IF_STOP;

    sMovingDirection = MOVE_DIRECTION_FORWARD;
    moveCreep(2);
    RETURN_IF_STOP;

    setSpeedForAllServos(200);
    moveCreep(2);
    RETURN_IF_STOP;

//    setSpeedForAllServos(160);
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveCreep(2);
    RETURN_IF_STOP;

    setSpeedForAllServos(160);
    centerServos();
    moveTrot(2);
    RETURN_IF_STOP;

    sMovingDirection = MOVE_DIRECTION_BACKWARD;
    moveTrot(4);
    RETURN_IF_STOP;

    centerServos();
    moveTurn(8);
    RETURN_IF_STOP;

    delayAndCheckIRInput(2000);
}
/*************************
 * Instant Commands
 *************************/
void doStop() {
    sRequestToStopReceived = true;
}

void doSetDirectionForward() {
    sMovingDirection = MOVE_DIRECTION_FORWARD;
}

void doSetDirectionBack() {
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
}

void doSetDirectionLeft() {
    sMovingDirection = MOVE_DIRECTION_LEFT;
}

void doSetDirectionRight() {
    sMovingDirection = MOVE_DIRECTION_RIGHT;
}

/*
 * Decrease moving speed by 25%
 */
void doIncreaseSpeed() {
    sServoSpeed += sServoSpeed / 4;
    if (sServoSpeed > 0xBF) {
        sServoSpeed = 0xBF;
    }
    setSpeedForAllServos(sServoSpeed);
}

/*
 * Increase moving speed by 25%
 */
void doDecreaseSpeed() {
    if (sServoSpeed > 2) {
        sServoSpeed -= sServoSpeed / 4;
        if (sServoSpeed < 4) {
            sServoSpeed = 4;
        }
    }
    setSpeedForAllServos(sServoSpeed);
}

/*
 * !!! The angle is inverse to the effective height !!!
 * Take two degrees to move faster
 */
void doIncreaseHeight() {
    if (sBodyHeightAngle > LIFT_MIN_ANGLE) {
        sBodyHeightAngle -= 2;
        if (!sJustExecutingCommand) {
            setLiftServosToBodyHeight();
        }
    }
}

void doDecreaseHeight() {
    if (sBodyHeightAngle < LIFT_MAX_ANGLE) {
        sBodyHeightAngle += 2;
        if (!sJustExecutingCommand) {
            setLiftServosToBodyHeight();
        }
    }
}

/*
 * Special calibration command
 */

/*
 * Signals which leg is to be calibrated
 */
void signalLeg(uint8_t aPivotServoIndex) {
    sServoArray[aPivotServoIndex + LIFT_SERVO_OFFSET]->easeTo(LIFT_MAX_ANGLE, 60);
    sServoArray[aPivotServoIndex]->easeTo(90, 60);
    sServoArray[aPivotServoIndex + LIFT_SERVO_OFFSET]->easeTo(90, 60);
}

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
