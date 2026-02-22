/*
 * QuadrupedServoControl.hpp
 *
 * Contains all the servo related functions and data.
 *
 * Program for controlling a mePed Robot V2 with 8 servos using an IR Remote at pin A0
 * Supported IR remote are KEYES (the original mePed remote) and WM10
 * Select the one you have in QuadrupedConfiguration.h
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

#ifndef _QUADRUPED_SERVO_CONTROL_HPP
#define _QUADRUPED_SERVO_CONTROL_HPP

#include <Arduino.h>

#include "QuadrupedServoControl.h"  // Contains macros to configure the ServoEasing library
#include "ServoEasing.hpp"          // include source

// This block must be located after the includes of other *.hpp files
//#define LOCAL_INFO  // This enables info output only for this file
#include "LocalDebugLevelStart.h"

// Indexes of Servos: 0 = Front Left Pivot Servo, 1 = Front Left Lift Servo, 2 = Back Left Pivot, 3 = Back Left Lift, 4 + 5 = Back Right, 6 + 7 = Front Right
ServoEasing frontLeftPivotServo;    // 0 - Front Left Pivot Servo
ServoEasing frontLeftLiftServo;     // 1 - Front Left Lift Servo
ServoEasing backLeftPivotServo;     // 2 - Back Left Pivot Servo
ServoEasing backLeftLiftServo;      // 3 - Back Left Lift Servo
ServoEasing backRightPivotServo;    // 4 - Back Right Pivot Servo
ServoEasing backRightLiftServo;     // 5 - Back Right Lift Servo
ServoEasing frontRightPivotServo;   // 6 - Front Right Pivot Servo
ServoEasing frontRightLiftServo;    // 7 - Front Right Lift Servo

uint16_t sQuadrupedServoSpeed;      // in degree/second
volatile uint8_t sRequestedBodyHeightAngle = LIFT_LOWEST_ANGLE + 20; // From LIFT_LOWEST_ANGLE to LIFT_HIGHEST_ANGLE !!! The bigger the angle, the lower the body !!!
uint8_t sBodyHeight;            // normalized body height from 0 (low) to 255 (high), used by e.g. NeoPixel display
uint8_t sBodyHeightPercent;     // normalized body height from 0% (low) to 100% (high), only used for printing

#if !E2END
# warning "Device does not have EEPROM available. No support for persistent storage of correction trim angels."
#else
// Arrays of trim angles stored in EEPROM
EEMEM int8_t sServoTrimAnglesEEPROM[NUMBER_OF_LEG_SERVOS]; // The one which resides in EEPROM and IR read out at startup - filled by eepromWriteServoTrim
#endif
int8_t sServoTrimAngles[NUMBER_OF_LEG_SERVOS]; // RAM copy for easy setting trim angles by remote, filled by eepromReadServoTrim

void attachAllQuadrupedServos() {
    // Attach servos to Arduino Pins in exact this order!
    // do not use initial position here, since using resetServosTo90Degree() is smaller
    frontLeftPivotServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN);
    frontLeftLiftServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 1);
    backLeftPivotServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 2);
    backLeftLiftServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 3);
    backRightPivotServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 4);
    backRightLiftServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 5);
    frontRightPivotServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 6);
    frontRightLiftServo.attach(FRONT_LEFT_PIVOT_SERVO_PIN + 7);

    // Invert direction for back left and front right lift servos. This is, because I mounted the right and left lift servos symmetrical.
    backLeftLiftServo.setReverseOperation(true);
    frontRightLiftServo.setReverseOperation(true);

#if defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
    USServo.attach(PIN_US_SERVO);
#endif
}

/*
 * @param aServoSpeed in degree per second
 */
void initializeAllQuadrupedServos(uint_fast16_t aQuadrupedServoSpeed) {
    attachAllQuadrupedServos();
    setQuadrupedServoSpeed(aQuadrupedServoSpeed);

#if E2END
    //Read and apply trim values
    eepromReadAndSetServoTrim();
#endif

    // Reset all servo to initial position of 90 degree
    resetServosTo90Degree();
    setLiftServosToBodyHeight();

#if defined(LOCAL_INFO)
    printBodyHeight();
#else
    setBodyHeight();
#endif
}

void shutdownServos() {
    INFO_PRINTLN(F("Shutdown servos"));
    auto tOriginalRequestedBodyHeightAngle = sRequestedBodyHeightAngle;
    sRequestedBodyHeightAngle = LIFT_HIGHEST_ANGLE;
    centerServos();
    sRequestedBodyHeightAngle = tOriginalRequestedBodyHeightAngle;
}

void centerServos() {
    uint8_t tRequestedBodyHeightAngle = sRequestedBodyHeightAngle; // sRequestedBodyHeightAngle is volatile
#if defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
    ServoEasing::ServoEasingNextPositionArray[INDEX_OF_US_DISTANCE_SERVO] = 90;
#endif
    moveAllServosToNextPositionsAndCheckInputAndWait(90, 90, 90, 90, tRequestedBodyHeightAngle, tRequestedBodyHeightAngle,
            tRequestedBodyHeightAngle, tRequestedBodyHeightAngle);
}

void setQuadrupedServoSpeed(uint_fast16_t aQuadrupedServoSpeed) {
    sQuadrupedServoSpeed = aQuadrupedServoSpeed;
    setSpeedForAllServos(sQuadrupedServoSpeed);
    printQuadrupedServoSpeed();
}

void printQuadrupedServoSpeed() {
    INFO_PRINT(F(" Speed="));
    INFO_PRINTLN(sQuadrupedServoSpeed);
}

void printAndSetTrimAngles() {
    for (uint_fast8_t i = 0; i < NUMBER_OF_LEG_SERVOS; ++i) {
        INFO_PRINT(F("ServoTrimAngle["));
        INFO_PRINT(i);
        INFO_PRINT(F("]="));
        INFO_PRINTLN(sServoTrimAngles[i]);
        ServoEasing::ServoEasingArray[i]->setTrim(sServoTrimAngles[i], true);
    }
}

void resetServosTo90Degree() {
    for (uint_fast8_t i = 0; i < NUMBER_OF_SERVOS; ++i) {
        ServoEasing::ServoEasingArray[i]->write(90);
    }
}

#if E2END
/*
 * Copy calibration array from EEPROM to RAM and set uninitialized values to 0
 */
void eepromReadAndSetServoTrim() {
    INFO_PRINTLN(F("eepromReadAndSetServoTrim()"));
    eeprom_read_block((void*) &sServoTrimAngles, &sServoTrimAnglesEEPROM, NUMBER_OF_LEG_SERVOS);
    printAndSetTrimAngles();
}

void eepromWriteServoTrim() {
    eeprom_write_block((void*) &sServoTrimAngles, &sServoTrimAnglesEEPROM, NUMBER_OF_LEG_SERVOS);
    printAndSetTrimAngles();
}
#endif

void setEasingTypeToLinear() {
    for (uint_fast8_t tServoIndex = 0; tServoIndex < NUMBER_OF_LEG_SERVOS; ++tServoIndex) {
        ServoEasing::ServoEasingArray[tServoIndex]->setEasingType(EASE_LINEAR);
    }
}

void setEasingTypeForMoving() {
    for (int tServoIndex = 0; tServoIndex < NUMBER_OF_LEG_SERVOS; ++tServoIndex) {
        ServoEasing::ServoEasingArray[tServoIndex]->setEasingType(EASE_LINEAR); // Pivot servos
        tServoIndex++;
        ServoEasing::ServoEasingArray[tServoIndex]->setEasingType(EASE_QUADRATIC_BOUNCING); // Up / down leg servos
    }
}

void setPivotServosSpeed(uint_fast16_t aDegreesPerSecond) {
    for (int tServoIndex = 0; tServoIndex < NUMBER_OF_LEG_SERVOS; tServoIndex += 2) {
        ServoEasing::ServoEasingArray[tServoIndex]->setSpeed(aDegreesPerSecond); // Pivot servos
    }
}

/**
 * Computes the mask to get the index of the mirrored Front <-> Back or Left <-> Right servo for the second part of creep
 * Works with the following index definitions:
 * 0 = Front Left Pivot Servo, 1 = Front Left Lift Servo, 2 = Back Left Pivot, 3 = Back Left Lift, 4 + 5 = Back Right, 6 + 7 = Front Right
 * @param aDirection
 * If moving direction is forward or backward, then we must swap the real Left and Right servos during the second half of the creep to get a complete creep.
 * If moving direction is left or right, then we must swap the real Front and Back servos instead,
 * because they now act as the virtual Left and Right servos. it is because the mirror has turned by 90 or -90 degree :-).
 * @return The XOR mask to get the mirrored index of a servo
 */
uint8_t getMirrorXorMask(uint8_t aDirection) {
// XOR the servo index with this value to get the mirrored index
    if (aDirection & MOVE_DIRECTION_SIDE_MASK) {
        // Direction MOVE_DIRECTION_LEFT or MOVE_DIRECTION_RIGHT here
        return 0x2; // Front <-> Back
    } else {
        // Direction MOVE_DIRECTION_FORWARD or MOVE_DIRECTION_BACKWARD here
        return 0x6; // Left <-> Right
    }
}

/**
 * Transformation routine for following index definitions:
 * 0 = Front Left Pivot Servo, 1 = Front Left Lift Servo, 2 = Back Left Pivot, 3 = Back Left Lift, 4 + 5 = Back Right, 6 + 7 = Front Right
 * Transform index of servo by direction and mirroring
 * @param doMirror swaps Front <-> Back or Left <-> Right for second part of creep.
 */
uint8_t transformOneServoIndex(uint8_t aServoIndexToTransform, uint8_t aDirection, bool doMirror) {
    if (doMirror) {
        // XOR the index with this value in order to get the mirrored index Front <-> Back or Left <-> Right
        return ((aServoIndexToTransform + (aDirection * SERVOS_PER_LEG)) % NUMBER_OF_LEG_SERVOS) ^ getMirrorXorMask(aDirection);
    }
    // E.g. for MOVE_DIRECTION_LEFT add 2 to the index
    return ((aServoIndexToTransform + (aDirection * SERVOS_PER_LEG)) % NUMBER_OF_LEG_SERVOS);
}
/**
 * Main transformation routine for creep and trot movements.
 * Works with the following index definitions:
 * 0 = Front Left Pivot Servo, 1 = Front Left Lift Servo, 2 = Back Left Pivot, 3 = Back Left Lift, 4 + 5 = Back Right, 6 + 7 = Front Right
 *
 * Input are servo positions for the forward move
 * Output are the servo positions in array ServoEasingNextPositionArray for the transformed move direction
 * Direction forward changes nothing.
 * Direction backward swaps forward and backward servos / increases index by NUMBER_OF_LEGS/2
 * Direction left increases index by 1 and right by 3.
 * Mirroring swaps left and right (XOR with 0x06) and invert all angles.
 *
 * @param aFrontLeftPivot, aBackLeftPivot etc. degree values for corresponding servo
 * @param aDirection MOVE_DIRECTION_FORWARD (0, no change), MOVE_DIRECTION_LEFT etc.
 * @param doMirror swaps Front <-> Back or Left <-> Right for second part of creep.
 * @param aDoMove start blocking move after transformation
 */
void transformAndSetAllServos(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot, int aFrontRightPivot,
        int aFrontLeftLift, int aBackLeftLift, int aBackRightLift, int aFrontRightLift, uint8_t aDirection, bool doMirror,
        bool aDoMove) {
    uint8_t tIndexToAdd = aDirection * SERVOS_PER_LEG; // +2 servos for each next direction
    uint8_t tXorToGetMirroredIndex = 0x0;
// Invert angles for pivot servos
    if (doMirror) {
// XOR the index with this value to get the mirrored index
        tXorToGetMirroredIndex = getMirrorXorMask(aDirection);
    }
    bool doInvert = doMirror; // better readability

    uint8_t tEffectivePivotServoIndex;
    tEffectivePivotServoIndex = ((FRONT_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_LEG_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFrontLeftPivot = 180 - aFrontLeftPivot;
    }
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex] = aFrontLeftPivot;
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aFrontLeftLift;

    tEffectivePivotServoIndex = ((BACK_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_LEG_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBackLeftPivot = 180 - aBackLeftPivot;
    }
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex] = aBackLeftPivot;
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aBackLeftLift;

    tEffectivePivotServoIndex = ((BACK_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_LEG_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBackRightPivot = 180 - aBackRightPivot;
    }
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex] = aBackRightPivot;
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aBackRightLift;

    tEffectivePivotServoIndex = ((FRONT_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_LEG_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFrontRightPivot = 180 - aFrontRightPivot;
    }
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex] = aFrontRightPivot;
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aFrontRightLift;

    if (aDoMove) {
        /*
         * Apply all movements now
         */
        synchronizeMoveAllServosAndCheckInputAndWait();
    }
}

/*
 * Like transformAndSetAllServos() but only for pivot servos
 */
void transformAndSetPivotServos(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot, int aFrontRightPivot,
        uint8_t aDirection, bool doMirror, bool aDoMove) {
    uint8_t tIndexToAdd = aDirection * SERVOS_PER_LEG;
    uint8_t tXorToGetMirroredIndex = 0x0;
// Invert angles for pivot servos
    bool doInvert = false;
    if (doMirror) {
// XOR the index with this value to get the mirrored index
        tXorToGetMirroredIndex = getMirrorXorMask(aDirection);
        doInvert = true;
    }

    uint8_t tEffectivePivotServoIndex;
    tEffectivePivotServoIndex = ((FRONT_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_LEG_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFrontLeftPivot = 180 - aFrontLeftPivot;
    }
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex] = aFrontLeftPivot;

    tEffectivePivotServoIndex = ((BACK_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_LEG_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBackLeftPivot = 180 - aBackLeftPivot;
    }
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex] = aBackLeftPivot;

    tEffectivePivotServoIndex = ((BACK_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_LEG_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBackRightPivot = 180 - aBackRightPivot;
    }
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex] = aBackRightPivot;

    tEffectivePivotServoIndex = ((FRONT_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_LEG_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFrontRightPivot = 180 - aFrontRightPivot;
    }
    ServoEasing::ServoEasingNextPositionArray[tEffectivePivotServoIndex] = aFrontRightPivot;

    if (aDoMove) {
        /*
         * Apply all movements now
         */
        synchronizeMoveAllServosAndCheckInputAndWait();
    }
}

void testTransform() {
// left legs are close together, right legs are in straight right direction
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_FORWARD, false, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_FORWARD, true, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_BACKWARD, false, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_BACKWARD, true, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_LEFT, false, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_LEFT, true, false);
    printArrayPositions(&Serial);
}

void movePivotServosToNextPositionsAndCheckInputAndWait(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot,
        int aFrontRightPivot) {
    ServoEasing::ServoEasingNextPositionArray[FRONT_LEFT_PIVOT] = aFrontLeftPivot;
    ServoEasing::ServoEasingNextPositionArray[BACK_LEFT_PIVOT] = aBackLeftPivot;
    ServoEasing::ServoEasingNextPositionArray[BACK_RIGHT_PIVOT] = aBackRightPivot;
    ServoEasing::ServoEasingNextPositionArray[FRONT_RIGHT_PIVOT] = aFrontRightPivot;
    synchronizeMoveAllServosAndCheckInputAndWait();
}

/*
 * Accepts height from 0 to 100
 */
void setLiftServoHeight(ServoEasing &aLiftServo, uint8_t aHeightPercent) {
    if (aHeightPercent > 100) {
        aHeightPercent = 100;
    }
    int tDegreeForLiftServo = map(aHeightPercent, 0, 100, LIFT_HIGHEST_ANGLE, LIFT_LOWEST_ANGLE);
    aLiftServo.easeTo(tDegreeForLiftServo);
}

/*
 * Set all servos to the same angle
 */
void moveLiftServosToNextPositionsAndCheckInputAndWait(int aBodyHeightAngle) {
    ServoEasing::ServoEasingNextPositionArray[FRONT_LEFT_LIFT] = aBodyHeightAngle;
    ServoEasing::ServoEasingNextPositionArray[BACK_LEFT_LIFT] = aBodyHeightAngle;
    ServoEasing::ServoEasingNextPositionArray[BACK_RIGHT_LIFT] = aBodyHeightAngle;
    ServoEasing::ServoEasingNextPositionArray[FRONT_RIGHT_LIFT] = aBodyHeightAngle;
    synchronizeMoveAllServosAndCheckInputAndWait();
}

void moveLiftServosToNextPositionsAndCheckInputAndWait(int aFrontLeftLift, int aBackLeftLift, int aBackRightLift,
        int aFrontRightLift) {
    ServoEasing::ServoEasingNextPositionArray[FRONT_LEFT_LIFT] = aFrontLeftLift;
    ServoEasing::ServoEasingNextPositionArray[BACK_LEFT_LIFT] = aBackLeftLift;
    ServoEasing::ServoEasingNextPositionArray[BACK_RIGHT_LIFT] = aBackRightLift;
    ServoEasing::ServoEasingNextPositionArray[FRONT_RIGHT_LIFT] = aFrontRightLift;
    synchronizeMoveAllServosAndCheckInputAndWait();
}

/*
 * Used after change of sRequestedBodyHeightAngle
 */
void setLiftServosToBodyHeight() {
    // Set values direct, since we expect only a change of 2 degree
    // We write value to every second servo from the array :-)
    for (uint_fast8_t tServoIndex = LIFT_SERVO_OFFSET; tServoIndex < NUMBER_OF_LEG_SERVOS; tServoIndex += SERVOS_PER_LEG) {
        ServoEasing::ServoEasingArray[tServoIndex]->write(sRequestedBodyHeightAngle);
    }
}

void setBodyHeight(){
    sBodyHeight = map(sRequestedBodyHeightAngle, LIFT_HIGHEST_ANGLE, LIFT_LOWEST_ANGLE, 0, 255);
    sBodyHeightPercent = map(sRequestedBodyHeightAngle, LIFT_HIGHEST_ANGLE, LIFT_LOWEST_ANGLE, 0, 100);
}

void printBodyHeight() {
    setBodyHeight();
    INFO_PRINT(F("BodyHeight="));
    INFO_PRINT(sBodyHeight);
    INFO_PRINT(F(" -> "));
    INFO_PRINT(sBodyHeightPercent);
    INFO_PRINTLN('%');
}

/*
 * Attention!!! Leg height is inverse to body height!
 */
uint8_t convertLegPercentHeightToAngle(uint8_t aLegHeightPercent) {
    return map(aLegHeightPercent, 0, 100, LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE);
}

void moveAllServosToNextPositionsAndCheckInputAndWait(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot,
        int aFrontRightPivot, int aFrontLeftLift, int aBackLeftLift, int aBackRightLift, int aFrontRightLift) {
    ServoEasing::ServoEasingNextPositionArray[FRONT_LEFT_PIVOT] = aFrontLeftPivot;
    ServoEasing::ServoEasingNextPositionArray[BACK_LEFT_PIVOT] = aBackLeftPivot;
    ServoEasing::ServoEasingNextPositionArray[BACK_RIGHT_PIVOT] = aBackRightPivot;
    ServoEasing::ServoEasingNextPositionArray[FRONT_RIGHT_PIVOT] = aFrontRightPivot;

    ServoEasing::ServoEasingNextPositionArray[FRONT_LEFT_LIFT] = aFrontLeftLift;
    ServoEasing::ServoEasingNextPositionArray[BACK_LEFT_LIFT] = aBackLeftLift;
    ServoEasing::ServoEasingNextPositionArray[BACK_RIGHT_LIFT] = aBackRightLift;
    ServoEasing::ServoEasingNextPositionArray[FRONT_RIGHT_LIFT] = aFrontRightLift;
    synchronizeMoveAllServosAndCheckInputAndWait();
}

void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, int aDegree) {
    moveOneServoAndCheckInputAndWait(aServoIndex, aDegree, sQuadrupedServoSpeed);
}

void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, int aDegree, uint16_t aDegreesPerSecond) {
    ServoEasing::ServoEasingArray[aServoIndex]->startEaseTo(aDegree, aDegreesPerSecond, false);
    do {
#if defined(_QUADRUPED_HELPER_HPP)
        if (delayAndCheckByApplication(SERVO_REFRESH_INTERVAL_MILLIS - 1)) { // 19 ms - REFRESH_INTERVAL is in Microseconds
            return;
        }
#else
        delay(SERVO_REFRESH_INTERVAL_MILLIS - 1);
#endif
    } while (!ServoEasing::ServoEasingArray[aServoIndex]->update());
}

void updateAndCheckInputAndWaitForAllServosToStop() {
    do {
#if defined(_QUADRUPED_HELPER_HPP)
        if (delayAndCheckByApplication(SERVO_REFRESH_INTERVAL_MILLIS - 1)) { // 19 ms - REFRESH_INTERVAL is in Microseconds
            return;
        }
#else
        delay(SERVO_REFRESH_INTERVAL_MILLIS - 1);
#endif
    } while (!updateAllServos() || sCurrentlyRunningAction == ACTION_TYPE_PAUSE); // sCurrentlyRunningAction = ACTION_TYPE_PAUSE -> supports pause / resume
}

void synchronizeMoveAllServosAndCheckInputAndWait() {
    setEaseToForAllServos();
    synchronizeAllServosAndStartInterrupt(false); // do not start interrupt
    updateAndCheckInputAndWaitForAllServosToStop();
}

#include "LocalDebugLevelEnd.h"

#endif // _QUADRUPED_SERVO_CONTROL_HPP
