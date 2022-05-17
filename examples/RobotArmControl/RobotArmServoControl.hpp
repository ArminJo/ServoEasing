/*
 * RobotArmServoControl.hpp
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef _ROBOT_ARM_SERVO_CONTROL_HPP
#define _ROBOT_ARM_SERVO_CONTROL_HPP

#include <Arduino.h>

#include "RobotArmServoConfiguration.h" // Contains macros to configure the ServoEasing library
#include <ServoEasing.hpp>              // include servo library source

#include "RobotArmServoControl.h"

#if defined(TRACE)
#define LOCAL_TRACE
#else
//#define LOCAL_TRACE // Enable TRACE output only for this file
#endif

// Define the 4 servos in exact this order!
ServoEasing BasePivotServo;
ServoEasing HorizontalServo;
ServoEasing LiftServo;
ServoEasing ClawServo;

struct ArmPosition sStartPosition;      // is loaded with end position at every goToPosition...()
struct ArmPosition sEndPosition;        // Target position
struct ArmPosition sCurrentPosition;    // Used by our user easing function
struct ArmPosition sPositionDelta;      // sEndPosition - sStartPosition - used for faster computing in easing function
/*
 * Angles for the servos for IR control
 */
int sBodyPivotAngle = 0;
int sHorizontalServoAngle = 0;
int sLiftServoAngle = 0;
int sClawServoAngle = 0;

/*
 * Servo movement
 */
uint8_t sEasingType = EASE_LINEAR;
float sLastPercentageOfCompletion;
uint16_t sRobotArmServoSpeed = 40; // in degree/second or millimeter/second for inverse kinematic

// User functions for ServoEasing implementing inverse kinematics movement
float moveInverseKinematic(float aPercentageOfCompletion, void *aUserDataPointer);

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

void setupRobotArmServos() {

    /*
     * Attach servos to Arduino Pins and set to start position. Must be done with write().
     * Set pivot servo first
     */
    // Operate from 90° (left) to -90° (right), this is an initial trim
    BasePivotServo.attach(PIVOT_SERVO_PIN, 0, PIVOT_MICROS_AT_MINUS_70_DEGREE, PIVOT_MICROS_AT_PLUS_70_DEGREE, -70, 70);
    BasePivotServo.write(0);
    // Register function for smooth moving with inverse kinematics
    BasePivotServo.registerUserEaseInFunction(&moveInverseKinematic, &sCurrentPosition.LeftRightDegree);
    Serial.print(F("Microseconds for pivot 0 degree="));
    Serial.println((PIVOT_MICROS_AT_PLUS_70_DEGREE + PIVOT_MICROS_AT_MINUS_70_DEGREE) / 2);

    /*
     * Wait for pivot servo and then move horizontal one
     */
    delay(200);
    // Using attachWithTrim() with an explicit trim value is easier to understand here
    HorizontalServo.attach(HORIZONTAL_SERVO_PIN, 0, HORIZONTAL_MICROS_AT_0_NEUTRAL, HORIZONTAL_MICROS_AT_90_DEGREE, 0, 90);
    HorizontalServo.registerUserEaseInFunction(&moveInverseKinematic, &sCurrentPosition.BackFrontDegree);

    /*
     * Wait for horizontal servo and then move lift and claw servos
     */
    delay(200);
    // Using attachWithTrim() with an explicit trim value is easier to understand here
    LiftServo.attach(LIFT_SERVO_PIN, 0, LIFT_MICROS_AT_MINUS_90_DEGREE, LIFT_MICROS_AT_0_NEUTRAL, -90, 0);
    LiftServo.registerUserEaseInFunction(&moveInverseKinematic, &sCurrentPosition.DownUpDegree);

    /*
     * Operate claw from 0° (close) to 90° (open), this is an initial trim
     */
    ClawServo.attach(CLAW_SERVO_PIN, 0, CLAW_MICROS_AT_180_DEGREE, CLAW_MICROS_AT_CLOSE, 180, 0);

    setSpeedForAllServos(sRobotArmServoSpeed); // must be after attach

//    Serial.println(
//            F(
//                    "Value for 0 degree=" STR(HORIZONTAL_ZERO_DEGREE_VALUE_MICROS) "us. Value for 180 degree=" STR(HORIZONTAL_AT_180_DEGREE_VALUE_MICROS) "us."));
}

/*
 *  aBasePivot,  aHorizontal,  aLift,  aClaw
 */
void setAllServos(uint8_t aNumberOfValues, ...) {
#if defined(DEBUG)
    printArrayPositions(&Serial);
#endif
    va_list aDegreeValues;
    va_start(aDegreeValues, aNumberOfValues);
    setDegreeForAllServos(aNumberOfValues, &aDegreeValues);
    va_end(aDegreeValues);
#if defined(DEBUG)
    printArrayPositions(&Serial);
#endif
    synchronizeMoveAllServosAndCheckInputAndWait();
}

/*
 *  aBasePivot,  aHorizontal,  aLift,  aClaw
 */
void setAllServosDAndWait(uint16_t aMillisForMove, uint8_t aNumberOfValues, ...) {
#if defined(DEBUG)
    printArrayPositions(&Serial);
#endif
    va_list aDegreeValues;
    va_start(aDegreeValues, aNumberOfValues);
    setDegreeForAllServos(aNumberOfValues, &aDegreeValues);
    va_end(aDegreeValues);
#if defined(DEBUG)
    printArrayPositions(&Serial);
#endif
    synchronizeMoveAllServosDAndCheckInputAndWait(aMillisForMove);
}

/*
 * Goto folded position
 */
void goToFolded() {
    Serial.println(F("Shutdown servos"));
    setAllServos(4, 0, HORIZONTAL_MINIMUM_DEGREE, 0, 0);
}

void goToCenter() {
    setAllServos(4, 0, 0, 0, 0);
}

uint16_t getMaxDeltaMillimeter() {
    float tMaxDelta = 0;
    if (abs(sPositionDelta.LeftRight) > tMaxDelta) {
        tMaxDelta = abs(sPositionDelta.LeftRight);
    }
    if (abs(sPositionDelta.BackFront) > tMaxDelta) {
        tMaxDelta = abs(sPositionDelta.BackFront);
    }
    if (abs(sPositionDelta.DownUp) > tMaxDelta) {
        tMaxDelta = abs(sPositionDelta.DownUp);
    }
    return tMaxDelta;
}

/*
 * To keep it simple, just get max distance of all 3 dimensions, not the geometric distance
 * Interpret sRobotArmServoSpeed as millimeter/second
 */
uint16_t getDurationMillisForMove() {
    uint16_t tMaxDeltaMillimeter = getMaxDeltaMillimeter(); // tMaxDeltaMillimeter can be greater than 256
    uint16_t tMillisForMove = ((tMaxDeltaMillimeter * 125) / sRobotArmServoSpeed) * 8; // *125 *8 to avoid temporary overflow at 16 bit arithmetic.
    if (sDebugOutputIsEnabled) {
        Serial.print(F("Speed="));
        Serial.print(sRobotArmServoSpeed);
        Serial.print(F(" max mm="));
        Serial.print(tMaxDeltaMillimeter);
        Serial.print(F(" millis="));
        Serial.println(tMillisForMove);
    }
    return tMillisForMove;
}

void openClaw() {
    sClawServoAngle = 90;
    ClawServo.easeTo(90);
}

void closeClaw() {
    sClawServoAngle = 0;
    ClawServo.easeTo(0);
}

/*
 * Use inverse kinematics user easing function for movement from current position to the new position
 * If parameter is KEEP_POSITION, the position will not be changed
 */
bool goToPosition(int aLeftRightMillimeter, int aBackFrontMillimeter, int aDownUpMillimeter) {
// First copy old end position to new start position
    sStartPosition = sEndPosition;
    if (sDebugOutputIsEnabled) {
        Serial.print("Start: ");
        printPosition(&sStartPosition);
    }
    /*
     * Fill EndPosition and PositionDelta
     */
    if (aLeftRightMillimeter == KEEP_POSITION) {
        sPositionDelta.LeftRight = 0;
    } else {
        sEndPosition.LeftRight = aLeftRightMillimeter;
        sPositionDelta.LeftRight = aLeftRightMillimeter - sStartPosition.LeftRight;
    }

    if (aBackFrontMillimeter == KEEP_POSITION) {
        sPositionDelta.BackFront = 0;
    } else {
        sEndPosition.BackFront = aBackFrontMillimeter;
        sPositionDelta.BackFront = aBackFrontMillimeter - sStartPosition.BackFront;
    }

    if (aDownUpMillimeter == KEEP_POSITION) {
        sPositionDelta.DownUp = 0;
    } else {
        sEndPosition.DownUp = aDownUpMillimeter;
        sPositionDelta.DownUp = aDownUpMillimeter - sStartPosition.DownUp;
    }

    /*
     * Do inverse kinematics (get servo angles from position) for new EndPosition
     */
    if (!doInverseKinematics(&sEndPosition)) {
        Serial.print("This end position cannot be solved: ");
        printPosition(&sEndPosition);
        return false;
    }
    if (sDebugOutputIsEnabled) {
        Serial.print("End: ");
        printPosition(&sEndPosition);
    }
    sLastPercentageOfCompletion = -2.0; // To force call to computeNewCurrentAngles(aPercentageOfCompletion) in easing function
    setAllServosDAndWait(getDurationMillisForMove(), 3, sEndPosition.LeftRightDegree, sEndPosition.BackFrontDegree,
            sEndPosition.DownUpDegree);
    return true;
}

bool goToPositionRelative(int aLeftRightDeltaMilliMeter, int aBackFrontDeltaMilliMeter, int aDownUpDeltaMilliMeter) {
    return goToPosition(sEndPosition.LeftRight + aLeftRightDeltaMilliMeter, sEndPosition.BackFront + aBackFrontDeltaMilliMeter,
            sEndPosition.DownUp + aDownUpDeltaMilliMeter);
}

/*
 * Compute all 3 angles for a given point in movement by inverse kinematics
 */
void computeNewCurrentAngles(float aPercentageOfCompletion) {
    sCurrentPosition.LeftRight = sStartPosition.LeftRight + (sPositionDelta.LeftRight * aPercentageOfCompletion);
    sCurrentPosition.BackFront = sStartPosition.BackFront + (sPositionDelta.BackFront * aPercentageOfCompletion);
    sCurrentPosition.DownUp = sStartPosition.DownUp + (sPositionDelta.DownUp * aPercentageOfCompletion);
    doInverseKinematics(&sCurrentPosition);
#if defined(LOCAL_TRACE)
    Serial.print("Current: ");
    Serial.print(aPercentageOfCompletion);
    Serial.print(" ");
    printPositionShort(&sCurrentPosition);
#endif
}

/*
 * Inverse kinematics callback function for ServoEasing
 */
float moveInverseKinematic(float aPercentageOfCompletion, void *aUserDataPointer) {
    if (aPercentageOfCompletion - sLastPercentageOfCompletion > 0.005) {
        // Use a global variable, to do computing only once for all 3 servos
        sLastPercentageOfCompletion = aPercentageOfCompletion;
        computeNewCurrentAngles(aPercentageOfCompletion);
    }
    return *((int*) aUserDataPointer) + EASE_FUNCTION_DEGREE_INDICATOR_OFFSET;
}

/*
 * Only test computation and print, do not move arm!
 */
void testInverseAndForwardKinematic() {
    /*
     * Neutral
     */
    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER; // 148
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER; // 80
    doInverseKinematics(&sEndPosition);
    Serial.println();
    Serial.print(F("Neutral IK="));
    printPosition(&sEndPosition);
    doForwardKinematics(&sEndPosition);
    Serial.print(F("Neutral FK="));
    printPosition(&sEndPosition);
    Serial.println();

    /*
     * up and down
     */
    Serial.println(F("up, down"));
    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = (LIFT_ARM_LENGTH_MILLIMETER / 2) + CLAW_LENGTH_MILLIMETER;
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER + 40; // up
    doInverseKinematics(&sEndPosition);
    Serial.print(F("Z+40 IK="));
    printPosition(&sEndPosition);
    doForwardKinematics(&sEndPosition);
    Serial.print(F("Z+40 FK="));
    printPosition(&sEndPosition);

    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = (LIFT_ARM_LENGTH_MILLIMETER / 2) + CLAW_LENGTH_MILLIMETER;
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER - 40; // down
    doInverseKinematics(&sEndPosition);
    Serial.print(F("Z-40 IK="));
    printPosition(&sEndPosition);
    doForwardKinematics(&sEndPosition);
    Serial.print(F("Z-40 FK="));
    printPosition(&sEndPosition);
    Serial.println();

    /*
     * Forward and back
     */
    Serial.println(F("forward, backward"));
    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER + 40; // front
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER;
    doInverseKinematics(&sEndPosition);
    Serial.print(F("Y+40 IK="));
    printPosition(&sEndPosition);
    doForwardKinematics(&sEndPosition);
    Serial.print(F("Y+40 FK="));
    printPosition(&sEndPosition);

    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER - 40; // back
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER;
    doInverseKinematics(&sEndPosition);
    Serial.print(F("Y-40 IK="));
    printPosition(&sEndPosition);
    doForwardKinematics(&sEndPosition);
    Serial.print(F("Y-40 FK="));
    printPosition(&sEndPosition);
    Serial.println();

    /*
     * Right and left
     */
    Serial.println(F("right, left"));
    sEndPosition.LeftRight = 100;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER;
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER;
    doInverseKinematics(&sEndPosition);
    Serial.print(F("X+100 IK="));
    printPosition(&sEndPosition);
    doForwardKinematics(&sEndPosition);
    Serial.print(F("X+100 FK="));
    printPosition(&sEndPosition);

    sEndPosition.LeftRight = -100;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER;
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER;
    doInverseKinematics(&sEndPosition);
    Serial.print(F("X-100 IK="));
    printPosition(&sEndPosition);
    doForwardKinematics(&sEndPosition);
    Serial.print(F("X-100 FK="));
    printPosition(&sEndPosition);
    Serial.println();

    /*
     * Minimum height, claw touches ground
     */
    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER;
    sEndPosition.DownUp = MINIMUM_HEIGHT_MILLIMETER;
    doInverseKinematics(&sEndPosition);
    Serial.print(F("MINIMUM_HEIGHT IK="));
    printPosition(&sEndPosition);
    doForwardKinematics(&sEndPosition);
    Serial.print(F("MINIMUM_HEIGHT FK="));
    printPosition(&sEndPosition);
    Serial.println();

}

void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, float aDegree, uint16_t aDegreesPerSecond) {
    ServoEasing::ServoEasingArray[aServoIndex]->startEaseTo(aDegree, aDegreesPerSecond, false);
    do {
#if defined(ROBOT_ARM_HAS_IR_CONTROL)
        DELAY_AND_RETURN_IF_STOP(REFRESH_INTERVAL_MILLIS); // 20 ms - REFRESH_INTERVAL is in Microseconds
#endif
    } while (!ServoEasing::ServoEasingArray[aServoIndex]->update());
}

void updateAndCheckInputAndWaitForAllServosToStop() {
    do {
#if defined(ROBOT_ARM_HAS_IR_CONTROL)
        DELAY_AND_RETURN_IF_STOP(REFRESH_INTERVAL_MILLIS); // 20 ms - REFRESH_INTERVAL is in Microseconds
#endif
    } while (!updateAllServos());
}

void synchronizeMoveAllServosDAndCheckInputAndWait(uint16_t aMillisForMove) {
    setEaseToDForAllServos(aMillisForMove);
    synchronizeAllServosAndStartInterrupt(false);
    updateAndCheckInputAndWaitForAllServosToStop();
}

void synchronizeMoveAllServosAndCheckInputAndWait() {
    setEaseToForAllServos();
    synchronizeAllServosAndStartInterrupt(false);
    updateAndCheckInputAndWaitForAllServosToStop();
}
#if defined(LOCAL_TRACE)
#undef LOCAL_TRACE
#endif
#endif // _ROBOT_ARM_SERVO_CONTROL_HPP
