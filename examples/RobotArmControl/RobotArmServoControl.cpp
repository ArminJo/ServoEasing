/*
 * RobotArmServoControl.cpp
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#include <Arduino.h>

#include "RobotArmServoControl.h"
#include "RobotArmKinematics.h"

#include "IRCommandDispatcher.h" // for checkIRInput(); and RETURN_IF_STOP;

// Define the 4 servos in exact this order!
ServoEasing BasePivotServo;    // 0 - Front Left Pivot Servo
ServoEasing HorizontalServo;     // 1 - Front Left Lift Servo
ServoEasing LiftServo;     // 2 - Back Left Pivot Servo
ServoEasing ClawServo;      // 3 - Back Left Lift Servo

struct ArmPosition sStartPosition, sEndPosition, sCurrentPosition, sPositionDelta;

/*
 * Angles for the servos for IR control
 */
uint8_t sBodyPivotAngle = PIVOT_NEUTRAL_OFFSET_DEGREE;
uint8_t sHorizontalServoAngle = HORIZONTAL_NEUTRAL_OFFSET_DEGREE;
uint8_t sLiftServoAngle = LIFT_NEUTRAL_OFFSET_DEGREE;
uint8_t sClawServoAngle = CLAW_START_ANGLE;

/*
 * Servo movement
 */
uint8_t sEasingType = EASE_LINEAR;
float sLastPercentageOfCompletion;
uint16_t sServoSpeed = 60;      // in degree/second

float moveInverseKinematicForBase(float aPercentageOfCompletion);
float moveInverseKinematicForHorizontal(float aPercentageOfCompletion);
float moveInverseKinematicForLift(float aPercentageOfCompletion);

void setupRobotArmServos() {

    setSpeedForAllServos(sServoSpeed);
    /*
     * Attach servos to Arduino Pins and set to start position. Must be done with write().
     * Set pivot servo first
     */
    BasePivotServo.attach(PIVOT_SERVO_PIN, PIVOT_ZERO_DEGREE_VALUE_MICROS, PIVOT_AT_180_DEGREE_VALUE_MICROS);
    BasePivotServo.setTrim(PIVOT_OFFSET + PIVOT_TRIM); // =86
    BasePivotServo.write(PIVOT_NEUTRAL_OFFSET_DEGREE);
    BasePivotServo.registerUserEaseInFunction(&moveInverseKinematicForBase);

    /*
     * Wait for pivot servo and then move horizontal one
     */
    delay(200);
    HorizontalServo.attach(HORIZONTAL_SERVO_PIN, HORIZONTAL_ZERO_DEGREE_VALUE_MICROS, HORIZONTAL_AT_180_DEGREE_VALUE_MICROS);
    HorizontalServo.write(HORIZONTAL_NEUTRAL_OFFSET_DEGREE);
    HorizontalServo.registerUserEaseInFunction(&moveInverseKinematicForHorizontal);

    /*
     * Wait for horizontal servo and then move lift and claw servos
     */
    delay(200);
    LiftServo.attach(LIFT_SERVO_PIN, LIFT_ZERO_DEGREE_VALUE_MICROS, LIFT_AT_180_DEGREE_VALUE_MICROS);
    ClawServo.attach(CLAW_SERVO_PIN);
    LiftServo.write(LIFT_NEUTRAL_OFFSET_DEGREE);
    ClawServo.write(CLAW_START_ANGLE);

    /*
     * Register functions for smooth moving with inverse kinematics
     */
    LiftServo.registerUserEaseInFunction(&moveInverseKinematicForLift);

    Serial.print(F("Value for 0 degree="));
    Serial.print(HORIZONTAL_ZERO_DEGREE_VALUE_MICROS);
    Serial.print(F("us. Value for 180 degree="));
    Serial.print(HORIZONTAL_AT_180_DEGREE_VALUE_MICROS);
    Serial.println(F("us."));
}

void shutdownServos() {
    Serial.println(F("Shutdown servos"));
    setAllServos(4, 0, 0, 100, 50);
}

/*
 * Set easing type for all servos except claw
 */
void setEasingType(uint8_t aEasingType) {
    sEasingType = aEasingType;
    for (uint8_t tServoIndex = 0; tServoIndex < NUMBER_OF_SERVOS - 1; ++tServoIndex) {
        sServoArray[tServoIndex]->setEasingType(aEasingType);
    }
}

/*
 *  aBasePivot,  aHorizontal,  aLift,  aClaw
 */
void setAllServos(uint8_t aNumberOfValues, ...) {
#ifdef DEBUG
    printArrayPositions(&Serial);
#endif
    va_list aDegreeValues;
    va_start(aDegreeValues, aNumberOfValues);
    setDegreeForAllServos(aNumberOfValues, &aDegreeValues);
    va_end(aDegreeValues);
#ifdef DEBUG
    printArrayPositions(&Serial);
#endif
    synchronizeMoveAllServosAndCheckInputAndWait();
}

/*void drawNumber(float aXOrigin, float aYOrigin, int aNumber, float aScale) {

  switch (aNumber) {

    case 0:
      drawTo(bx + 12 * scale, by + 6 * scale);
      lift(0);
      bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
      lift(1);
      break;
    case 1:

      drawTo(bx + 3 * scale, by + 15 * scale);
      lift(0);
      drawTo(bx + 10 * scale, by + 20 * scale);
      drawTo(bx + 10 * scale, by + 0 * scale);
      lift(1);
      break;
    case 2:
      drawTo(bx + 2 * scale, by + 12 * scale);
      lift(0);
      bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
      drawTo(bx + 1 * scale, by + 0 * scale);
      drawTo(bx + 14 * scale, by + 0 * scale);
      lift(1);
      break;
    case 3:
      drawTo(bx + 2 * scale, by + 17 * scale);
      lift(0);
      bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
      bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
      lift(1);
      break;
    case 4:
      drawTo(bx + 10 * scale, by + 0 * scale);
      lift(0);
      drawTo(bx + 10 * scale, by + 20 * scale);
      drawTo(bx + 2 * scale, by + 6 * scale);
      drawTo(bx + 12 * scale, by + 6 * scale);
      lift(1);
      break;
    case 5:
      drawTo(bx + 2 * scale, by + 5 * scale);
      lift(0);
      bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
      drawTo(bx + 5 * scale, by + 20 * scale);
      drawTo(bx + 12 * scale, by + 20 * scale);
      lift(1);
      break;
    case 6:
      drawTo(bx + 2 * scale, by + 10 * scale);
      lift(0);
      bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
      drawTo(bx + 11 * scale, by + 20 * scale);
      lift(1);
      break;
    case 7:
      drawTo(bx + 2 * scale, by + 20 * scale);
      lift(0);
      drawTo(bx + 12 * scale, by + 20 * scale);
      drawTo(bx + 2 * scale, by + 0);
      lift(1);
      break;
    case 8:
      drawTo(bx + 5 * scale, by + 10 * scale);
      lift(0);
      bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
      bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
      lift(1);
      break;

    case 9:
      drawTo(bx + 9 * scale, by + 11 * scale);
      lift(0);
      bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
      drawTo(bx + 5 * scale, by + 0);
      lift(1);
      break;

    case 11: // Doppelpunkt
      drawTo(bx + 5 * scale, by + 15 * scale);
      lift(0);
      bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
      lift(1);
      drawTo(bx + 5 * scale, by + 5 * scale);
      lift(0);
      bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
      lift(1);
      break;

  }
}
*/
/*
 * Use inverse kinematics user easing function for movement from current position to the new position
 */
bool goToPosition(int aLeftRight, int aBackFront, int aDownUp) {
    sStartPosition = sEndPosition;
#ifdef DEBUG
    Serial.print("Start: ");
    printPosition(&sStartPosition);
#endif
    sEndPosition.LeftRight = aLeftRight;
    sPositionDelta.LeftRight = aLeftRight - sStartPosition.LeftRight;
    sEndPosition.BackFront = aBackFront;
    sPositionDelta.BackFront = aBackFront - sStartPosition.BackFront;
    sEndPosition.DownUp = aDownUp;
    sPositionDelta.DownUp = aDownUp - sStartPosition.DownUp;
    if (!solve(&sEndPosition)) {
        Serial.print("Position cannot be solved: ");
        printPosition(&sEndPosition);
        return false;
    }
#ifdef DEBUG
    Serial.print("End: ");
    printPosition(&sEndPosition);
#endif
    sLastPercentageOfCompletion = -2.0; // to force first computation
    setAllServos(3, sEndPosition.LeftRightDegree, sEndPosition.BackFrontDegree, sEndPosition.DownUpDegree);
    return true;
}

/*
 * Compute all 3 angles for a given point in movement by inverse kinematics
 */
void computeNewCurrentAngles(float aPercentageOfCompletion) {
    sCurrentPosition.LeftRight = sStartPosition.LeftRight + (sPositionDelta.LeftRight * aPercentageOfCompletion);
    sCurrentPosition.BackFront = sStartPosition.BackFront + (sPositionDelta.BackFront * aPercentageOfCompletion);
    sCurrentPosition.DownUp = sStartPosition.DownUp + (sPositionDelta.DownUp * aPercentageOfCompletion);
    solve(&sCurrentPosition);
#ifdef TRACE
    Serial.print("Current: ");
    Serial.print(aPercentageOfCompletion);
    Serial.print(" ");
    printPosition(&sCurrentPosition);
#endif
}

/*
 * Inverse kinematics callback functions for ServoEasing
 */
float moveInverseKinematicForBase(float aPercentageOfCompletion) {
    if (aPercentageOfCompletion - sLastPercentageOfCompletion > 0.005) {
#ifdef TRACE
        Serial.print("B ");
#endif
        /*
         * Use a global variable in order to do it only once for all 3 servos
         */
        sLastPercentageOfCompletion = aPercentageOfCompletion;
        computeNewCurrentAngles(aPercentageOfCompletion);
    }
    return sCurrentPosition.LeftRightDegree + EASE_FUNCTION_DEGREE_INDICATOR_OFFSET;
}

float moveInverseKinematicForHorizontal(float aPercentageOfCompletion) {
    if (aPercentageOfCompletion - sLastPercentageOfCompletion > 0.005) {
#ifdef TRACE
        Serial.print("H ");
#endif
        sLastPercentageOfCompletion = aPercentageOfCompletion;
        computeNewCurrentAngles(aPercentageOfCompletion);
    }
    return sCurrentPosition.BackFrontDegree + EASE_FUNCTION_DEGREE_INDICATOR_OFFSET;
}

float moveInverseKinematicForLift(float aPercentageOfCompletion) {
    if (aPercentageOfCompletion - sLastPercentageOfCompletion > 0.005) {
#ifdef TRACE
        Serial.print("V ");
#endif
        sLastPercentageOfCompletion = aPercentageOfCompletion;
        computeNewCurrentAngles(aPercentageOfCompletion);
    }
    return sCurrentPosition.DownUpDegree + EASE_FUNCTION_DEGREE_INDICATOR_OFFSET;
}

void testInverseKinematic() {
    // init start position for first move
    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER; // 148;
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER; // 80
    // go to start position
    goToPosition(0, 148, 80);
    // go forward, backward
    goToPosition(0, 168, 80);
    goToPosition(0, 128, 80);
    goToPosition(0, 148, 80);
    // go up, down
    goToPosition(0, 148, 100);
    goToPosition(0, 148, 60);
    goToPosition(0, 148, 80);
    // go right, left
    goToPosition(40, 148, 80);
    goToPosition(-40, 148, 80);
    goToPosition(0, 148, 80);
}

/*
 * Only test computation, do not move arm!
 */
void testInverseAndForwardKinematic() {
    // init neutral position for first test
    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER; // 148
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER; // 80
    solve(&sEndPosition);
    Serial.print(F("End1="));
    printPosition(&sEndPosition);
    unsolve(&sEndPosition);
    Serial.print(F("End2="));
    printPosition(&sEndPosition);
    Serial.println();

    /*
     * back and down
     */
    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER;
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER - 20;
    solve(&sEndPosition);
    Serial.print(F("End3="));
    printPosition(&sEndPosition);
    unsolve(&sEndPosition);
    Serial.print(F("End4="));
    printPosition(&sEndPosition);
    Serial.println();

    /*
     * Initial position but right
     */
    sEndPosition.LeftRight = 80;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER;
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER;
    solve(&sEndPosition);
    Serial.print(F("End5="));
    printPosition(&sEndPosition);
    unsolve(&sEndPosition);
    Serial.print(F("End6="));
    printPosition(&sEndPosition);
    Serial.println();

    /*
     * initial position but down
     */
    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER;
    sEndPosition.DownUp = -40;
    solve(&sEndPosition);
    Serial.print(F("End7="));
    printPosition(&sEndPosition);
    unsolve(&sEndPosition);
    Serial.print(F("End8="));
    printPosition(&sEndPosition);
    Serial.println();

    /*
     * initial position but left
     */
    sEndPosition.LeftRight = -40;
    sEndPosition.BackFront = LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER;
    sEndPosition.DownUp = HORIZONTAL_ARM_LENGTH_MILLIMETER;
    solve(&sEndPosition);
    Serial.print(F("End9="));
    printPosition(&sEndPosition);
    unsolve(&sEndPosition);
    Serial.print(F("End10="));
    printPosition(&sEndPosition);
    Serial.println();
}

void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, int aDegree) {
    moveOneServoAndCheckInputAndWait(aServoIndex, aDegree, sServoSpeed);
}

void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, int aDegree, uint16_t aDegreesPerSecond) {
    sServoArray[aServoIndex]->startEaseTo(aDegree, aDegreesPerSecond, false);
    do {
        checkIRInput();
        RETURN_IF_STOP;
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!sServoArray[aServoIndex]->update());
}

void updateAndCheckInputAndWaitForAllServosToStop() {
    do {
        checkIRInput();
        RETURN_IF_STOP;
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!updateAllServos());
}

void synchronizeMoveAllServosAndCheckInputAndWait() {
    setEaseToForAllServos();
    synchronizeAllServosAndStartInterrupt(false);
    updateAndCheckInputAndWaitForAllServosToStop();
}
