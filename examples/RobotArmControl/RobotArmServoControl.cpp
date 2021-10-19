/*
 * RobotArmServoControl.cpp
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#include <Arduino.h>

// Must specify this before the include of "ServoEasing.hpp"
//#define USE_PCA9685_SERVO_EXPANDER // Activate this to enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB // Activate this to force additional using of regular servo library.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT // Activate this to disable all but LINEAR movement. Saves up to 1540 bytes FLASH.
#define DISABLE_COMPLEX_FUNCTIONS // Activate this to disable the SINE, CIRCULAR, BACK, ELASTIC and BOUNCE easings. Saves up to 1850 bytes FLASH.
//#define MAX_EASING_SERVOS 3
//#define ENABLE_MICROS_AS_DEGREE_PARAMETER // Activate this to enable also microsecond values as (target angle) parameter. Requires additional 128 Bytes FLASH.
//#define DEBUG // Activate this to generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER // Activate this to generate the Arduino plotter output.
#include <ServoEasing.hpp>    // include servo library source

#include "RobotArmServoControl.h"
#include "RobotArmKinematics.h"
#include "RobotArmControl.h"

//#define DEBUG

// Define the 4 servos in exact this order!
ServoEasing BasePivotServo;    // 0 - Front Left Pivot Servo
ServoEasing HorizontalServo;     // 1 - Front Left Lift Servo
ServoEasing LiftServo;     // 2 - Back Left Pivot Servo
ServoEasing ClawServo;      // 3 - Back Left Lift Servo

struct ArmPosition sStartPosition; // is loaded with end position at every goToPosition...()
struct ArmPosition sEndPosition;
struct ArmPosition sCurrentPosition; // Used for movement
struct ArmPosition sPositionDelta; // sEndPosition - sStartPosition - used for faster computing the movement

/*
 * Angles for the servos for IR control
 */
int sBodyPivotAngle = PIVOT_NEUTRAL_OFFSET_DEGREE;
int sHorizontalServoAngle = HORIZONTAL_NEUTRAL_OFFSET_DEGREE;
int sLiftServoAngle = LIFT_NEUTRAL_OFFSET_DEGREE;
int sClawServoAngle = CLAW_START_ANGLE;

/*
 * Servo movement
 */
uint8_t sEasingType = EASE_LINEAR;
float sLastPercentageOfCompletion;
uint16_t sServoSpeed = 60;      // in degree/second

// User functions for ServoEasing implementing Inverse kinematics movement
// TODO test carefully
float moveInverseKinematicForBase(float aPercentageOfCompletion);
float moveInverseKinematicForHorizontal(float aPercentageOfCompletion);
float moveInverseKinematicForLift(float aPercentageOfCompletion);

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

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
    HorizontalServo.attach(HORIZONTAL_SERVO_PIN, HORIZONTAL_NEUTRAL_OFFSET_DEGREE, HORIZONTAL_ZERO_DEGREE_VALUE_MICROS,
            HORIZONTAL_AT_180_DEGREE_VALUE_MICROS);
    HorizontalServo.registerUserEaseInFunction(&moveInverseKinematicForHorizontal);

    /*
     * Wait for horizontal servo and then move lift and claw servos
     */
    delay(200);
    LiftServo.attach(LIFT_SERVO_PIN, LIFT_NEUTRAL_OFFSET_DEGREE, LIFT_ZERO_DEGREE_VALUE_MICROS, LIFT_AT_180_DEGREE_VALUE_MICROS);
    ClawServo.attach(CLAW_SERVO_PIN, CLAW_START_ANGLE);

    /*
     * Register functions for smooth moving with inverse kinematics
     */
    LiftServo.registerUserEaseInFunction(&moveInverseKinematicForLift);

    Serial.println(
            F(
                    "Value for 0 degree=" STR(HORIZONTAL_ZERO_DEGREE_VALUE_MICROS) "us. Value for 180 degree=" STR(HORIZONTAL_AT_180_DEGREE_VALUE_MICROS) "us."));
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
        ServoEasing::ServoEasingArray[tServoIndex]->setEasingType(aEasingType);
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

/*
 *  aBasePivot,  aHorizontal,  aLift,  aClaw
 */
void setAllServosD(uint16_t aMillisForMove, uint8_t aNumberOfValues, ...) {
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
    synchronizeMoveAllServosDAndCheckInputAndWait(aMillisForMove);
}

void goToNeutral() {
    sEndPosition.LeftRightDegree = PIVOT_NEUTRAL_OFFSET_DEGREE;
    sEndPosition.BackFrontDegree = HORIZONTAL_NEUTRAL_OFFSET_DEGREE;
    sEndPosition.DownUpDegree = LIFT_NEUTRAL_OFFSET_DEGREE;
    unsolve(&sEndPosition);
    goToPositionRelative(0, 0, 0);
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

uint16_t getDurationMillisForMove() {
    uint16_t tMaxDeltaMillimeter = getMaxDeltaMillimeter();
    uint16_t tMillisForMove = ((tMaxDeltaMillimeter * 125) / sServoSpeed) * 8; // *125 *8 to avoid temporary overflow at 16 bit arithmetic.
#ifdef DEBUG
    Serial.print(F("Speed="));
    Serial.print(sServoSpeed);
    Serial.print(F(" max mm="));
    Serial.print(tMaxDeltaMillimeter);
    Serial.print(F(" millis="));
    Serial.println(tMillisForMove);
#endif
    return tMillisForMove;
}

void openClaw() {
    sClawServoAngle = CLAW_OPEN_ANGLE;
    ClawServo.easeTo(CLAW_OPEN_ANGLE);
}

void closeClaw() {
    sClawServoAngle = CLAW_CLOSE_ANGLE;
    ClawServo.easeTo(CLAW_CLOSE_ANGLE);
}

/*
 * Use inverse kinematics user easing function for movement from current position to the new position
 * If parameter is KEEP_POSITION, the position will not be changed
 */
bool goToPosition(int aLeftRightMilliMeter, int aBackFrontMilliMeter, int aDownUpMilliMeter) {
// First copy old end position to new start position
    sStartPosition = sEndPosition;
#ifdef DEBUG
    Serial.print("Start: ");
    printPosition(&sStartPosition);
#endif
    if (aLeftRightMilliMeter == KEEP_POSITION) {
        sPositionDelta.LeftRight = 0;
    } else {
        sEndPosition.LeftRight = aLeftRightMilliMeter;
        sPositionDelta.LeftRight = aLeftRightMilliMeter - sStartPosition.LeftRight;
    }

    if (aBackFrontMilliMeter == KEEP_POSITION) {
        sPositionDelta.BackFront = 0;
    } else {
        sEndPosition.BackFront = aBackFrontMilliMeter;
        sPositionDelta.BackFront = aBackFrontMilliMeter - sStartPosition.BackFront;
    }

    if (aDownUpMilliMeter == KEEP_POSITION) {
        sPositionDelta.DownUp = 0;
    } else {
        sEndPosition.DownUp = aDownUpMilliMeter;
        sPositionDelta.DownUp = aDownUpMilliMeter - sStartPosition.DownUp;
    }
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
    setAllServosD(getDurationMillisForMove(), 3, sEndPosition.LeftRightDegree, sEndPosition.BackFrontDegree,
            sEndPosition.DownUpDegree);
    return true;
}

bool goToPositionRelative(int aLeftRightDeltaMilliMeter, int aBackFrontDeltaMilliMeter, int aDownUpDeltaMilliMeter) {
// First copy old end position to new start position
    sStartPosition = sEndPosition;
#ifdef DEBUG
    Serial.print("Start: ");
    printPosition(&sStartPosition);
#endif

    sEndPosition.LeftRight = sStartPosition.LeftRight + aLeftRightDeltaMilliMeter;
    sPositionDelta.LeftRight = aLeftRightDeltaMilliMeter;

    sEndPosition.BackFront = sStartPosition.BackFront + aBackFrontDeltaMilliMeter;
    sPositionDelta.BackFront = aBackFrontDeltaMilliMeter;

    sEndPosition.DownUp = sStartPosition.DownUp + aDownUpDeltaMilliMeter;
    sPositionDelta.DownUp = aDownUpDeltaMilliMeter;

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
    setAllServosD(getDurationMillisForMove(), 3, sEndPosition.LeftRightDegree, sEndPosition.BackFrontDegree,
            sEndPosition.DownUpDegree);
    return true;
}

/*
 * Compute all 3 angles for a given point in movement by inverse kinematics
 */
void computeNewCurrentAngles(float aPercentageOfCompletion) {
    sCurrentPosition.LeftRight = sStartPosition.LeftRight + (sPositionDelta.LeftRight * aPercentageOfCompletion);
    sCurrentPosition.BackFront = sStartPosition.BackFront + (sPositionDelta.BackFront * aPercentageOfCompletion);
    sCurrentPosition.DownUp = sStartPosition.DownUp + (sPositionDelta.DownUp * aPercentageOfCompletion);
    if (!solve(&sCurrentPosition)) {
        Serial.print("Current: ");
        Serial.print(aPercentageOfCompletion);
        Serial.print(" ");
        printPositionCartesianWithLinefeed(&sCurrentPosition);
    }
#ifdef TRACE
    else {
        Serial.print("Current: ");
        Serial.print(aPercentageOfCompletion);
        Serial.print(" ");
        printPositionCartesianWithLinefeed(&sCurrentPosition);
    }
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
    ServoEasing::ServoEasingArray[aServoIndex]->startEaseTo(aDegree, aDegreesPerSecond, false);
    do {
        if (!delayAndCheckForRobotArm(REFRESH_INTERVAL_MILLIS)) { // 20 ms - REFRESH_INTERVAL is in Microseconds
            return;
        }
    } while (!ServoEasing::ServoEasingArray[aServoIndex]->update());
}

void updateAndCheckInputAndWaitForAllServosToStop() {
    do {
        if (!delayAndCheckForRobotArm(REFRESH_INTERVAL_MILLIS)) { // 20 ms - REFRESH_INTERVAL is in Microseconds
            return;
        }
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
