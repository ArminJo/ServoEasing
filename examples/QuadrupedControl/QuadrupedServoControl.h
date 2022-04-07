/*
 * QuadrupedServoControl.h
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef _QUADRUPED_SERVO_CONTROL_H
#define _QUADRUPED_SERVO_CONTROL_H

#include "QuadrupedServoConfiguration.h"

// Must specify this before the include of "ServoEasing.hpp"
//#define USE_PCA9685_SERVO_EXPANDER // Activate this to enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB // Activate this to force additional using of regular servo library.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT // Activate this to disable all but LINEAR movement. Saves up to 1540 bytes FLASH.
#define DISABLE_COMPLEX_FUNCTIONS // Activate this to disable the SINE, CIRCULAR, BACK, ELASTIC and BOUNCE easings. Saves up to 1850 bytes FLASH.
#define MAX_EASING_SERVOS 8
//#define ENABLE_MICROS_AS_DEGREE_PARAMETER // Activate this to enable also microsecond values as (target angle) parameter. Requires additional 128 Bytes FLASH.
//#define DEBUG // Activate this to generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER // Activate this to generate the Arduino plotter output#include "ServoEasing.hpp"    // include ServoEasing library source code

#define SUPPRESS_HPP_WARNING
#include "ServoEasing.h"

// Define 8 servos in exact this order!
extern ServoEasing frontLeftPivotServo;     // 0 - Front Left Pivot Servo
extern ServoEasing frontLeftLiftServo;      // 1 - Front Left Lift Servo
extern ServoEasing backLeftPivotServo;      // 2 - Back Left Pivot Servo
extern ServoEasing backLeftLiftServo;       // 3 - Back Left Lift Servo
extern ServoEasing backRightPivotServo;     // 4 - Back Right Pivot Servo
extern ServoEasing backRightLiftServo;      // 5 - Back Right Lift Servo
extern ServoEasing frontRightPivotServo;    // 6 - Front Right Pivot Servo
extern ServoEasing frontRightLiftServo;     // 7 - Front Right Lift Servo

extern uint16_t sQuadrupedServoSpeed;       // In degree/second, default is 90
extern uint8_t sActualBodyHeightAngle;      // The actual angle of the servos
extern volatile uint8_t sRequestedBodyHeightAngle;   // From LIFT_MIN_ANGLE to LIFT_MAX_ANGLE, default is 60 !!! The bigger the angle, the lower the body !!!
extern uint8_t sBodyHeight;                 // normalized body height from 0 (low) to 255 (high)
extern uint8_t sBodyHeightPercent;          // normalized body height from 0% (low) to 100% (high)

extern int8_t sServoTrimAngles[];

void attachAllQuadrupedServos();
void initializeAllQuadrupedServos(uint_fast16_t aQuadrupedServoSpeed);
void setQuadrupedServoSpeed(uint_fast16_t aQuadrupedServoSpeed);
void printQuadrupedServoSpeed();

void resetServosTo90Degree();
void shutdownServos();
void centerServos();
void setEasingTypeToLinear();
void setEasingTypeForMoving();

/*
 * Move and wait functions
 */
void synchronizeMoveAllServosAndCheckInputAndWait();
void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, int aDegree);
void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, int aDegree, uint16_t aDegreesPerSecond);
void updateAndCheckInputAndWaitForAllServosToStop();

/*
 * Set servo positions
 */
void setLiftServosToBodyHeight();
void setAllServos(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot, int aFrontRightPivot, int aFrontLeftLift,
        int aBackLeftLift, int aBackRightLift, int aFrontRightLift);
void setPivotServos(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot, int aFrontRightPivot);
void setLiftServos(int aFrontLeftLift, int aBackLeftLift, int aBackRightLift, int aFrontRightLift);
void setLiftServos(int aBodyHeightAngle);
void setLiftServoHeight(ServoEasing & aLiftServo, uint8_t aHeightPercent);

void printBodyHeight();
uint8_t convertLegPercentHeightToAngle(uint8_t aLegHeightPercent);


/*
 * Main transformation functions
 */
void transformAndSetAllServos(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot, int aFrontRightPivot,
        int aFrontLeftLift, int aBackLeftLift, int aBackRightLift, int aFrontRightLift, uint8_t aDirection =
        MOVE_DIRECTION_FORWARD, bool doMirror = false, bool aDoMove = true);
void transformAndSetPivotServos(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot, int aFrontRightPivot,
        uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false, bool aDoMove = true);
uint8_t transformOneServoIndex(uint8_t aServoIndexToTransform, uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false);

/*
 * Servo trim handling
 */
void printAndSetTrimAngles();
void eepromReadAndSetServoTrim();
void eepromWriteServoTrim();

#endif // _QUADRUPED_SERVO_CONTROL_H
#pragma once
