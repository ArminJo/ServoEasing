/*
 * QuadrupedServoControl.h
 *
 *  This file is part of QuadrupedControl https://github.com/ArminJo/QuadrupedControl.
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  Copyright (C) 2019-2026  Armin Joachimsmeyer
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

#ifndef _QUADRUPED_SERVO_CONTROL_H
#define _QUADRUPED_SERVO_CONTROL_H

// Define 8 servos in exact this order!
extern ServoEasing frontLeftPivotServo;     // 0 - Front Left Pivot Servo
extern ServoEasing frontLeftLiftServo;      // 1 - Front Left Lift Servo
extern ServoEasing backLeftPivotServo;      // 2 - Back Left Pivot Servo
extern ServoEasing backLeftLiftServo;       // 3 - Back Left Lift Servo
extern ServoEasing backRightPivotServo;     // 4 - Back Right Pivot Servo
extern ServoEasing backRightLiftServo;      // 5 - Back Right Lift Servo
extern ServoEasing frontRightPivotServo;    // 6 - Front Right Pivot Servo
extern ServoEasing frontRightLiftServo;     // 7 - Front Right Lift Servo
#if defined(QUADRUPED_HAS_US_DISTANCE_SENSOR_SERVO)
extern ServoEasing USServo;
#endif

extern uint16_t sQuadrupedServoSpeed;       // In degree/second, default is 90
extern volatile uint8_t sRequestedBodyHeightAngle; // From LIFT_MIN_ANGLE to LIFT_MAX_ANGLE, default is 60 !!! The bigger the angle, the lower the body !!!
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
void setPivotServosSpeed(uint_fast16_t aDegreesPerSecond);

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
void moveAllServosToNextPositionsAndCheckInputAndWait(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot,
        int aFrontRightPivot, int aFrontLeftLift, int aBackLeftLift, int aBackRightLift, int aFrontRightLift);
void movePivotServosToNextPositionsAndCheckInputAndWait(int aFrontLeftPivot, int aBackLeftPivot, int aBackRightPivot,
        int aFrontRightPivot);
void moveLiftServosToNextPositionsAndCheckInputAndWait(int aFrontLeftLift, int aBackLeftLift, int aBackRightLift,
        int aFrontRightLift);
void moveLiftServosToNextPositionsAndCheckInputAndWait(int aBodyHeightAngle);
void setLiftServoHeight(ServoEasing &aLiftServo, uint8_t aHeightPercent);

void setBodyHeight();
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

#if E2END
/*
 * Servo trim handling
 */
void printAndSetTrimAngles();
void eepromReadAndSetServoTrim();
void eepromWriteServoTrim();
#endif

#endif // _QUADRUPED_SERVO_CONTROL_H
