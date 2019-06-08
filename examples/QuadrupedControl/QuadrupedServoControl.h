/*
 * QuadrupedServoControl.h
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef QUADRUPEDSERVOCONTROL_H_
#define QUADRUPEDSERVOCONTROL_H_

#include "QuadrupedServoConfiguration.h"
#include "ServoEasing.h"    // include servo library

// Define 8 servos in exact this order!
extern ServoEasing frontLeftPivotServo;    // 0 - Front Left Pivot Servo
extern ServoEasing frontLeftLiftServo;     // 1 - Front Left Lift Servo
extern ServoEasing backLeftPivotServo;     // 2 - Back Left Pivot Servo
extern ServoEasing backLeftLiftServo;      // 3 - Back Left Lift Servo
extern ServoEasing backRightPivotServo;    // 4 - Back Right Pivot Servo
extern ServoEasing backRightLiftServo;     // 5 - Back Right Lift Servo
extern ServoEasing frontRightPivotServo;   // 6 - Front Right Pivot Servo
extern ServoEasing frontRightLiftServo;    // 7 - Front Right Lift Servo

extern uint16_t sServoSpeed;      //  = 90 in degree/second
extern uint8_t sBodyHeightAngle;  //  = 60 From LIFT_MIN_ANGLE to LIFT_MAX_ANGLE !!! The bigger the angle, the lower the body !!!

extern int8_t sServoTrimAngles[];

void setupQuadrupedServos();
void printSpeed();

void resetServosTo90Degree();
void shutdownServos() ;
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
void setAllServos(int aFLP, int aBLP, int aBRP, int aFRP, int aFLL, int aBLL, int aBRL, int aFRL);
void setPivotServos(int aFLP, int aBLP, int aBRP, int aFRP);
void setLiftServos(int aFLL, int aBLL, int aBRL, int aFRL);
void setLiftServos(int aBodyHeightAngle);

/*
 * Main transformation functions
 */
void transformAndSetAllServos(int aFLP, int aBLP, int aBRP, int aFRP, int aFLL, int aBLL, int aBRL,
        int aFRL, uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false, bool aDoMove = true);
void transformAndSetPivotServos(int aFLP, int aBLP, int aBRP, int aFRP, uint8_t aDirection = MOVE_DIRECTION_FORWARD,
        bool doMirror = false, bool aDoMove = true);
uint8_t transformOneServoIndex(uint8_t aServoIndexToTransform, uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false);

/*
 * Servo trim handling
 */
void printTrimAngles();
void eepromReadAndSetServoTrim();
void eepromWriteServoTrim();

#endif /* QUADRUPEDSERVOCONTROL_H_ */
