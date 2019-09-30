/*
 * RobotArmServoControl.h
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef ROBOTARMSERVOCONTROL_H_
#define ROBOTARMSERVOCONTROL_H_

#include "RobotArmServoConfiguration.h"
#include "RobotArmKinematics.h"

#include <ServoEasing.h>    // include servo library

extern ServoEasing BasePivotServo;    // 0 - Front Left Pivot Servo
extern ServoEasing HorizontalServo;     // 1 - Front Left Lift Servo
extern ServoEasing LiftServo;     // 2 - Back Left Pivot Servo
extern ServoEasing ClawServo;      // 3 - Back Left Lift Servo

extern struct ArmPosition sStartPosition, sEndPosition, sCurrentPosition, sPositionDelta;

extern int sBodyPivotAngle;
extern int sHorizontalServoAngle;
extern int sLiftServoAngle;
extern int sClawServoAngle;

extern uint8_t sEasingType;
extern float sLastPercentageOfCompletion;
extern uint16_t sServoSpeed;      // in degree/second

void setupRobotArmServos();
void shutdownServos();

void setEasingType(uint8_t aEasingType);

void setAllServos(uint8_t aNumberOfValues, ...);

/*
 * Inverse kinematic
 */
#define KEEP_POSITION (-1000) // Can be used as parameter for goToPosition()
void goToNeutral();
void openClaw();
void closeClaw();
bool goToPosition(int aLeftRightMilliMeter, int aBackFrontMilliMeter, int aDownUpMilliMeter);
bool goToPositionRelative(int aLeftRightDeltaMilliMeter, int aBackFrontDeltaMilliMeter, int aDownUpDeltaMilliMeter);

uint16_t getMaxDeltaMillimeter();

void computeNewCurrentAngles(float aPercentageOfCompletion);
void testInverseKinematic();
void testInverseAndForwardKinematic();

/*
 * Move and wait functions
 */
void synchronizeMoveAllServosAndCheckInputAndWait();
void synchronizeMoveAllServosDAndCheckInputAndWait(uint16_t aMillisForMove);
void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, int aDegree);
void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, int aDegree, uint16_t aDegreesPerSecond);
void updateAndCheckInputAndWaitForAllServosToStop();

#endif /* ROBOTARMSERVOCONTROL_H_ */

#pragma once
