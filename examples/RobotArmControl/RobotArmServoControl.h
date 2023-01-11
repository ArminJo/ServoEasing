/*
 * RobotArmServoControl.h
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef _ROBOT_ARM_SERVO_CONTROL_H
#define _ROBOT_ARM_SERVO_CONTROL_H

#include "RobotArmKinematics.h" // for ArmPosition

#define SUPPRESS_HPP_WARNING
#include <ServoEasing.h>    // include servo library

extern ServoEasing BasePivotServo;
extern ServoEasing HorizontalServo;
extern ServoEasing LiftServo;
extern ServoEasing ClawServo;

extern struct ArmPosition sStartPosition;
extern struct ArmPosition sEndPosition;
extern struct ArmPosition sCurrentPosition;
extern struct ArmPosition sPositionDelta;

extern int sBodyPivotAngle;
extern int sHorizontalServoAngle;
extern int sLiftServoAngle;
extern int sClawServoAngle;

extern uint8_t sEasingType;
extern float sLastPercentageOfCompletion;
extern uint16_t sRobotArmServoSpeed;      // in degree/second

void setupRobotArmServos();
void goToFolded();
void goToCenter();

void setEasingType(uint8_t aEasingType);

void setAllServos(uint8_t aNumberOfValues, ...);

/*
 * Inverse kinematic
 */
#define KEEP_POSITION (-1000) // Can be used as parameter for goToPosition()
void openClaw();
void closeClaw();
bool goToPosition(int aLeftRightMilliMeter, int aBackFrontMilliMeter, int aDownUpMilliMeter);
bool goToPositionRelative(int aLeftRightDeltaMilliMeter, int aBackFrontDeltaMilliMeter, int aDownUpDeltaMilliMeter);

uint16_t getMaxDeltaMillimeter();

void computeNewCurrentAngles(float aPercentageOfCompletion);
void doTestMove();
void testInverseAndForwardKinematic();

/*
 * Move and wait functions
 */
void synchronizeMoveAllServosAndCheckInputAndWait();
void synchronizeMoveAllServosDAndCheckInputAndWait(uint16_t aMillisForMove);
void moveOneServoAndCheckInputAndWait(uint8_t aServoIndex, float aDegree, uint16_t aDegreesPerSecond);
void updateAndCheckInputAndWaitForAllServosToStop();

#endif // _ROBOT_ARM_SERVO_CONTROL_H
