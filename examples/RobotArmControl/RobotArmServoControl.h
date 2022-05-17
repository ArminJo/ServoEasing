/*
 * RobotArmServoControl.h
 *
 *  Created on: 21.05.2019
 *      Author: Armin
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
