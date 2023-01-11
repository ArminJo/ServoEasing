/*
 * RobotArmKinematics.h
 *
 *  Copyright (C) 2022  Armin Joachimsmeyer
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

#ifndef ROBOT_ARM_KINEMATICS_H_
#define ROBOT_ARM_KINEMATICS_H_

#include <stdint.h>

/*
 * First part of the name describes negative values. LeftRight value negative means left side.
 */
struct ArmPosition {
    float LeftRight;
    float BackFront;
    float DownUp;
    int LeftRightDegree; // horizontal plane angle: forward = 0 left > 0 right < 0
    int BackFrontDegree; // 0 is vertical, front > 0, back < 0
    int DownUpDegree;    // 0 is horizontal, up > 0, down < 0
};

/*
 * Inverse kinematics: X,Y,Z -> servo angle
 */
void cartesianToPolar(float a, float b, float& r, float& theta);
bool getAngleOfTriangle(float opp, float adj1, float adj2, float& theta);
bool doInverseKinematics(struct ArmPosition * aPositionStruct);

/*
 * Forward kinematics: servo angle -> X,Y,Z
 */
void polarToCartesian(float r, float theta, float& a, float& b);
void doForwardKinematics(struct ArmPosition * aPositionStruct);

/*
 * Helper functions
 */
void printPosition(struct ArmPosition * aPositionStruct);
void printPositionCartesianWithLinefeed(struct ArmPosition * aPositionStruct);
void printPositionCartesian(struct ArmPosition * aPositionStruct);
void printPositionShort(struct ArmPosition * aPositionStruct);
void printPositionShortWithUnits(struct ArmPosition * aPositionStruct);

#endif // ROBOT_ARM_KINEMATICS_H_
