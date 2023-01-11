/*
 * RobotArmKinematics.cpp
 *
 * Contains the kinematics functions for the meArm robot arm
 * See also: https://www.instructables.com/id/4-DOF-Mechanical-Arm-Robot-Controlled-by-Arduino
 *
 * Servo trims are chosen, so that 0°, 0°, 0° (left/right, back/front, up/down) results in the neutral position of the robot arm.
 * Neutral means: pivot direction forward, and both arms rectangular up and forward
 * Neutral position: X=0
 *                   Y=(LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER) - Here claw length is from wrist to hand PLUS base center to shoulder
 *                   Z=HORIZONTAL_ARM_LENGTH_MILLIMETER
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
 */

#include <Arduino.h>     // for PI etc.
#include "RobotArmKinematics.h"

#if __has_include("RobotArmServoConfiguration.h")
#include "RobotArmServoConfiguration.h"
#else
#define HORIZONTAL_ARM_LENGTH_MILLIMETER    80
#define LIFT_ARM_LENGTH_MILLIMETER          80
#define CLAW_LENGTH_MILLIMETER              68 // Length from wrist to hand PLUS base center to shoulder
#endif

//#define LOCAL_TRACE

/*
 * Inverse kinematics: X,Y,Z -> servo angle
 */

/**
 * Get angle from a triangle using the cosine rule
 * All sides are given
 */
bool getAngleOfTriangle(float aOppositeSide, float aAdjacentSide1, float aAdjacentSide2, float &aComputedAngle) {
    /*
     * Cosine rule:
     * C*C = A*A + B*B - 2*A*B*cos(angle_AB)
     * C is opposite side
     * A, B are adjacent sides
     */
    float tDenominator = 2 * aAdjacentSide1 * aAdjacentSide2; // (2*A*B)

    if (tDenominator == 0) { // @suppress("Direct float comparison")
        return false;
    }
    float tCosinusAB = (aAdjacentSide1 * aAdjacentSide1 + aAdjacentSide2 * aAdjacentSide2 - aOppositeSide * aOppositeSide)
            / tDenominator;

    if (tCosinusAB > 1 || tCosinusAB < -1) {
        return false;
    }
    aComputedAngle = acos(tCosinusAB);

    return true;
}

/*
 * Convert Cartesian to polar coordinates
 * returns 0 to PI (0 to 180 degree)
 * returns 0 to -PI (o to -180 degree) if aYValue < 0
 */
void cartesianToPolar(float aXValue, float aYValue, float &aRadius, float &aAngleRadiant) {
    // Determine magnitude of Cartesian coordinates
    aRadius = sqrt(aXValue * aXValue + aYValue * aYValue);

    // Don't try to calculate zero-magnitude vectors' angles
    if (aRadius == 0) { // @suppress("Direct float comparison")
        aAngleRadiant = 0;
        return;
    }

    float tNormalizedXValue = aXValue / aRadius;

    // Calculate angle in 0..PI
    // use cos since for negative y values it needs only inverting the sign of radiant
    aAngleRadiant = acos(tNormalizedXValue);

    // Convert to full range
    if (aYValue < 0) {
        aAngleRadiant *= -1;
    }

#if defined(LOCAL_TRACE)
    Serial.print(F("XValue="));
    Serial.print(aXValue);
    Serial.print(F(" YValue="));
    Serial.print(aYValue);
    Serial.print(F(" Radius="));
    Serial.print(aRadius);
    Serial.print(F(" AngleDegree="));
    Serial.println(aAngleRadiant * RAD_TO_DEG);
#endif
}

/**
 * Inverse kinematics: X,Y,Z -> servo angle
 * Reads the Cartesian positions from aPositionStruct and fills the angles of aPositionStruct with the computed value
 * @param aPositionStruct structure holding input position and output angles
 * returns true if solving was successful, false if solving is not possible
 */
bool doInverseKinematics(struct ArmPosition *aPositionStruct) {
    bool tReturnValue = true;
    // Get horizontal degree for servo (0-180 degree) and get radius for next computations
    float tRadiusHorizontalMillimeter, tHorizontalAngleRadiant;
    /*
     * First doInverseKinematics the horizontal (X/Y) plane. Get LeftRightDegree and radius from X and Y.
     */
    cartesianToPolar(aPositionStruct->LeftRight, aPositionStruct->BackFront, tRadiusHorizontalMillimeter, tHorizontalAngleRadiant);
    aPositionStruct->LeftRightDegree = (tHorizontalAngleRadiant * RAD_TO_DEG) - 90; // tLeftRightDegree: -90 to 90 degree, -90 is right, 0 degree is neutral, 90 is left

    /*
     * Now we have the base rotation for horizontal (X/Y) plane.
     * Because we cannot move claw behind the base axis, we must check if the radius in horizontal plane is smaller than the virtual claw length!
     */
    if (tRadiusHorizontalMillimeter < CLAW_LENGTH_MILLIMETER) {
        Serial.print(F("For "));
        printPositionCartesian(aPositionStruct);
        Serial.print(F(" horizontal radius "));
        Serial.print(tRadiusHorizontalMillimeter);
        Serial.println(F("mm is < claw length. -> Take claw length now."));
        tRadiusHorizontalMillimeter = 0; // fallback
        tReturnValue = false;
    } else {
        tRadiusHorizontalMillimeter -= CLAW_LENGTH_MILLIMETER;
    }

    /*
     * Now doInverseKinematics the vertical plane
     * Get vertical angle and radius/distance to claw - angle is negative for claw below horizontal zero plane
     */
    float tVerticalAngleToClawRadiant, tRadiusVerticalMillimeter;
    cartesianToPolar(tRadiusHorizontalMillimeter, aPositionStruct->DownUp, tRadiusVerticalMillimeter, tVerticalAngleToClawRadiant);
#if defined(LOCAL_TRACE)
    Serial.print(F("RadiusHorizontal="));
    Serial.print(tRadiusHorizontalMillimeter);
    Serial.print(F(" VerticalAngleToClawDegee="));
    Serial.print(tVerticalAngleToClawRadiant * RAD_TO_DEG);
    Serial.print(F(" RadiusVertical="));
    Serial.print(tRadiusVerticalMillimeter);
#endif

    // Solve arm inner angles
    float tAngleClawHorizontalRadiant;      // angle between vertical angle to claw and horizontal arm
    if (!getAngleOfTriangle(LIFT_ARM_LENGTH_MILLIMETER, HORIZONTAL_ARM_LENGTH_MILLIMETER, tRadiusVerticalMillimeter,
            tAngleClawHorizontalRadiant)) {
        Serial.print(F("Cannot solve angle between claw base and horizontal arm : "));
        printPositionCartesianWithLinefeed(aPositionStruct);
        return false;
    }
#if defined(LOCAL_TRACE)
    Serial.print(F(" AngleClawHorizontalDegree="));
    Serial.print(tAngleClawHorizontalRadiant * RAD_TO_DEG);
#endif

    float tAngleHorizontalLiftRadiant;  // angle between horizontal and lift arm
    if (!getAngleOfTriangle(tRadiusVerticalMillimeter, HORIZONTAL_ARM_LENGTH_MILLIMETER, LIFT_ARM_LENGTH_MILLIMETER,
            tAngleHorizontalLiftRadiant)) {
        Serial.print(F("Cannot solve angle between horizontal and lift arm: "));
        printPositionCartesianWithLinefeed(aPositionStruct);
        return false;
    }

    /*
     * Vertical plane schematic of inverse kinematic values
     *            LIFT_ARM
     * __________/______________________________
     * | <-AngleHorizontalLift                 /
     * |                                     /
     * |<-HORIZONTAL_ARM                   /
     *  |                                /
     *  |                              /
     *  |                            /
     *   |                         /
     *   |                       /
     *   |   RadiusVertical->  /
     *    |                  /
     *    |                /
     *    |              /
     *     |           /
     *     |         /
     *     | AngleClawHorizontal
     *      | |  /
     *      |  /
     *      |/ <-VerticalAngleToClaw
     * _______________________________
     */
    aPositionStruct->BackFrontDegree = (HALF_PI - (tVerticalAngleToClawRadiant + tAngleClawHorizontalRadiant)) * RAD_TO_DEG;
    // Now BackFrontDegree == 0 is vertical, front > 0, back < 0
#if defined(LOCAL_TRACE)
    Serial.print(F(" BackFrontDegree="));
    Serial.print(aPositionStruct->BackFrontDegree);
#endif

    aPositionStruct->DownUpDegree = (tVerticalAngleToClawRadiant + tAngleClawHorizontalRadiant + tAngleHorizontalLiftRadiant - PI)
            * RAD_TO_DEG;
    // Now DownUpDegree == 0 is horizontal, up > 0, down < 0
#if defined(LOCAL_TRACE)
    Serial.print(F(" DownUpDegree="));
    Serial.println(aPositionStruct->DownUpDegree);
#endif
    return tReturnValue;
}

/*
 * Forward kinematics: servo angle -> X,Y,Z
 */
void polarToCartesian(float aRadius, float aAngleRadiant, float &aXValue, float &aYValue) {
    aXValue = aRadius * cos(aAngleRadiant);
    aYValue = aRadius * sin(aAngleRadiant);
}

/**
 * Fill X,Y,Z in aPositionStruct according to angles of aPositionStruct.
 * always solvable :-)
 */
void doForwardKinematics(struct ArmPosition *aPositionStruct) {
    float tInputAngle;
    float tHeightOfHorizontalArm, tHorizontalArmHorizontalShift, tLiftHorizontal, tLiftHeight, tInputAngleRad;

    // input angle: horizontal = 0 vertical = 90
    tInputAngle = aPositionStruct->BackFrontDegree;
    // here we have:  0 is vertical, front > 0, back < 0
    tInputAngle *= -1;
    // here we have:  0 is vertical, front < 0, back > 0
    tInputAngleRad = (tInputAngle + 90) * DEG_TO_RAD;

    polarToCartesian(HORIZONTAL_ARM_LENGTH_MILLIMETER, tInputAngleRad, tHorizontalArmHorizontalShift, tHeightOfHorizontalArm);

    // input angle: horizontal = 0, up > 0, down < 0
    tInputAngle = aPositionStruct->DownUpDegree;
    tInputAngleRad = tInputAngle * DEG_TO_RAD;
    polarToCartesian(LIFT_ARM_LENGTH_MILLIMETER, tInputAngleRad, tLiftHorizontal, tLiftHeight);
    aPositionStruct->DownUp = tHeightOfHorizontalArm + tLiftHeight;

    // Add horizontal length
    float tHorizontalArmAndClawShift = tHorizontalArmHorizontalShift + tLiftHorizontal + CLAW_LENGTH_MILLIMETER;

    // input is horizontal plane angle: forward = 0 right > 0 left < 0
    tInputAngle = aPositionStruct->LeftRightDegree;
    // here we have horizontal plane angle: forward = 0 left > 0 right < 0
    tInputAngle *= -1;
    tInputAngleRad = tInputAngle * DEG_TO_RAD;
    polarToCartesian(tHorizontalArmAndClawShift, tInputAngleRad, aPositionStruct->BackFront, aPositionStruct->LeftRight);
}

/*
 * No trailing linefeed!
 */
void printPositionCartesian(struct ArmPosition *aPositionStruct) {
    Serial.print(F("X="));
    Serial.print(aPositionStruct->LeftRight);
    Serial.print(F("mm, Y="));
    Serial.print(aPositionStruct->BackFront);
    Serial.print(F("mm, Z="));
    Serial.print(aPositionStruct->DownUp);
    Serial.print(F("mm"));
}

void printPositionCartesianWithLinefeed(struct ArmPosition *aPositionStruct) {
    printPositionCartesian(aPositionStruct);
    Serial.println();
}

void printPosition(struct ArmPosition *aPositionStruct) {
    Serial.print(F("LeftRight="));
    Serial.print(aPositionStruct->LeftRight);
    Serial.print("|");
    Serial.print(aPositionStruct->LeftRightDegree);
    Serial.print(F(" BackFront="));
    Serial.print(aPositionStruct->BackFront);
    Serial.print("|");
    Serial.print(aPositionStruct->BackFrontDegree);
    Serial.print(F(" DownUp="));
    Serial.print(aPositionStruct->DownUp);
    Serial.print("|");
    Serial.println(aPositionStruct->DownUpDegree);
}

void printPositionShort(struct ArmPosition *aPositionStruct) {
    Serial.print(aPositionStruct->LeftRight);
    Serial.print(" ");
    Serial.print(aPositionStruct->BackFront);
    Serial.print(" ");
    Serial.print(aPositionStruct->DownUp);
    Serial.print(F(" <-> "));

    Serial.print(aPositionStruct->LeftRightDegree);
    Serial.print(" ");
    Serial.print(aPositionStruct->BackFrontDegree);
    Serial.print(" ");
    Serial.println(aPositionStruct->DownUpDegree);
}

void printPositionShortWithUnits(struct ArmPosition *aPositionStruct) {
    printPositionCartesian(aPositionStruct);
    Serial.print(F(" <-> "));
    Serial.print(aPositionStruct->LeftRightDegree);
    Serial.print(F(", "));
    Serial.print(aPositionStruct->BackFrontDegree);
    Serial.print(F(", "));
    Serial.print(aPositionStruct->DownUpDegree);
    Serial.println(F(" degree"));
}

#if defined(LOCAL_TRACE)
#undef LOCAL_TRACE
#endif
