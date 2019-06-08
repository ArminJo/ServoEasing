/*
 * RobotArmKinematics.cpp
 *
 * Contains the kinematics functions for the robot arm
 * See also: https://www.instructables.com/id/4-DOF-Mechanical-Arm-Robot-Controlled-by-Arduino
 *
 *  Copyright (C) 2019  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include "RobotArmKinematics.h"

#include "RobotArmServoConfiguration.h"

#include <math.h>
#include <Arduino.h>     // for PI etc.
#include "ServoEasing.h" // for clipDegreeSpecial()

//#define TRACE

/*
 * Inverse kinematics: X,Y,Z -> servo angle
 */

// Get angle from a triangle using cosine rule
bool cosangle(float opp, float adj1, float adj2, float& theta) {
    // Cosine rule:
    // C^2 = A^2 + B^2 - 2*A*B*cos(angle_AB)
    // cos(angle_AB) = (A^2 + B^2 - C^2)/(2*A*B)
    // C is opposite
    // A, B are adjacent
    float den = 2 * adj1 * adj2;

    if (den == 0)
        return false;
    float c = (adj1 * adj1 + adj2 * adj2 - opp * opp) / den;

    if (c > 1 || c < -1)
        return false;

    theta = acos(c);

    return true;
}

/*
 * Get polar coords from cartesian ones
 * returns 0 to PI (0 to 180 degree)
 * returns 0 to -PI (o to -180 degree) if aYValue < 0
 */
void cart2polar(float aXValue, float aYValue, float& aRadius, float& aAngleRadiant) {
#ifdef TRACE
    Serial.print("XValue=");
    Serial.print(aXValue);
    Serial.print(" YValue=");
    Serial.print(aYValue);
#endif

    // Determine magnitude of cartesian coords
    aRadius = sqrt(aXValue * aXValue + aYValue * aYValue);

    // Don't try to calculate zero-magnitude vectors' angles
    if (aRadius == 0)
        return;

    float tNormalizedXValue = aXValue / aRadius;

    // Calculate angle in 0..PI
    // use cos since for negative y values it needs only inverting the sign of radiant
    aAngleRadiant = acos(tNormalizedXValue);

    // Convert to full range
    if (aYValue < 0) {
        aAngleRadiant *= -1;
    }
#ifdef TRACE
    Serial.print(" AngleRadiant=");
    Serial.println(aAngleRadiant);
#endif
}

/*
 * returns false if solving is not possible
 */
bool solve(struct ArmPosition * aPositionStruct) {
    // Get horizontal degree for servo (0-180 degree) and get radius for next computations
    float tRadiusHorizontal, tHorizontalAngleRadiant;
    cart2polar(aPositionStruct->LeftRight, aPositionStruct->BackFront, tRadiusHorizontal, tHorizontalAngleRadiant);
    aPositionStruct->LeftRightDegree = tHorizontalAngleRadiant * RAD_TO_DEG; // (0-180 degree, 0 is right, 90 degree is neutral)
    aPositionStruct->LeftRightDegree = aPositionStruct->LeftRightDegree + (PIVOT_NEUTRAL_OFFSET_DEGREE - 90); // compensate with measured neutral angle of servo

    // Account for the virtual claw length!
    tRadiusHorizontal -= CLAW_LENGTH_MILLIMETER;

    // Get vertical angle to claw - can be negative for claw below horizontal plane
    float tVerticalAngleToClawRadiant, tRadiusVertical;
    cart2polar(tRadiusHorizontal, aPositionStruct->DownUp, tRadiusVertical, tVerticalAngleToClawRadiant);

    // Solve arm inner angles
    float tAngleClawHorizontalRadiant;      // angle between vertical angle to claw and horizontal arm
    if (!cosangle(LIFT_ARM_LENGTH_MILLIMETER, HORIZONTAL_ARM_LENGTH_MILLIMETER, tRadiusVertical, tAngleClawHorizontalRadiant)) {
        Serial.print(F("Cannot solve 1 "));
        printPosition(aPositionStruct);
        return false;
    }
    float tAngleHorizontalLiftRadiant;  // angle between horizontal and lift arm
    if (!cosangle(tRadiusVertical, HORIZONTAL_ARM_LENGTH_MILLIMETER, LIFT_ARM_LENGTH_MILLIMETER, tAngleHorizontalLiftRadiant)) {
        Serial.print(F("Cannot solve 2 "));
        printPosition(aPositionStruct);
        return false;
    }
    bool tRetval = true;

    aPositionStruct->BackFrontDegree = (HALF_PI - (tVerticalAngleToClawRadiant + tAngleClawHorizontalRadiant)) * RAD_TO_DEG;
    // Now BackFrontDegree == 0 is vertical, front > 0, back < 0
    aPositionStruct->BackFrontDegree += HORIZONTAL_NEUTRAL_OFFSET_DEGREE;

    aPositionStruct->DownUpDegree = (tVerticalAngleToClawRadiant + tAngleClawHorizontalRadiant + tAngleHorizontalLiftRadiant - PI)
            * RAD_TO_DEG;
    // Now DownUpDegree == 0 is horizontal, up > 0, down < 0
    aPositionStruct->DownUpDegree += LIFT_NEUTRAL_OFFSET_DEGREE;

    return tRetval;
}

/*
 * Forward kinematics: servo angle -> X,Y,Z
 */

void polar2cart(float aRadius, float aAngleRadiant, float& aXValue, float& aYValue) {
    aXValue = aRadius * cos(aAngleRadiant);
    aYValue = aRadius * sin(aAngleRadiant);
}

/*
 * Get X,Y,Z from angles
 */
void unsolve(struct ArmPosition * aPositionStruct) {
    int tInputAngle;
    float tHeightOfHorizontalArm, tHorizontalArmHorizontalShift, tLiftHorizontal, tLiftHeight, tInputAngleRad;

    // input angle: horizontal = 0 vertical = 90
    tInputAngle = aPositionStruct->BackFrontDegree;
    tInputAngle -= HORIZONTAL_NEUTRAL_OFFSET_DEGREE;
    // here we have:  0 is vertical, front > 0, back < 0
    tInputAngle *= -1;
    // here we have:  0 is vertical, front < 0, back > 0
    tInputAngle += 90;
    tInputAngleRad = tInputAngle * DEG_TO_RAD;
#ifdef TRACE
    Serial.print("InputAngle BackFront=");
    Serial.print(tInputAngle);
#endif
    polar2cart(HORIZONTAL_ARM_LENGTH_MILLIMETER, tInputAngleRad, tHorizontalArmHorizontalShift, tHeightOfHorizontalArm);

    // input angle: horizontal = 0, up > 0, down < 0
    tInputAngle = aPositionStruct->DownUpDegree;
    tInputAngle -= LIFT_NEUTRAL_OFFSET_DEGREE;
#ifdef TRACE
    Serial.print(" InputAngle DownUp=");
    Serial.print(tInputAngle);
#endif
    tInputAngleRad = tInputAngle * DEG_TO_RAD;
    polar2cart(LIFT_ARM_LENGTH_MILLIMETER, tInputAngleRad, tLiftHorizontal, tLiftHeight);
    aPositionStruct->DownUp = tHeightOfHorizontalArm + tLiftHeight;

    // Add horizontal length
    float tHorizontalArmAndClawShift = tHorizontalArmHorizontalShift + tLiftHorizontal + CLAW_LENGTH_MILLIMETER;

    // input is horizontal plane angle: forward = 0 right > 0 left < 0
    tInputAngle = aPositionStruct->LeftRightDegree;
    tInputAngle -= PIVOT_NEUTRAL_OFFSET_DEGREE;
    // here we have horizontal plane angle: forward = 0 left > 0 right < 0
    tInputAngle *= -1;
#ifdef TRACE
    Serial.print(" InputAngle LeftRight=");
    Serial.println(tInputAngle);
#endif
    tInputAngleRad = tInputAngle * DEG_TO_RAD;
    polar2cart(tHorizontalArmAndClawShift, tInputAngleRad, aPositionStruct->BackFront, aPositionStruct->LeftRight);
}

void printPosition(struct ArmPosition * aPositionStruct) {
    Serial.print("LeftRight=");
    Serial.print(aPositionStruct->LeftRight);
    Serial.print("|");
    Serial.print(aPositionStruct->LeftRightDegree);
    Serial.print(" BackFront=");
    Serial.print(aPositionStruct->BackFront);
    Serial.print("|");
    Serial.print(aPositionStruct->BackFrontDegree);
    Serial.print(" DownUp=");
    Serial.print(aPositionStruct->DownUp);
    Serial.print("|");
    Serial.println(aPositionStruct->DownUpDegree);
}

void printPositionShort(struct ArmPosition * aPositionStruct) {
    Serial.print(aPositionStruct->LeftRight);
    Serial.print(" ");
    Serial.print(aPositionStruct->BackFront);
    Serial.print(" ");
    Serial.print(aPositionStruct->DownUp);
    Serial.print(" <-> ");

    Serial.print(aPositionStruct->LeftRightDegree);
    Serial.print(" ");
    Serial.print(aPositionStruct->BackFrontDegree);
    Serial.print(" ");
    Serial.println(aPositionStruct->DownUpDegree);
}

void printPositionShortWithUnits(struct ArmPosition * aPositionStruct) {
    Serial.print("X=");
    Serial.print(aPositionStruct->LeftRight);
    Serial.print("mm, Y=");
    Serial.print(aPositionStruct->BackFront);
    Serial.print("mm, Z=");
    Serial.print(aPositionStruct->DownUp);
    Serial.print("mm <-> ");

    Serial.print(aPositionStruct->LeftRightDegree);
    Serial.print(", ");
    Serial.print(aPositionStruct->BackFrontDegree);
    Serial.print(", ");
    Serial.print(aPositionStruct->DownUpDegree);
    Serial.println(" degree");

}
