#include <math.h>
#include "ik.h"
#include <Arduino.h> // for PI etc.
#include "ServoEasing.h" // for clipDegreeSpecial()

//#define TRACE

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
    Serial.print("aXValue=");
    Serial.print(aXValue);
    Serial.print(" aYValue=");
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
    Serial.print(" aAngleRadiant=");
    Serial.println(aAngleRadiant);
#endif
}

bool solve(struct ArmPosition * aPositionStruct) {
    // Get horizontal degree for servo (0-180 degree) and get radius for next computations
    float tRadiusHorizontal, tHorizontalAngleRadiant;
    cart2polar(aPositionStruct->LeftRight, aPositionStruct->BackFront, tRadiusHorizontal, tHorizontalAngleRadiant);
    aPositionStruct->LeftRightDegree = tHorizontalAngleRadiant * RAD_TO_DEG; // (0-180 degree, 90 degree is neutral)
    aPositionStruct->LeftRightDegree = clipDegreeSpecial(aPositionStruct->LeftRightDegree + (PIVOT_NEUTRAL_ANGLE - 90)); // compensate with measured neutral angle of servo

    // Account for the virtual claw length!
    tRadiusHorizontal -= CLAW_LENGTH_MILLIMETER;

    // Get vertical angle to claw - can be negative for claw below horizontal plane
    float tVerticalAngleToClawRadiant, tRadiusVertical;
    cart2polar(tRadiusHorizontal, aPositionStruct->DownUp, tRadiusVertical, tVerticalAngleToClawRadiant);

    // Solve arm inner angles
    float tAngleClawHorizontalRadiant;      // angle between vertical angle to claw and horizontal arm
    if (!cosangle(LIFT_ARM_LENGTH_MILLIMETER, HORIZONTAL_ARM_LENGTH_MILLIMETER, tRadiusVertical, tAngleClawHorizontalRadiant)) {
        return false;
    }
    float tAngleHorizontalLiftRadiant;  // angle between horizontal and lift arm
    if (!cosangle(tRadiusVertical, HORIZONTAL_ARM_LENGTH_MILLIMETER, LIFT_ARM_LENGTH_MILLIMETER, tAngleHorizontalLiftRadiant)) {
        return false;
    }
    bool tRetval = true;
    // Solve for servo angles from horizontal

    aPositionStruct->BackFrontDegree = (PI - (tVerticalAngleToClawRadiant + tAngleClawHorizontalRadiant)) * RAD_TO_DEG
            + HORIZONTAL_NEUTRAL_ANGLE;
    if (aPositionStruct->BackFrontDegree >= 90) {
        aPositionStruct->BackFrontDegree -= 90;
    } else {
        aPositionStruct->BackFrontDegree = 0;
        tRetval = false;
    }
    aPositionStruct->DownUpDegree = (tVerticalAngleToClawRadiant + tAngleClawHorizontalRadiant + tAngleHorizontalLiftRadiant - PI)
            * RAD_TO_DEG + LIFT_NEUTRAL_ANGLE;

    return tRetval;
}

// Solve angles!
bool solve(float x, float y, float z, float& a0, float& a1, float& a2) {
    // Solve top-down view
    float r, th0;
    cart2polar(y, x, r, th0);

    // Account for the wrist length!
    r -= CLAW_LENGTH_MILLIMETER;

    // In arm plane, convert to polar
    float ang_P, R;
    cart2polar(r, z, R, ang_P);

    // Solve arm inner angles as required
    float B, C;
    if (!cosangle(LIFT_ARM_LENGTH_MILLIMETER, HORIZONTAL_ARM_LENGTH_MILLIMETER, R, B))
        return false;
    if (!cosangle(R, HORIZONTAL_ARM_LENGTH_MILLIMETER, LIFT_ARM_LENGTH_MILLIMETER, C))
        return false;

    // Solve for servo angles from horizontal
    a0 = th0;
    a1 = ang_P + B;
    a2 = C + a1 - PI;

    return true;
}
