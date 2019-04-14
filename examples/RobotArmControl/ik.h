#ifndef IK_H_INCLUDED
#define IK_H_INCLUDED

#include <stdint.h>

/*
 * Geometry of the mechanic
 */
#define PIVOT_NEUTRAL_ANGLE         98

#define HORIZONTAL_ARM_LENGTH_MILLIMETER 80 //Shoulder to elbow length
#define HORIZONTAL_NEUTRAL_ANGLE    60
#define LIFT_ARM_LENGTH_MILLIMETER  80 //Elbow to wrist length
#define LIFT_NEUTRAL_ANGLE          115

#define CLAW_LENGTH_MILLIMETER 68 //Length from wrist to hand PLUS base center to shoulder

/*
 * First part of the name describes negative values. LeftRight value negative means left side.
 */
struct ArmPosition {
    float LeftRight;
    float BackFront;
    float DownUp;
    uint8_t LeftRightDegree;
    uint8_t BackFrontDegree;
    uint8_t DownUpDegree;
};

// Get polar coords from cartesian ones
void cart2polar(float a, float b, float& r, float& theta);

// Get angle from a triangle using cosine rule
bool cosangle(float opp, float adj1, float adj2, float& theta);

// Solve angles!
bool solve(float x, float y, float z, float& a0, float& a1, float& a2);
bool solve(struct ArmPosition * aPositionStruct);

#endif // IK_H_INCLUDED
