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
