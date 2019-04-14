#ifndef ROBOT_ARM_KINEMATICS_H_
#define ROBOT_ARM_KINEMATICS_H_

#include <stdint.h>

/*
 * Geometry of the mechanic of the robot arm
 */
#define PIVOT_NEUTRAL_OFFSET_DEGREE         98

#define HORIZONTAL_ARM_LENGTH_MILLIMETER    80
#define HORIZONTAL_NEUTRAL_OFFSET_DEGREE    60
#define LIFT_ARM_LENGTH_MILLIMETER          80
#define LIFT_NEUTRAL_OFFSET_DEGREE          115

#define CLAW_LENGTH_MILLIMETER 68 //Length from wrist to hand PLUS base center to shoulder

/*
 * First part of the name describes negative values. LeftRight value negative means left side.
 */
struct ArmPosition {
    float LeftRight;
    float BackFront;
    float DownUp;
    uint8_t LeftRightDegree; // horizontal plane angle: forward = 0 left > 0 right < 0  | add PIVOT_NEUTRAL_OFFSET_DEGREE to use for servo
    uint8_t BackFrontDegree; // 0 is vertical, front > 0, back < 0 | add HORIZONTAL_NEUTRAL_OFFSET_DEGREE to use for servo
    uint8_t DownUpDegree;    // 0 is horizontal, up > 0, down < 0  | add LIFT_NEUTRAL_OFFSET_DEGREE to use for servo
};

/*
 * Inverse kinematics: X,Y,Z -> servo angle
 */
void cart2polar(float a, float b, float& r, float& theta);
bool cosangle(float opp, float adj1, float adj2, float& theta);
bool solve(struct ArmPosition * aPositionStruct);

/*
 * Forward kinematics: servo angle -> X,Y,Z
 */
void polar2cart(float r, float theta, float& a, float& b);
void unsolve(struct ArmPosition * aPositionStruct);
#endif // ROBOT_ARM_KINEMATICS_H_
