/*
 * RobotArmServoConfiguration.h
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef SRC_ROBOTARMSERVOCONFIGURATION_H_
#define SRC_ROBOTARMSERVOCONFIGURATION_H_

#define PIVOT_SERVO_PIN         5
#define HORIZONTAL_SERVO_PIN    4
#define CLAW_SERVO_PIN          7
#define LIFT_SERVO_PIN          8

#define PIVOT_INPUT_PIN         A1
#define HORIZONTAL_INPUT_PIN    A2
#define LIFT_INPUT_PIN          A3
#define CLAW_INPUT_PIN          A6

/*
 * Geometry values of the robot arm
 */
#define HORIZONTAL_ARM_LENGTH_MILLIMETER    80
#define LIFT_ARM_LENGTH_MILLIMETER          80
#define CLAW_LENGTH_MILLIMETER              68 // Length from wrist to hand PLUS base center to shoulder
#define ORIGIN_HEIGHT_OVER_GROUND_PLANE     68 // Height of Z=0 over ground
#define CLAW_HEIGHT_OVER_GROUND_PLANE       18 // Height of claw position over ground if first parts of claw touches the ground
#define FRONT_VALUE_FOR_GROUND             120 // Y value if X == 0 and claw is at its lowest position

// Index into (external) servo array. Order must be the same as of definitions in main.
#define SERVO_BASE_PIVOT 0
#define SERVO_HORIZONTAL 1
#define SERVO_LIFT 2
#define SERVO_CLAW 3
#define NUMBER_OF_SERVOS 4

/*
 * Values for my MG90 clones. They have different values :-(
 */
#ifdef ROBOT_ARM_2
#define PIVOT_ZERO_DEGREE_VALUE_MICROS       460 // 1120 per 90 degree
#define PIVOT_AT_180_DEGREE_VALUE_MICROS    2620
#define PIVOT_MAX_DEGREE_VALUE_MICROS       2480
#define PIVOT_OFFSET                          90 // operate pivot servo from -90 to +90 degree
#define PIVOT_TRIM                            -1

// take default values
#define LIFT_ZERO_DEGREE_VALUE_MICROS    544
#define LIFT_AT_180_DEGREE_VALUE_MICROS 2400
#define HORIZONTAL_ZERO_DEGREE_VALUE_MICROS    544
#define HORIZONTAL_AT_180_DEGREE_VALUE_MICROS 2400

/*
 * Neutral values of the servos of the robot arm
 * Neutral means: pivot direction forward, and both arms rectangular up and forward
 * Change this values to reflect your assembly
 */
#define PIVOT_NEUTRAL_OFFSET_DEGREE          0 // virtual value. The real servo value uses PIVOT_OFFSET and PIVOT_TRIM
#define HORIZONTAL_NEUTRAL_OFFSET_DEGREE    62
#define LIFT_NEUTRAL_OFFSET_DEGREE         110

#define LIFT_MAX_ANGLE          140
#define CLAW_MAX_ANGLE          120

#define CLAW_START_ANGLE        CLAW_MAX_ANGLE
#define CLAW_CLOSE_ANGLE        CLAW_MAX_ANGLE
#define CLAW_OPEN_ANGLE         (CLAW_MAX_ANGLE - 30)
#else
#define PIVOT_ZERO_DEGREE_VALUE_MICROS       380
#define PIVOT_AT_180_DEGREE_VALUE_MICROS    2360
#define PIVOT_MAX_DEGREE_VALUE_MICROS       2570
#define PIVOT_OFFSET                          90 // operate pivot servo from -90 to +90 degree
#define PIVOT_TRIM                            -4

#define LIFT_ZERO_DEGREE_VALUE_MICROS    380
#define LIFT_AT_180_DEGREE_VALUE_MICROS 2300

#define HORIZONTAL_ZERO_DEGREE_VALUE_MICROS    500
#define HORIZONTAL_AT_180_DEGREE_VALUE_MICROS 2300

/*
 * Neutral values of the servos of the robot arm
 * Neutral means: pivot direction forward, and both arms rectangular up and forward
 * Change this values to reflect your assembly
 */
#define PIVOT_NEUTRAL_OFFSET_DEGREE          0 // virtual value. The real servo value uses PIVOT_OFFSET and PIVOT_TRIM
#define HORIZONTAL_NEUTRAL_OFFSET_DEGREE    68
#define LIFT_NEUTRAL_OFFSET_DEGREE         122

#define LIFT_MAX_ANGLE          160
#define CLAW_MAX_ANGLE          54

#define CLAW_OPEN_ANGLE         (CLAW_MAX_ANGLE - 30)
#define CLAW_START_ANGLE        CLAW_OPEN_ANGLE
#define CLAW_CLOSE_ANGLE        CLAW_MAX_ANGLE
#endif

#endif /* SRC_ROBOTARMSERVOCONFIGURATION_H_ */

#pragma once
