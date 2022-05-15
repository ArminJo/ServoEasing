/*
 * RobotArmServoConfiguration.h
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef _ROBOT_ARM_SERVO_CONFIGURATION_H
#define _ROBOT_ARM_SERVO_CONFIGURATION_H

// Must specify this before the include of "ServoEasing.hpp"
//#define USE_PCA9685_SERVO_EXPANDER    // Activate this to enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SOFT_I2C_MASTER           // Saves 1756 bytes program memory and 218 bytes RAM compared with Arduino Wire
//#define USE_SERVO_LIB                 // Activate this to force additional using of regular servo library.
//#define USE_LEIGHTWEIGHT_SERVO_LIB    // Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activate this to disable all but LINEAR movement. Saves up to 1540 bytes program memory.
//#define DISABLE_COMPLEX_FUNCTIONS     // Activate this to disable the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
#define MAX_EASING_SERVOS 4
//#define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle) parameter. Saves 128 bytes program memory.
//#define DEBUG                         // Activate this to generate lots of lovely debug output for this library.

/*
 * Specify which easings types should be available.
 * If no easing is defined, all easings are active.
 * This must be done before the #include "ServoEasing.hpp"
 */
#define ENABLE_EASE_QUADRATIC
//#define ENABLE_EASE_CUBIC
#define ENABLE_EASE_QUARTIC
//#define ENABLE_EASE_SINE
#define ENABLE_EASE_CIRCULAR
//#define ENABLE_EASE_BACK
//#define ENABLE_EASE_ELASTIC
#define ENABLE_EASE_BOUNCE
//#define ENABLE_EASE_PRECISION
#define ENABLE_EASE_USER

#define PIVOT_SERVO_PIN         4
#define HORIZONTAL_SERVO_PIN    3
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
#define MINIMUM_HEIGHT                      (CLAW_HEIGHT_OVER_GROUND_PLANE - ORIGIN_HEIGHT_OVER_GROUND_PLANE)
#define FRONT_VALUE_FOR_GROUND             120 // Y value if X == 0 and claw is at its lowest position

// Index into (external) servo array. Order must be the same as of definitions in main.
#define SERVO_BASE_PIVOT 0
#define SERVO_HORIZONTAL 1
#define SERVO_LIFT 2
#define SERVO_CLAW 3
#define NUMBER_OF_SERVOS 4

/*
 * Trims are chosen, so that 0, 0, 0 results in the neutral position of the robot arm.
 * Neutral means: pivot direction forward, and both arms rectangular up and forward
 * In neutral position X=0, Y=LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER, Z=HORIZONTAL_ARM_LENGTH_MILLIMETER
 * Change this values to reflect your assembly
 */
#if defined(ROBOT_ARM_2)
// Values for my MG90 clones. They differ from default values :-(
#define PIVOT_ZERO_DEGREE_VALUE_MICROS         460 // 1120 per 90 degree
#define PIVOT_AT_180_DEGREE_VALUE_MICROS      2620
#define PIVOT_MAX_DEGREE_VALUE_MICROS         2480
#define PIVOT_TRIM                              89 // trim to operate servo from -90 to +90 degree

// take default values here
#define LIFT_ZERO_DEGREE_VALUE_MICROS          544
#define LIFT_AT_180_DEGREE_VALUE_MICROS       2400
#define LIFT_TRIM                              112 // trim to operate servo from -90 to +90 degree
#define LIFT_MINIMUM_DEGREE                     30 // Has contact with bottom plate
#define LIFT_MAXIMUM_DEGREE                   -150 // Servo arm is vertical

#define HORIZONTAL_ZERO_DEGREE_VALUE_MICROS    544
#define HORIZONTAL_AT_180_DEGREE_VALUE_MICROS 2400
#define HORIZONTAL_TRIM                         62 // trim to operate servo from -90 to +90 degree
#define HORIZONTAL_MINIMUM_DEGREE -HORIZONTAL_TRIM // Fold back position
#define HORIZONTAL_MAXIMUM_DEGREE              100 // Horizontal forward position

#define CLAW_MAXIMUM_DEGREE                    120

#define CLAW_START_DEGREE      CLAW_MAXIMUM_DEGREE
#define CLAW_CLOSE_DEGREE      CLAW_MAXIMUM_DEGREE
#define CLAW_OPEN_DEGREE      (CLAW_MAXIMUM_DEGREE - 30)
#else
#define PIVOT_ZERO_DEGREE_VALUE_MICROS         380
#define PIVOT_AT_180_DEGREE_VALUE_MICROS      2360
#define PIVOT_MAX_DEGREE_VALUE_MICROS         2570
#define PIVOT_OFFSET                            90 // operate pivot servo from -90 to +90 degree
#define PIVOT_TRIM                              -4

#define LIFT_ZERO_DEGREE_VALUE_MICROS          380
#define LIFT_AT_180_DEGREE_VALUE_MICROS       2300
#define LIFT_TRIM                              122 // trim to operate servo from -90 to +90 degree
#define LIFT_MINIMUM_DEGREE                    240 // Has contact with bottom plate
#define LIFT_MAXIMUM_DEGREE                    -30 // Servo arm is vertical

#define HORIZONTAL_ZERO_DEGREE_VALUE_MICROS    500
#define HORIZONTAL_AT_180_DEGREE_VALUE_MICROS 2300
#define HORIZONTAL_TRIM                         68 // trim to operate servo from -90 to +90 degree
#define HORIZONTAL_MINIMUM_DEGREE -HORIZONTAL_TRIM // Fold back position
#define HORIZONTAL_MAXIMUM_DEGREE              100 // Horizontal forward position

#define LIFT_MAX_ANGLE          160
#define CLAW_MAXIMUM_DEGREE                     54

#define CLAW_START_DEGREE      CLAW_MAXIMUM_DEGREE
#define CLAW_CLOSE_DEGREE      CLAW_MAXIMUM_DEGREE
#define CLAW_OPEN_DEGREE      (CLAW_MAXIMUM_DEGREE - 30)
#endif

#endif // _ROBOT_ARM_SERVO_CONFIGURATION_H
