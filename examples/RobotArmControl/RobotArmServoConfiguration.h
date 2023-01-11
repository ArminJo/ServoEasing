/*
 * RobotArmServoConfiguration.h
 *
 * To calibrate your robot arm, open the Serial Monitor, move the arm manually and change the microsecond values
 * for the PIVOT_MICROS_AT_*, LIFT_MICROS_AT_*, HORIZONTAL_MICROS_AT_* and CLAW_MICROS_AT_* positions below.
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
 *
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
 * This values seem to be constant values, the claw and base height can be different for MeArm build variants.
 */
#define HORIZONTAL_ARM_LENGTH_MILLIMETER       80
#define LIFT_ARM_LENGTH_MILLIMETER             80
#define CLAW_LENGTH_MILLIMETER                 68 // Length from wrist to hand PLUS base center to shoulder
// There are at least two claw variants available, so define values for it at the individual configurations
#define MINIMUM_HEIGHT_MILLIMETER              (CLAW_HEIGHT_OVER_GROUND_PLANE - ORIGIN_HEIGHT_OVER_GROUND_PLANE) // -50
#define MAX_X_VALUE_FOR_GROUND                120 // Y value if X == 0 and claw is at its foremost position
#define HORIZONTAL_NEUTRAL_MILLIMETER          LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER
#define VERTICAL_NEUTRAL_MILLIMETER            HORIZONTAL_ARM_LENGTH_MILLIMETER

// Index into (external) servo array. Order must be the same as of definitions in main.
#define SERVO_BASE_PIVOT    0
#define SERVO_HORIZONTAL    1
#define SERVO_LIFT          2
#define SERVO_CLAW          3
#define NUMBER_OF_SERVOS    4

/*
 * Trims are chosen, so that 0, 0, 0 results in the neutral position of the robot arm.
 * Neutral means: pivot direction forward, and both arms rectangular up and forward
 * In neutral position X=0, Y=LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER, Z=HORIZONTAL_ARM_LENGTH_MILLIMETER
 * Change this values to reflect your assembly
 */
#if defined(ROBOT_ARM_1)
#define ORIGIN_HEIGHT_OVER_GROUND_PLANE        68 // Height of Z=0 over ground, depends of the screw length used and the bumper attached on the base plate
#define CLAW_HEIGHT_OVER_GROUND_PLANE          20 // Height of claw position over ground if first parts of claw touches the ground

// Values for my MG90 clones. They differ from default values :-(
// operate pivot servo from -90° to +90°
#define PIVOT_MICROS_AT_PLUS_70_DEGREE        2400 // Left the MG90 servos are not capable of full 180°
#define PIVOT_MICROS_AT_MINUS_70_DEGREE        700 // Right

// operate lift servo from 30° to -90°
#define LIFT_MICROS_AT_0_NEUTRAL              1670 // 2. arm is horizontal forward
#define LIFT_MICROS_AT_MINUS_90_DEGREE         690 // 2. arm is vertical down
#define LIFT_MINIMUM_DEGREE                    -90 // 2. arm is vertical down
#define LIFT_MAXIMUM_DEGREE                     30 // Mechanic has contact with bottom plate - not used yet

// operate horizontal servo from -50° to 90°
#define HORIZONTAL_MICROS_AT_0_NEUTRAL        1180 // 1. arm is vertical up
#define HORIZONTAL_MICROS_AT_90_DEGREE        2240 // 1. arm is vertical forward
#define HORIZONTAL_MAXIMUM_DEGREE               90 // 1. arm is vertical forward
#define HORIZONTAL_MINIMUM_DEGREE              -50 // Mechanic is folded

// operate claw servo from 0° to 180°
#define CLAW_MICROS_AT_CLOSE                  1080 // The higher this value, the lower the degree / the closer the claw
#define CLAW_MICROS_AT_180_DEGREE              750 // Value for 180° open claw (each side has 90°)
#define CLAW_MAXIMUM_DEGREE                    180
#define CLAW_CLOSE_DEGREE                        0 // 0° is close by definition

#elif defined(ROBOT_ARM_2)
#define ORIGIN_HEIGHT_OVER_GROUND_PLANE        68 // Height of Z=0 over ground, depends of the screw length used and the bumper attached on the base plate
#define CLAW_HEIGHT_OVER_GROUND_PLANE          18 // Height of claw position over ground if first parts of claw touches the ground

// operate pivot servo from -90° to +90°
#define PIVOT_MICROS_AT_PLUS_70_DEGREE        2400 // Left the MG90 servos are not capable of full 180°
#define PIVOT_MICROS_AT_MINUS_70_DEGREE        700 // Right

// operate lift servo from 30° to -90°
#define LIFT_MICROS_AT_0_NEUTRAL              1700 // 2. arm is horizontal forward
#define LIFT_MICROS_AT_MINUS_90_DEGREE         630 // 2. arm is vertical down
#define LIFT_MINIMUM_DEGREE                    -90 // 2. arm is vertical down
#define LIFT_MAXIMUM_DEGREE                     30 // Mechanic has contact with bottom plate - not used yet

// operate horizontal servo from -50° to 90°
#define HORIZONTAL_MICROS_AT_0_NEUTRAL        1170 // 1. arm is vertical up
#define HORIZONTAL_MICROS_AT_90_DEGREE        2210 // 1. arm is vertical forward
#define HORIZONTAL_MAXIMUM_DEGREE               90 // 1. arm is vertical forward
#define HORIZONTAL_MINIMUM_DEGREE              -50 // Mechanic is folded

// operate claw servo from 0° to 180°
#define CLAW_MICROS_AT_CLOSE                  1700
#define CLAW_MICROS_AT_180_DEGREE              300 // Computed value for 180" open claw (each side has 90°) since my claw cannot really open to 180°
#define CLAW_MAXIMUM_DEGREE                    180
#define CLAW_CLOSE_DEGREE                        0 // 0° is close by definition

#else
/*
 * Default uncalibrated value
 */
#define ORIGIN_HEIGHT_OVER_GROUND_PLANE         68 // Height of Z=0 over ground, depends of the screw length used and the bumper attached on the base plate
#define CLAW_HEIGHT_OVER_GROUND_PLANE           20 // Height of claw position over ground if first parts of claw touches the ground

// operate pivot servo from -90° to +90°
#define PIVOT_MICROS_AT_PLUS_70_DEGREE        2400 // Left the MG90 servos are not capable of full 180°
#define PIVOT_MICROS_AT_MINUS_70_DEGREE        544 // Right

// operate lift servo from 30° to -90°
#define LIFT_MICROS_AT_0_NEUTRAL              1800 // 2. arm is horizontal forward
#define LIFT_MICROS_AT_MINUS_90_DEGREE         544 // 2. arm is vertical down
#define LIFT_MINIMUM_DEGREE                    -90 // 2. arm is vertical down
#define LIFT_MAXIMUM_DEGREE                     30 // Mechanic has contact with bottom plate - not used yet

// operate horizontal servo from -50° to 90°
#define HORIZONTAL_MICROS_AT_0_NEUTRAL        1100 // 1. arm is vertical up
#define HORIZONTAL_MICROS_AT_90_DEGREE        2400 // 1. arm is vertical forward
#define HORIZONTAL_MAXIMUM_DEGREE               90 // 1. arm is vertical forward
#define HORIZONTAL_MINIMUM_DEGREE              -50 // Mechanic is folded

// operate claw servo from 0° to 180°
#define CLAW_MICROS_AT_CLOSE                  1200
#define CLAW_MICROS_AT_180_DEGREE              600 // Value for 180" open claw (each side has 90°)
#define CLAW_MAXIMUM_DEGREE                    180
#define CLAW_CLOSE_DEGREE                        0 // 0° is close by definition
#endif

#endif // _ROBOT_ARM_SERVO_CONFIGURATION_H
