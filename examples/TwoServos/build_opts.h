/*
 * build_opts.h
 *
 *  List of build options for building Arduino code.
 *
 *  Copyright (C) 2021  Armin Joachimsmeyer
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

/*
 * Copied from Readme:
 | `USE_PCA9685_SERVO_EXPANDER` | disabled | ServoEasing.h | Enables the use of the PCA9685 I2C expander chip/board. |
 | `USE_SERVO_LIB` | disabled | ServoEasing.h | Use of PCA9685 normally disables use of regular servo library. You can force using of regular servo library by defining `USE_SERVO_LIB`. See [below](#using-pca9685-16-channel-servo-expander). |
 | `PROVIDE_ONLY_LINEAR_MOVEMENT` | disabled | ServoEasing.h | Saves up to 1540 bytes FLASH. |
 | `DISABLE_COMPLEX_FUNCTIONS` | disabled | ServoEasing.h | Disables the SINE, CIRCULAR, BACK, ELASTIC and BOUNCE easings. Saves up to 1850 bytes FLASH. |
 | `ENABLE_MICROS_AS_DEGREE_PARAMETER` | disabled | ServoEasing.h | Enables passing also microsecond values as (target angle) parameter (see OneServo example). This requires additional 128 Bytes FLASH. |
 | `PRINT_FOR_SERIAL_PLOTTER` | disabled | ServoEasing.h | Generate serial output for Arduino Plotter. |
 | `USE_LEIGHTWEIGHT_SERVO_LIB` | disabled | ServoEasing.h | Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple. See below. Saves up to 742 bytes FLASH and 42 bytes RAM. |
 */

/*
 * For use with e.g. the Adafruit PCA9685 16-Channel Servo Driver board. It has a resolution of 4096 per 20 ms => 4.88 µs per step/unit.
 * One PCA9685 has 16 outputs. You must modify MAX_EASING_SERVOS below, if you have more than one PCA9685 attached!
 * Use of PCA9685 normally disables use of regular servo library. You can force using of regular servo library by defining USE_SERVO_LIB
 * All internal values *MicrosecondsOrUnits now contains no more microseconds but PCA9685 units!!!
 */
//#define USE_PCA9685_SERVO_EXPANDER
//#define USE_SERVO_LIB // Force additional using of regular servo library in conjunction with PCA9685 library.

/*
 * If you need only the linear movement you may define `PROVIDE_ONLY_LINEAR_MOVEMENT`. This saves additional 1540 Bytes FLASH.
 */
//#define PROVIDE_ONLY_LINEAR_MOVEMENT
//
/*
 * Define `DISABLE_COMPLEX_FUNCTIONS` if space (1850 Bytes) matters.
 * It disables the SINE, CIRCULAR, BACK, ELASTIC and BOUNCE easings.
 * The saving comes mainly from avoiding the sin() cos() sqrt() and pow() library functions in this code.
 * If you need only a single complex easing function and want to save space,
 * you can specify it any time as a user function. See EaseQuadraticInQuarticOut() function in AsymmetricEasing example line 206.
 */
//#define DISABLE_COMPLEX_FUNCTIONS
//
/*
 * If you require passing microsecond values as parameter instead of degree values. This requires additional 128 Bytes FLASH.
 */
//#define ENABLE_MICROS_AS_DEGREE_PARAMETER
//#define THRESHOLD_VALUE_FOR_INTERPRETING_VALUE_AS_MICROSECONDS  400  // treat values less than 400 as angles in degrees, others are handled as microseconds
//#define PRINT_FOR_SERIAL_PLOTTER  // Enable this to generate output for Arduino Serial Plotter (Ctrl-Shift-L)
//
/*
 * If you have only one or two servos and an ATmega328, then you can save program space by defining symbol `USE_LEIGHTWEIGHT_SERVO_LIB`.
 * This saves 742 bytes FLASH and 42 bytes RAM.
 * Using Lightweight Servo library (or PCA9685 servo expander) makes the servo pulse generating immune
 * to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
 * If not using the Arduino IDE take care that Arduino Servo library sources are not compiled / included in the project.
 * Use of Lightweight Servo library disables use of regular servo library.
 */
#define USE_LEIGHTWEIGHT_SERVO_LIB
//
/*****************************************************************************************
 * Important definition of MAX_EASING_SERVOS !!!
 * If this value is smaller than the amount of servos declared,
 * attach() will return error and other library functions will not work as expected.
 * Of course all *AllServos*() functions and isOneServoMoving() can't work correctly!
 * Saves 4 byte RAM per servo.
 ****************************************************************************************/
//#define MAX_EASING_SERVOS 3
