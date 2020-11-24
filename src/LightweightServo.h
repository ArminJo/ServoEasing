/*
 * LightweightServo.h
 *
 *  Copyright (C) 2019-2020  Armin Joachimsmeyer
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

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef LIGHTWEIGHT_SERVO_H_
#define LIGHTWEIGHT_SERVO_H_

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

#define VERSION_LIGHTWEIGHT_SERVO "1.1.0"
#define VERSION_LIGHTWEIGHT_SERVO_MAJOR 1
#define VERSION_LIGHTWEIGHT_SERVO_MINOR 1

#include <stdint.h>

/*
 * Commenting out this saves 70 bytes flash memory. You must then use the init function initLightweightServoPin9And10() manually.
 */
//#define DISABLE_SERVO_TIMER_AUTO_INITIALIZE
#define ISR1_COUNT_FOR_20_MILLIS 40000 // you can modify this if you have servos which accept a higher rate

/*
 * Lightweight servo library
 * Uses timer1 and Pin 9 + 10 as output
 */
void initLightweightServoPin9And10();
void initLightweightServoPin9();    // Disables Pin 10!
void initLightweightServoPin10();   // Disables Pin 9!
void initLightweightServoPin9_10(bool aUsePin9, bool aUsePin10);
void deinitLightweightServoPin9_10(bool aUsePin9, bool aUsePin10);

void setLightweightServoPulseMicrosFor0And180Degree(int aMicrosecondsForServo0Degree, int a180DegreeValue);
void setLightweightServoRefreshRate(unsigned int aRefreshPeriodMicroseconds);

int writeLightweightServo(int aDegree, bool aUsePin9, bool aUpdateFast = false);
void writeMicrosecondsLightweightServo(int aMicroseconds, bool aUsePin9, bool aUpdateFast = false);

void write9(int aDegree, bool aUpdateFast = false); // setLightweightServoPulsePin9 Channel A
void writeMicroseconds9(int aMicroseconds, bool aUpdateFast = false);
void writeMicroseconds9Direct(int aMicroseconds);

void write10(int aDegree, bool aUpdateFast = false); // setLightweightServoPulsePin10 Channel B
void writeMicroseconds10(int aMicroseconds, bool aUpdateFast = false);
void writeMicroseconds10Direct(int aMicroseconds);

// convenience functions
int DegreeToMicrosecondsLightweightServo(int aDegree);
int MicrosecondsToDegreeLightweightServo(int aMicroseconds);

#endif // AVR_ATmega328

/*
 * Version 1.1.0 - 11/2020
 * - Improved API.
 */

#endif // LIGHTWEIGHT_SERVO_H_

#pragma once
