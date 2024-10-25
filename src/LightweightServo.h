/*
 * LightweightServo.h
 *
 *  Copyright (C) 2019-2024  Armin Joachimsmeyer
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

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef _LIGHTWEIGHT_SERVO_H
#define _LIGHTWEIGHT_SERVO_H

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)

#define VERSION_LIGHTWEIGHT_SERVO "2.0.0"
#define VERSION_LIGHTWEIGHT_SERVO_MAJOR 2
#define VERSION_LIGHTWEIGHT_SERVO_MINOR 0

#include <stdint.h>

/*
 * Activating this saves 40 bytes program space. You must then use the init functions initLightweightServoPin*() manually.
 */
//#define DISABLE_SERVO_TIMER_AUTO_INITIALIZE
//
#define ISR_COUNT_FOR_20_MILLIS (F_CPU / (8 * 50)) // 40000 For 50 Hz, 20 ms using a prescaler of 8. You can modify this if you have servos which accept a higher rate
#define ISR_COUNT_FOR_2_5_MILLIS (F_CPU / (8 * 400)) // 5000 For 400 Hz, 2.5 ms using a prescaler of 8.

#if defined(__AVR_ATmega2560__)
#define LIGHTWEIGHT_SERVO_CHANNEL_A_PIN   46
#define LIGHTWEIGHT_SERVO_CHANNEL_B_PIN   45
#define LIGHTWEIGHT_SERVO_CHANNEL_C_PIN   44
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328PB__)
#define LIGHTWEIGHT_SERVO_CHANNEL_A_PIN    9
#define LIGHTWEIGHT_SERVO_CHANNEL_B_PIN   10
#endif

void initLightweightServoPins(); // currently only pin 44 = OC5C/PL5, 45 = OC5B/PL4 + 46 = OC5A/PL3 on 2560 and pin 9 = OC1A + 10 = OC1B on 328
void checkAndInitLightweightServoPin(uint8_t aPin);
void deinitLightweightServoPin(uint8_t aPin); // Set pin to input and disable non-inverting Compare Output mode

int writeLightweightServoPin(int aDegree, uint8_t aPin, bool aUpdateFast = false);
// The last parameter requires 8 byte more than DISABLE_SERVO_TIMER_AUTO_INITIALIZE, if false, but saves around 60 bytes anyway
void writeMicrosecondsLightweightServoPin(int aMicroseconds, uint8_t aPin, bool aUpdateFast = false, bool aDoAutoInit = true);

void setLightweightServoPulseMicrosFor0And180Degree(int aMicrosecondsForServo0Degree, int a180DegreeValue);
void setLightweightServoRefreshRate(unsigned int aRefreshPeriodMicroseconds);

// convenience functions
int DegreeToMicrosecondsLightweightServo(int aDegree);
int MicrosecondsToDegreeLightweightServo(int aMicroseconds);

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328PB__)
#define ISR1_COUNT_FOR_20_MILLIS    ISR_COUNT_FOR_20_MILLIS
#define ISR1_COUNT_FOR_2_5_MILLIS   ISR_COUNT_FOR_2_5_MILLIS

/*
 * Old and fast ATmega328 functions
 * Uses timer1 and Pin 9 + 10 as output
 */
void initLightweightServoPin9And10() __attribute__ ((deprecated ("Renamed to initLightweightServoPins()"))); // deprecated
void initLightweightServoPin9();    // Disables Pin 10!
void initLightweightServoPin10();   // Disables Pin 9!
void initLightweightServoPin9_10(bool aUsePin9, bool aUsePin10);
void deinitLightweightServoPin9_10(bool aUsePin9, bool aUsePin10);

void write9(int aDegree, bool aUpdateFast = false); // setLightweightServoPulsePin9 Channel A
void writeMicroseconds9(int aMicroseconds, bool aUpdateFast = false);
void writeMicroseconds9Direct(int aMicroseconds);

void write10(int aDegree, bool aUpdateFast = false); // setLightweightServoPulsePin10 Channel B
void writeMicroseconds10(int aMicroseconds, bool aUpdateFast = false);
void writeMicroseconds10Direct(int aMicroseconds);

int writeLightweightServo(int aDegree, bool aUsePin9, bool aUpdateFast = false);
void writeMicrosecondsLightweightServo(int aMicroseconds, bool aUsePin9, bool aUpdateFast = false);
#endif // Old and fast ATmega328 functions

class LightweightServo {
public:
    uint8_t attach(int aPin);
    uint8_t attach(int aPin, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree);
    void detach();
    void write(int aTargetDegreeOrMicrosecond);
    void writeMicroseconds(int aTargetMicrosecond); // Write pulse width in microseconds
    /*
     * Variables to enable adjustment for different servo types
     * 544 and 2400 are values compatible with standard arduino values
     */
    int MicrosecondsForServo0Degree = 544;
    int MicrosecondsForServo180Degree = 2400;
    uint8_t LightweightServoPin;
};

#endif // defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)

/*
 * Version 2.0.0 - 10/2024
 * - Improved API.
 * - Support for ATmega2560.
 *
 * Version 1.1.0 - 11/2020
 * - Improved API.
 */

#endif // _LIGHTWEIGHT_SERVO_H
