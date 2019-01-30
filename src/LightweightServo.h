/*
 * LightweightServo.h
 *
 *  Created on: 02.01.2019
 *  Copyright (C) 2019  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 */

#include <stdint.h>

#ifndef LIGHTWEIGHT_SERVO_H_
#define LIGHTWEIGHT_SERVO_H_

/*
 * Lightweight servo library
 * Uses timer1 and Pin 9 + 10 as output
 */
void initLightweightServoPin9And10();
void initLightweightServoPin9_10(bool aUsePin9 = true, bool aUsePin10 = true);
void setLightweightServoPulseMicrosFor0And180Degree(int aMicrosecondsForServo0Degree, int a180DegreeValue);
int writeLightweightServo(int aValue, bool aUsePin9, bool aUpdateFast = false);
void writeMicrosecondsLightweightServo(int aMicroseconds, bool aUsePin9, bool aUpdateFast = false);
void write9(int aValue, bool aUpdateFast = false); // setLightweightServoPulsePin9 Channel A
void writeMicroseconds9(int aMicroseconds, bool aUpdateFast = false);
void write10(int aValue, bool aUpdateFast = false); // setLightweightServoPulsePin10 Channel B
void writeMicroseconds10(int aMicroseconds, bool aUpdateFast = false);

// convenience functions
int DegreeToMicrosecondsLightweightServo(int aValueDegree);
int MicrosecondsToDegreeLightweightServo(int aValueMicros);

#endif // LIGHTWEIGHT_SERVO_H_
