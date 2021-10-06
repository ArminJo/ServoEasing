/*
 *  LightweightServo.cpp
 *
 *  Lightweight Servo implementation only for pin 9 and 10 using only timer1 hardware and no interrupts or other overhead.
 *  Provides auto initialization.
 *  300 bytes code size / 4 bytes RAM including auto initialization compared to 700 / 48 bytes for Arduino Servo library.
 *  8 bytes for each call to setLightweightServoPulse...
 *
 *  Copyright (C) 2019  Armin Joachimsmeyer
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

#ifndef LIGHTWEIGHT_SERVO_HPP
#define LIGHTWEIGHT_SERVO_HPP

#include <Arduino.h>

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#include "LightweightServo.h"

/*
 * Variables to enable adjustment for different servo types
 * 544 and 2400 are values compatible with standard arduino values
 * 4 bytes RAM compared to 48 bytes for standard Arduino library
 */
int sMicrosecondsForServo0Degree = 544;
int sMicrosecondsForServo180Degree = 2400;

/*
 * Use 16 bit timer1 for generating 2 servo signals entirely by hardware without any interrupts.
 * Use FastPWM mode and generate pulse at start of the 20 ms period
 * The 2 servo signals are tied to pin 9 and 10 of an 328.
 * Attention - both pins are set to OUTPUT here!
 * 32 bytes code size
 */
void initLightweightServoPin9And10() {
    /*
     * Periods below 20 ms gives problems with long signals i.e. the positioning is not possible
     */
    DDRB |= _BV(DDB1) | _BV(DDB2);                // set pins OC1A = PortB1 -> PIN 9 and OC1B = PortB2 -> PIN 10 to output direction
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); // FastPWM Mode mode TOP (20 ms) determined by ICR1 - non-inverting Compare Output mode OC1A+OC1B
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);    // set prescaler to 8, FastPWM mode mode bits WGM13 + WGM12
    ICR1 = ISR1_COUNT_FOR_20_MILLIS;                 // set period to 20 ms
    // do not set counter here, since with counter = 0 (default) no output signal is generated.
}

/*
 * Use 16 bit timer1 for generating 2 servo signals entirely by hardware without any interrupts.
 * Use FastPWM mode and generate pulse at start of the 20 ms period
 * The 2 servo signals are tied to pin 9 and 10 of an ATMega328.
 * Attention - the selected pin is set to OUTPUT here!
 * 54 bytes code size
 */
void initLightweightServoPin9_10(bool aUsePin9, bool aUsePin10) {

    uint8_t tNewTCCR1A = TCCR1A & (_BV(COM1A1) | _BV(COM1B1)); // keep existing COM1A1 and COM1B1 settings
    tNewTCCR1A |= _BV(WGM11);                       // FastPWM Mode mode TOP (20 ms) determined by ICR1

    if (aUsePin9) {
        DDRB |= _BV(DDB1);                          // set OC1A = PortB1 -> PIN 9 to output direction
        tNewTCCR1A |= _BV(COM1A1);                  // non-inverting Compare Output mode OC1A
        OCR1A = 0xFFFF;                                 // Set counter > ICR1 here, to avoid output signal generation.
   }
    if (aUsePin10) {
        DDRB |= _BV(DDB2);                          // set OC1B = PortB2 -> PIN 10 to output direction
        tNewTCCR1A |= _BV(COM1B1);                  // non-inverting Compare Output mode OC1B
        OCR1B = 0xFFFF;                                 // Set counter > ICR1 here, to avoid output signal generation.
    }
    TCCR1A = tNewTCCR1A;
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);   // set prescaler to 8, FastPWM Mode mode bits WGM13 + WGM12
    ICR1 = ISR1_COUNT_FOR_20_MILLIS;                // set period to 20 ms
}

/*
 * Disables Pin 10!
 */
void initLightweightServoPin9() {
    DDRB |= _BV(DDB1);                              // set OC1A = PortB1 -> PIN 9 to output direction
    TCCR1A = _BV(WGM11) | _BV(COM1A1);  //  FastPWM Mode mode TOP (20 ms) determined by ICR1, non-inverting Compare Output mode OC1A
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);   // set prescaler to 8, FastPWM Mode mode bits WGM13 + WGM12
    ICR1 = ISR1_COUNT_FOR_20_MILLIS;                // set period to 20 ms
    OCR1A = 0xFFFF;                                 // Set counter > ICR1 here, to avoid output signal generation.
}
/*
 * Disables Pin 9!
 */
void initLightweightServoPin10() {
    DDRB |= _BV(DDB2);                              // set OC1B = PortB2 -> PIN 10 to output direction
    TCCR1A = _BV(WGM11) | _BV(COM1B1);  //  FastPWM Mode mode TOP (20 ms) determined by ICR1, non-inverting Compare Output mode OC1B
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);   // set prescaler to 8, FastPWM Mode mode bits WGM13 + WGM12
    ICR1 = ISR1_COUNT_FOR_20_MILLIS;                // set period to 20 ms
    OCR1B = 0xFFFF;                                 // Set counter > ICR1 here, to avoid output signal generation.
}

void deinitLightweightServoPin9_10(bool aUsePin9, bool aUsePin10) {
    if (aUsePin9) {
        DDRB &= ~(_BV(DDB1));           // set OC1A = PortB1 -> PIN 9 to input direction
        TCCR1A &= ~(_BV(COM1A1));       // disable non-inverting Compare Output mode OC1A
    }
    if (aUsePin10) {
        DDRB &= ~(_BV(DDB2));           // set OC1B = PortB2 -> PIN 10 to input direction
        TCCR1A &= ~(_BV(COM1B1));       // disable non-inverting Compare Output mode OC1B
    }
}

/*
 * If value is below 180 then assume degree, otherwise assume microseconds
 * If aUpdateFast then enable starting a new output pulse if more than 5 ms since last one, some servo might react faster in this mode.
 * If aUsePin9 is false, then Pin10 is used
 * 236 / 186(without auto init) bytes code size
 */
int writeLightweightServo(int aDegree, bool aUsePin9, bool aUpdateFast) {
    if (aDegree <= 180) {
        aDegree = DegreeToMicrosecondsLightweightServo(aDegree);
    }
    writeMicrosecondsLightweightServo(aDegree, aUsePin9, aUpdateFast);
    return aDegree;
}

void writeMicrosecondsLightweightServo(int aMicroseconds, bool aUsePin9, bool aUpdateFast) {
#ifndef DISABLE_SERVO_TIMER_AUTO_INITIALIZE
    // auto initialize
    if ((TCCR1B != (_BV(WGM13) | _BV(WGM12) | _BV(CS11))) || (aUsePin9 && ((TCCR1A & ~_BV(COM1B1)) != (_BV(COM1A1) | _BV(WGM11))))
            || (!aUsePin9 && ((TCCR1A & ~_BV(COM1A1)) != (_BV(COM1B1) | _BV(WGM11))))) {
        initLightweightServoPin9_10(aUsePin9, !aUsePin9);
    }
#endif
    // since the resolution is 1/2 of microsecond
    aMicroseconds *= 2;
    if (aUpdateFast) {
        uint16_t tTimerCount = TCNT1;
        if (tTimerCount > 5000) {
            // more than 2.5 ms since last pulse -> start a new one
            TCNT1 = ICR1 - 1;
        }
    }
    if (aUsePin9) {
        OCR1A = aMicroseconds;
    } else {
        OCR1B = aMicroseconds;
    }
}

/*
 * Sets the period of the servo pulses. Reasonable values are 2500 to 20000 microseconds.
 * No parameter checking is done here!
 */
void setLightweightServoRefreshRate(unsigned int aRefreshPeriodMicroseconds) {
    ICR1 = aRefreshPeriodMicroseconds * 2;
}
/*
 * Set the mapping pulse width values for 0 and 180 degree
 */
void setLightweightServoPulseMicrosFor0And180Degree(int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree) {
    sMicrosecondsForServo0Degree = aMicrosecondsForServo0Degree;
    sMicrosecondsForServo180Degree = aMicrosecondsForServo180Degree;
}

/*
 * Pin 9 / Channel A. If value is below 180 then assume degree, otherwise assume microseconds
 */
void write9(int aDegree, bool aUpdateFast) {
    writeLightweightServo(aDegree, true, aUpdateFast);
}

void writeMicroseconds9(int aMicroseconds, bool aUpdateFast) {
    writeMicrosecondsLightweightServo(aMicroseconds, true, aUpdateFast);
}

/*
 * Without auto initialize!
 */
void writeMicroseconds9Direct(int aMicroseconds) {
    OCR1A = aMicroseconds * 2;
}

/*
 * Pin 10 / Channel B
 */
void write10(int aDegree, bool aUpdateFast) {
    writeLightweightServo(aDegree, false, aUpdateFast);
}

void writeMicroseconds10(int aMicroseconds, bool aUpdateFast) {
    writeMicrosecondsLightweightServo(aMicroseconds, false, aUpdateFast);
}

/*
 * Without auto initialize!
 */
void writeMicroseconds10Direct(int aMicroseconds) {
    OCR1B = aMicroseconds * 2;
}

/*
 * Conversion functions
 */
int DegreeToMicrosecondsLightweightServo(int aDegree) {
    return (map(aDegree, 0, 180, sMicrosecondsForServo0Degree, sMicrosecondsForServo180Degree));
}

int MicrosecondsToDegreeLightweightServo(int aMicroseconds) {
    return map(aMicroseconds, sMicrosecondsForServo0Degree, sMicrosecondsForServo180Degree, 0, 180);
}

#endif // defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#endif // #ifndef LIGHTWEIGHT_SERVO_HPP
#pragma once
