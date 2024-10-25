/*
 *  LightweightServo.hpp
 *
 *  Lightweight Servo implementation timer hardware and no interrupts or other overhead.
 *  Supported pins:
 *  ATmega328: pin 44 = OC5C/PL5, 45 = OC5B/PL4 and 46 = OC5A/PL3 using only timer5 hardware
 *  ATmega2560 pin 9 = OC1A and 10 = OC1B on 328 using only timer1 hardware
 *  Provides auto initialization.
 *  300 bytes code size / 4 bytes RAM including auto initialization compared to 700 / 48 bytes for Arduino Servo library.
 *  8 bytes for each call to setLightweightServoPulse...
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

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)
#include "LightweightServo.h"

#ifndef _LIGHTWEIGHT_SERVO_HPP
#define _LIGHTWEIGHT_SERVO_HPP

#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file
#endif

/*
 * Variables to enable adjustment for different servo types
 * 544 and 2400 are values compatible with standard arduino values
 * 4 bytes RAM compared to 48 bytes for standard Arduino library
 */
int sMicrosecondsForServo0Degree = 544;
int sMicrosecondsForServo180Degree = 2400;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)  || defined (__AVR_ATmega328PB__)
/*
 * Use 16 bit timer1 for generating 2 servo signals entirely by hardware without any interrupts.
 * Use FastPWM mode and generate pulse at start of the 20 ms period
 * The 2 servo signals are tied to pin 9 and 10 of an 328.
 * Attention - both pins are set to OUTPUT here!
 * 32 bytes code size
 * Supports pin 9 = OC1A + 10 = OC1B
 */
void initLightweightServoPin9And10() {
    initLightweightServoPins();
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
        OCR1A = UINT16_MAX;                         // Set counter > ICR1 here, to avoid output signal generation.
    }
    if (aUsePin10) {
        DDRB |= _BV(DDB2);                          // set OC1B = PortB2 -> PIN 10 to output direction
        tNewTCCR1A |= _BV(COM1B1);                  // non-inverting Compare Output mode OC1B
        OCR1B = UINT16_MAX;                         // Set counter > ICR1 here, to avoid output signal generation.
    }
    TCCR1A = tNewTCCR1A;
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);   // set prescaler to 8, FastPWM Mode mode bits WGM13 + WGM12
    ICR1 = ISR_COUNT_FOR_20_MILLIS;                // set period to 20 ms
}

/*
 * Disables Pin 10!
 */
void initLightweightServoPin9() {
    DDRB |= _BV(DDB1);                              // set OC1A = PortB1 -> PIN 9 to output direction
    TCCR1A = _BV(WGM11) | _BV(COM1A1);  //  FastPWM Mode mode TOP (20 ms) determined by ICR1, non-inverting Compare Output mode OC1A
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);   // set prescaler to 8, FastPWM Mode mode bits WGM13 + WGM12
    ICR1 = ISR_COUNT_FOR_20_MILLIS;                // set period to 20 ms
    OCR1A = UINT16_MAX;                             // Set counter > ICR1 here, to avoid output signal generation.
}
/*
 * Disables Pin 9!
 */
void initLightweightServoPin10() {
    DDRB |= _BV(DDB2);                              // set OC1B = PortB2 -> PIN 10 to output direction
    TCCR1A = _BV(WGM11) | _BV(COM1B1);  //  FastPWM Mode mode TOP (20 ms) determined by ICR1, non-inverting Compare Output mode OC1B
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);   // set prescaler to 8, FastPWM Mode mode bits WGM13 + WGM12
    ICR1 = ISR_COUNT_FOR_20_MILLIS;                // set period to 20 ms
    OCR1B = UINT16_MAX;                             // Set counter > ICR1 here, to avoid output signal generation.
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
 * @param aDegree - If value is below 180 then assume degree, otherwise assume microseconds
 * @param aUpdateFast - If true, enable starting a new output pulse if more than 5 ms since last one, some servo might react faster in this mode.
 * @param aUsePin9 - If false, then Pin10 is used
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
#if !defined(DISABLE_SERVO_TIMER_AUTO_INITIALIZE)
    // auto initialize
    if ((TCCR1B != (_BV(WGM13) | _BV(WGM12) | _BV(CS11))) || (aUsePin9 && ((TCCR1A & ~_BV(COM1B1)) != (_BV(COM1A1) | _BV(WGM11))))
            || (!aUsePin9 && ((TCCR1A & ~_BV(COM1A1)) != (_BV(COM1B1) | _BV(WGM11))))) {
        initLightweightServoPin9_10(aUsePin9, !aUsePin9);
    }
#endif
    // since the resolution is 1/2 of microsecond for 16 MHz CPU clock and prescaler of 8
#if (F_CPU == 16000000L)
    aMicroseconds *= 2;
#elif (F_CPU < 8000000L) // for 8 MHz resolution is exactly 1 microsecond :-)
    aMicroseconds /= (8000000L / F_CPU);
#endif
    if (aUpdateFast) {
        uint16_t tTimerCount = TCNT1;
        if (tTimerCount > ISR_COUNT_FOR_2_5_MILLIS) {
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
#if (F_CPU == 16000000L)
    aMicroseconds *= 2;
#elif (F_CPU < 8000000L) // for 8 MHz resolution is exactly 1 microsecond :-)
    aMicroseconds /= (8000000L / F_CPU);
#endif
    OCR1A = aMicroseconds;
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
#if (F_CPU == 16000000L)
    aMicroseconds *= 2;
#elif (F_CPU < 8000000L) // for 8 MHz resolution is exactly 1 microsecond :-)
    aMicroseconds /= (8000000L / F_CPU);
#endif
    OCR1B = aMicroseconds;
}
#endif

/*
 * Use 16 bit timer1 for generating 2 servo signals entirely by hardware without any interrupts.
 * Use FastPWM mode and generate pulse at start of the 20 ms period
 * Attention - both pins are set to OUTPUT here!
 * 32 bytes code size
 * Assume, that PRR1 PRTIM5 bit is low, which enables timer 5
 *  Supported pins:
 *  ATmega328: pin 44 = OC5C/PL5, 45 = OC5B/PL4 and 46 = OC5A/PL3 using only timer5 hardware
 *  ATmega2560 pin 9 = OC1A and 10 = OC1B on 328 using only timer1 hardware
 */
void initLightweightServoPins() {
#if defined(__AVR_ATmega2560__)
    /*
     * Periods below 20 ms gives problems with long signals i.e. the positioning is not possible
     */
    DDRL |= _BV(DDL3) | _BV(DDL4) | _BV(DDL5); // Required! Set pins pin 44 = OC5C/PL5, 45 = OC5B/PL4 + 46 = OC5A/PL3 to output direction
    TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51); // FastPWM Mode mode TOP (20 ms) determined by ICR1 - non-inverting Compare Output mode OC1A+OC1B
    TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS51); // set prescaler to 8, FastPWM mode mode bits WGM53 + WGM52 - other available prescaler are 1 and 64 :-(
    ICR5 = ISR_COUNT_FOR_20_MILLIS;                // set period to 20 ms
#else
    DDRB |= _BV(DDB1) | _BV(DDB2);      // Required! Set pins OC1A = PortB1 -> PIN 9 and OC1B = PortB2 -> PIN 10 to output direction
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); // FastPWM Mode mode TOP (20 ms) determined by ICR1 - non-inverting Compare Output mode OC1A+OC1B
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // set prescaler to 8, FastPWM mode mode bits WGM13 + WGM12 - other available prescaler are 1 and 64 :-(
    ICR1 = ISR_COUNT_FOR_20_MILLIS;                 // set period to 20 ms
#endif
    // do not set counter here, since with counter = 0 (default) no output signal is generated.
}

/*
 * Use 16 bit timer for generating 2 servo signals entirely by hardware without any interrupts.
 * Use FastPWM mode and generate pulse at start of the 20 ms period
 * Attention - the selected pin is set to OUTPUT here!
 * 54 bytes code size
 * Set pin to output (required) and enable non-inverting Compare Output mode
 * Set output compare > ICRx, to avoid output signal generation.
 */
void checkAndInitLightweightServoPin(uint8_t aPin) {
    bool tPinWasNotInitialized = false;
    // Mask all other bits to zero!
#if defined(__AVR_ATmega2560__)
    uint8_t tNewTCCR5A = TCCR5A & (_BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51));
    if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_A_PIN && !(TCCR5A & (_BV(COM5A1)))) {
        OCR5A = UINT16_MAX;
        DDRL |= (_BV(DDL3));
        tNewTCCR5A |= (_BV(COM5A1)) | _BV(WGM51); // FastPWM Mode mode TOP (20 ms) determined by ICR
        tPinWasNotInitialized = true;
    } else if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_B_PIN && !(TCCR5A & (_BV(COM5B1)))) {
        OCR5B = UINT16_MAX;
        DDRL |= (_BV(DDL4));
        tNewTCCR5A |= (_BV(COM5B1)) | _BV(WGM51);
        tPinWasNotInitialized = true;
    } else if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_C_PIN && !(TCCR5A & (_BV(COM5C1)))) {
        OCR5C = UINT16_MAX;
        DDRL |= (_BV(DDL5));
        tNewTCCR5A |= (_BV(COM5C1)) | _BV(WGM51);
        tPinWasNotInitialized = true;
    }
#else
    uint8_t tNewTCCR1A = TCCR1A & (_BV(COM1A1) | _BV(COM1B1)); // keep existing COM1A1 and COM1B1 settings
    if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_A_PIN && !(TCCR1A & (_BV(COM1A1)))) {
        OCR1A = UINT16_MAX;
        DDRB |= _BV(DDB1);                          // set OC1A = PortB1 -> PIN 9 to output direction
        tNewTCCR1A |= (_BV(COM1A1)) | _BV(WGM11);       // FastPWM Mode mode TOP (20 ms) determined by ICR1
        tPinWasNotInitialized = true;
    } else if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_B_PIN && !(TCCR1A & (_BV(COM1B1)))) {
        OCR1B = UINT16_MAX;
        DDRB |= _BV(DDB2);                          // set OC1B = PortB2 -> PIN 10 to output direction
        tNewTCCR1A |= (_BV(COM1B1)) | _BV(WGM11);
        tPinWasNotInitialized = true;
    }
#endif

    if (tPinWasNotInitialized) {
#if defined(LOCAL_DEBUG)
        Serial.print(F("Auto initialize pin "));
        Serial.print(aPin);
        Serial.print(F(" TCCR5A=0x"));
        Serial.println(tNewTCCR5A,HEX);
#endif
        /*
         * Initialize timer with constant values
         */
#if defined(__AVR_ATmega2560__)
        TCCR5A = tNewTCCR5A;
        TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS51); // set prescaler to 8, FastPWM mode mode bits WGM53 + WGM52 - other available prescaler are 1 and 64 :-(
        /*
         * Periods below 20 ms gives problems with long signals i.e. the positioning is not possible
         */
        ICR5 = ISR_COUNT_FOR_20_MILLIS;                // set period to 20 ms
        // do not set counter here, since with counter = 0 (default) no output signal is generated.
#else
        TCCR1A = tNewTCCR1A;
        TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);   // set prescaler to 8, FastPWM Mode mode bits WGM13 + WGM12
        ICR1 = ISR_COUNT_FOR_20_MILLIS;                // set period to 20 ms
#endif
    }
}

/*
 * Set pin to input and disable non-inverting Compare Output mode
 * Init state is reflected by COMXX1 register bit
 */
void deinitLightweightServoPin(uint8_t aPin) {
#if defined(__AVR_ATmega2560__)
    if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_A_PIN) {
        DDRL &= ~(_BV(DDL3));
        TCCR5A &= ~(_BV(COM5A1));
    } else if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_B_PIN) {
        DDRL &= ~(_BV(DDL4));
        TCCR5A &= ~(_BV(COM5B1));
    } else {
        DDRL &= ~(_BV(DDL5));
        TCCR5A &= ~(_BV(COM5C1));
    }
#else
    if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_A_PIN) {
        DDRB &= ~(_BV(DDB1));           // set OC1A = PortB1 -> PIN 9 to input direction
        TCCR1A &= ~(_BV(COM1A1));       // disable non-inverting Compare Output mode OC1A
    } else if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_B_PIN) {
        DDRB &= ~(_BV(DDB2));           // set OC1B = PortB2 -> PIN 10 to input direction
        TCCR1A &= ~(_BV(COM1B1));       // disable non-inverting Compare Output mode OC1B
    }
#endif
}

/*
 * @param aDegree - If value is below 180 then assume degree, otherwise assume microseconds
 * @param aUpdateFast - If true, enable starting a new output pulse if more than 5 ms since last one, some servo might react faster in this mode.
 * @param aUsePin9 - If false, then Pin10 is used
 * 236 / 186(without auto init) bytes code size
 */
int writeLightweightServoPin(int aDegree, uint8_t aPin, bool aUpdateFast) {
    if (aDegree <= 180) {
        aDegree = DegreeToMicrosecondsLightweightServo(aDegree);
    }
    writeMicrosecondsLightweightServoPin(aDegree, aPin, aUpdateFast);
    return aDegree;
}

void writeMicrosecondsLightweightServoPin(int aMicroseconds, uint8_t aPin, bool aUpdateFast, bool aDoAutoInit) {
#if defined(LOCAL_DEBUG)
    Serial.print(F(" Micros=")); // trailing space required if called by _writeMicrosecondsOrUnits()
    Serial.print(aMicroseconds);
    Serial.print(F(" pin="));
    Serial.println(aPin);
#endif
#if !defined(DISABLE_SERVO_TIMER_AUTO_INITIALIZE)
    if (aDoAutoInit) {
        checkAndInitLightweightServoPin(aPin);
    }
#endif
// since the resolution is 1/2 of microsecond for 16 MHz CPU clock and prescaler of 8
#if (F_CPU == 16000000L)
    aMicroseconds *= 2;
#elif (F_CPU < 8000000L) // for 8 MHz resolution is exactly 1 microsecond :-)
aMicroseconds /= (8000000L / F_CPU);
#endif
#if defined(__AVR_ATmega2560__)
if (aUpdateFast) {
    uint16_t tTimerCount = TCNT5;
    if (tTimerCount > ISR_COUNT_FOR_2_5_MILLIS) {
        // more than 2.5 ms since last pulse -> start a new one
        TCNT5 = ICR5 - 1;
    }
}
if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_A_PIN) {
    OCR5A = aMicroseconds;
} else if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_B_PIN) {
    OCR5B = aMicroseconds;
} else {
    OCR5C = aMicroseconds;
}
#else
    if (aUpdateFast) {
        uint16_t tTimerCount = TCNT1;
        if (tTimerCount > ISR_COUNT_FOR_2_5_MILLIS) {
            // more than 2.5 ms since last pulse -> start a new one
            TCNT1 = ICR1 - 1;
        }
    }
    if (aPin == LIGHTWEIGHT_SERVO_CHANNEL_A_PIN) {
        OCR1A = aMicroseconds;
    } else {
        OCR1B = aMicroseconds;
    }
#endif
}

/*
 * Sets the period of the servo pulses. Reasonable values are 2500 to 20000 microseconds.
 * No parameter checking is done here!
 */
void setLightweightServoRefreshRate(unsigned int aRefreshPeriodMicroseconds) {
#if defined(__AVR_ATmega2560__)
    ICR5 = aRefreshPeriodMicroseconds * 2;
#else
    ICR1 = aRefreshPeriodMicroseconds * 2;
#endif
}
/*
 * Set the mapping pulse width values for 0 and 180 degree
 */
void setLightweightServoPulseMicrosFor0And180Degree(int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree) {
    sMicrosecondsForServo0Degree = aMicrosecondsForServo0Degree;
    sMicrosecondsForServo180Degree = aMicrosecondsForServo180Degree;
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

/*
 * LightweightServo class functions
 */
uint8_t LightweightServo::attach(int aPin) {
    LightweightServoPin = aPin;
    checkAndInitLightweightServoPin(aPin);
    return aPin;
}

/*
 * do not use parameters aMicrosecondsForServo0Degree and aMicrosecondsForServo180Degree
 */
uint8_t LightweightServo::attach(int aPin, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree) {
    LightweightServoPin = aPin;
    MicrosecondsForServo0Degree = aMicrosecondsForServo0Degree;
    MicrosecondsForServo180Degree = aMicrosecondsForServo180Degree;
    checkAndInitLightweightServoPin(aPin);
    return aPin;
}

void LightweightServo::detach() {
    deinitLightweightServoPin(LightweightServoPin);
}

void LightweightServo::write(int aTargetDegreeOrMicrosecond) {
    if (aTargetDegreeOrMicrosecond <= 180) {
        aTargetDegreeOrMicrosecond = (map(aTargetDegreeOrMicrosecond, 0, 180, MicrosecondsForServo0Degree,
                MicrosecondsForServo180Degree));
    }
    // The last false parameter requires 8 byte more than DISABLE_SERVO_TIMER_AUTO_INITIALIZE, but saves around 60 bytes anyway
    writeMicrosecondsLightweightServoPin(aTargetDegreeOrMicrosecond, LightweightServoPin, false, false);
}

void LightweightServo::writeMicroseconds(int aTargetMicrosecond){
    // The last false parameter requires 8 byte more than DISABLE_SERVO_TIMER_AUTO_INITIALIZE, but saves around 60 bytes anyway
    writeMicrosecondsLightweightServoPin(aTargetMicrosecond, LightweightServoPin, false, false);
}

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _LIGHTWEIGHT_SERVO_HPP
#endif // defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega2560__)
