/*
 *  HCSR04.hpp
 *
 *  US Sensor (HC-SR04) functions.
 *  The non blocking functions are using pin change interrupts and need the PinChangeInterrupt library to be installed.
 *
 *  58,23 us per centimeter and 17,17 cm/ms (forth and back).
 *
 *  Supports 1 Pin mode as you get on the HY-SRF05 if you connect OUT to ground.
 *  You can modify the HC-SR04 modules to 1 Pin mode by:
 *  Old module with 3 16 pin chips: Connect Trigger and Echo direct or use a resistor < 4.7 kOhm.
 *        If you remove both 10 kOhm pullup resistors you can use a connecting resistor < 47 kOhm, but I suggest to use 10 kOhm which is more reliable.
 *  Old module with 3 16 pin chips but with no pullup resistors near the connector row: Connect Trigger and Echo with a resistor > 200 ohm. Use 10 kOhm.
 *  New module with 1 16 pin and 2 8 pin chips: Connect Trigger and Echo by a resistor > 200 ohm and < 22 kOhm.
 *  All modules: Connect Trigger and Echo by a resistor of 4.7 kOhm.
 *  Some old HY-SRF05 modules of mine cannot be converted by adding a 4.7 kOhm resistor,
 *  since the output signal going low triggers the next measurement. But they work with removing the 10 kOhm pull up resistors and adding 10 kOhm.
 *
 * Sensitivity is increased by removing C3 / the low pass part of the 22 kHz Bandpass filter.
 * After this the crosstalking of the output signal will be detected as a low distance. We can avoid this by changing R7 to 0 ohm.
 *
 *  Module Type                   |   Characteristics     |         3 Pin Mode          | Increase sensitivity
 *  ------------------------------------------------------------------------------------------------------------
 *  3 * 14 pin IC's 2 transistors | C2 below right IC/U2  | 10 kOhm pin 1+2 middle IC   | not needed, because of Max232
 *                                | right IC is Max232    |                             |
 *  3 * 14 pin IC's 2 transistors | Transistor between    |                             | -C2, R11=1.5MOhm, R12=0
 *                                | middle and right IC   |                             |
 *  3 * 14 pin IC's               | R17 below right IC    | 10 kOhm pin 1+2 middle IC   |
 *  1*4 2*8 pin IC's              |                       | 10 kOhm pin 3+4 right IC    | -C4, R7=1.5MOhm, R10=0
 *  HY-SRF05 3 * 14 pin IC's      |                       | 10 kOhm pin 1+2 middle IC   | - bottom left C, R16=1.5MOhm, R15=?
 *
 *  The CS100A module is not very sensitive at short or mid range but can detect up to 3m. Smallest distance is 2 cm.
 *  The amplified analog signal is available at pin 5 and the comparator output at pin 6. There you can see other echoes.
 *  3 Pin mode is difficult since it retriggers itself at distances below 7 cm.
 *
 *  Copyright (C) 2018-2020  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Utils https://github.com/ArminJo/Arduino-Utils.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
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

#ifndef _HCSR04_HPP
#define _HCSR04_HPP

#include <Arduino.h>

/*
 * The NON BLOCKING version only blocks for around 12 microseconds for code + generation of trigger pulse
 * Be sure to have the right interrupt vector below.
 * check with: while (!isUSDistanceMeasureFinished()) {<do something> };
 * Result is in sUSDistanceCentimeter;
 * 150 bytes program space for interrupt handler, mainly because of requiring the micros() function.
 */

// Activate the line according to the echo in pin number if using the non blocking version
//#define USE_PIN_CHANGE_INTERRUPT_D0_TO_D7  // using PCINT2_vect - PORT D
//#define USE_PIN_CHANGE_INTERRUPT_D8_TO_D13 // using PCINT0_vect - PORT B - Pin 13 is feedback output
//#define USE_PIN_CHANGE_INTERRUPT_A0_TO_A5  // using PCINT1_vect - PORT C
#if __has_include("digitalWriteFast.h")
#include "digitalWriteFast.h"
#else
#define pinModeFast             pinMode
#define digitalReadFast         digitalRead
#define digitalWriteFast        digitalWrite
#define digitalToggleFast(P)    digitalWrite(P, ! digitalRead(P))
#endif

#include "HCSR04.h"

//#define DEBUG
#if !defined(MICROS_IN_ONE_MILLI)
#define MICROS_IN_ONE_MILLI 1000L
#endif

#if defined (TRIGGER_OUT_PIN)
#define sTriggerOutPin TRIGGER_OUT_PIN
#else
uint8_t sTriggerOutPin; // also used as aTriggerOutEchoInPin for 1 pin mode
#endif
#if defined (ECHO_IN_PIN)
#define sEchoInPin ECHO_IN_PIN
#else
uint8_t sEchoInPin;
#endif

uint8_t sHCSR04Mode = HCSR04_MODE_UNITITIALIZED;

unsigned long sLastUSDistanceMeasurementMillis; // Only written by getUSDistanceAsCentimeterWithCentimeterTimeoutPeriodicallyAndPrintIfChanged()
unsigned int sLastUSDistanceCentimeter; // Only written by getUSDistanceAsCentimeterWithCentimeterTimeoutPeriodicallyAndPrintIfChanged()
unsigned int sUSDistanceMicroseconds;
unsigned int sUSDistanceCentimeter;
uint8_t sUsedMillisForMeasurement; // is optimized out if not used

/*
 * @param aEchoInPin - If aEchoInPin == 0 then assume 1 pin mode
 */
void initUSDistancePins(uint8_t aTriggerOutPin, uint8_t aEchoInPin) {
#if !defined (TRIGGER_OUT_PIN)
    sTriggerOutPin = aTriggerOutPin;
#endif
    if (aEchoInPin == 0) {
        sHCSR04Mode = HCSR04_MODE_USE_1_PIN;
    } else {
#if !defined (ECHO_IN_PIN)
        sEchoInPin = aEchoInPin;
#endif
        pinModeFast(aTriggerOutPin, OUTPUT);
        pinModeFast(sEchoInPin, INPUT);
        sHCSR04Mode = HCSR04_MODE_USE_2_PINS;
    }
}

void setHCSR04OnePinMode(bool aUseOnePinMode) {
    if (aUseOnePinMode) {
        sHCSR04Mode = HCSR04_MODE_USE_1_PIN;
    } else {
        sHCSR04Mode = HCSR04_MODE_USE_2_PINS;

    }
}

/*
 * Using this determines one pin mode
 */
void initUSDistancePin(uint8_t aTriggerOutEchoInPin) {
#if !defined (TRIGGER_OUT_PIN)
    sTriggerOutPin = aTriggerOutEchoInPin;
#endif
    (void) aTriggerOutEchoInPin;
    sHCSR04Mode = HCSR04_MODE_USE_1_PIN;
}

/*
 * Start of standard blocking implementation using pulseInLong() since PulseIn gives wrong (too small) results :-(
 * @param aTimeoutMicros timeout of 5825 micros is equivalent to 1 meter, default timeout of 20000 micros is 3.43 meter
 * @return 0 / DISTANCE_TIMEOUT_RESULT if uninitialized or timeout happened
 */
unsigned int getUSDistance(unsigned int aTimeoutMicros) {
    if (sHCSR04Mode == HCSR04_MODE_UNITITIALIZED) {
        return DISTANCE_TIMEOUT_RESULT;
    }

// need minimum 10 usec Trigger Pulse
    digitalWriteFast(sTriggerOutPin, HIGH);

    if (sHCSR04Mode == HCSR04_MODE_USE_1_PIN) {
        // do it AFTER digitalWrite to avoid spurious triggering by just switching pin to output
        pinModeFast(sTriggerOutPin, OUTPUT);
    }

#if defined(DEBUG)
    delayMicroseconds(100); // to see it on scope
#else
    delayMicroseconds(10);
#endif
// falling edge starts measurement after 400/600 microseconds (old/new modules)
    digitalWriteFast(sTriggerOutPin, LOW);

    uint8_t tEchoInPin;
    if (sHCSR04Mode == HCSR04_MODE_USE_1_PIN) {
        // allow for 20 us low (20 us instead of 10 us also supports the JSN-SR04T) before switching to input which is high because of the modules pullup resistor.
        delayMicroseconds(20);
        pinModeFast(sTriggerOutPin, INPUT);
        tEchoInPin = sTriggerOutPin;
    } else {
        tEchoInPin = sEchoInPin;
    }

    /*
     * Get echo length.
     * Speed of sound is: 331.5 + (0.6 * TemperatureCelsius).
     * Exact value at 20 degree celsius is 343,46 m/s => 58,23 us per centimeter and 17,17 cm/ms (forth and back)
     * Exact value at 10 degree celsius is 337,54 m/s => 59,25 us per centimeter and 16,877 cm/ms (forth and back)
     * At 20 degree celsius => 50cm gives 2914 us, 2m gives 11655 us
     *
     * Use pulseInLong, this uses micros() as counter, relying on interrupts being enabled, which is not disturbed by (e.g. the 1 ms timer) interrupts.
     * Only thing is, that if the pulse ends when we are in an interrupt routine, the measured pulse duration is prolonged.
     * I measured 6 us for the millis() and 14 to 20 us for the Servo signal generating interrupt. This is equivalent to around 1 to 3 mm distance.
     * Alternatively we can use pulseIn() in a noInterrupts() context, but this will effectively stop the millis() timer for duration of pulse / or timeout.
     */
#if ! defined(__AVR__) || defined(TEENSYDUINO) || defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
    noInterrupts();
    sUSDistanceMicroseconds = pulseIn(tEchoInPin, HIGH, aTimeoutMicros);
    interrupts();
#else
    sUSDistanceMicroseconds = pulseInLong(tEchoInPin, HIGH, aTimeoutMicros); // returns 0 (DISTANCE_TIMEOUT_RESULT) for timeout
#endif
    // Division takes 48 us and adds 50 bytes program space. Statement is optimized out if sUsedMillisForMeasurement is not used
    sUsedMillisForMeasurement = (sUSDistanceMicroseconds + 550) / MICROS_IN_ONE_MILLI;
    return sUSDistanceMicroseconds;
}

/*
 * No return of 0 at
 */
unsigned int getCentimeterFromUSMicroSeconds(unsigned int aDistanceMicros) {
    // The reciprocal of formula in getUSDistanceAsCentimeterWithCentimeterTimeout()
    return (aDistanceMicros * 100L) / 5825;
}

uint8_t getMillisFromUSCentimeter(unsigned int aDistanceCentimeter) {
    // Use formula from getUSDistanceAsCentimeterWithCentimeterTimeout()
    return ((aDistanceCentimeter * 233L) + 2000) / 4000; // = * 58.25 (rounded by using +1)
}

/**
 * @param aTimeoutMicros timeout of 5825 micros is equivalent to 1 meter, 10000 is 1.71 m, default timeout of 20000 micro seconds is 3.43 meter
 * @return  Distance in centimeter @20 degree celsius (time in us/58.25)
 *          0 / DISTANCE_TIMEOUT_RESULT if timeout or pins are not initialized
 */
unsigned int getUSDistanceAsCentimeter(unsigned int aTimeoutMicros) {
    sUSDistanceCentimeter = getCentimeterFromUSMicroSeconds(getUSDistance(aTimeoutMicros));
    return sUSDistanceCentimeter;
}

// 58,23 us per centimeter (forth and back)
unsigned int getUSDistanceAsCentimeterWithCentimeterTimeout(unsigned int aTimeoutCentimeter) {
// The reciprocal of formula in getCentimeterFromUSMicroSeconds()
    unsigned int tTimeoutMicros = ((aTimeoutCentimeter * 233L) + 2) / 4; // = * 58.25 (rounded by using +1)
    return getUSDistanceAsCentimeter(tTimeoutMicros);
}

/**
 * @return  true, if US distance has changed
 */
bool getUSDistanceAsCentimeterWithCentimeterTimeoutPeriodicallyAndPrintIfChanged(unsigned int aTimeoutCentimeter,
        unsigned int aMillisBetweenMeasurements, Print *aSerial) {
    if ((millis() - sLastUSDistanceMeasurementMillis) >= aMillisBetweenMeasurements) {
        getUSDistanceAsCentimeterWithCentimeterTimeout(aTimeoutCentimeter);
        if (sLastUSDistanceCentimeter != sUSDistanceCentimeter) {
            sLastUSDistanceCentimeter = sUSDistanceCentimeter;
            if (sUSDistanceCentimeter == 0) {
                aSerial->println(F("Timeout"));
            } else {
                aSerial->print(F("Distance="));
                aSerial->print(sUSDistanceCentimeter);
                aSerial->println(F("cm"));
            }
            return true;
        }
    }
    return false;
}

/*
 * Trigger US sensor as fast as sensible if called in a loop to test US devices.
 * trigger pulse is equivalent to 10 cm and then we wait for 20 ms / 3.43 meter
 */
void testUSSensor(uint16_t aSecondsToTest) {
    for (long i = 0; i < aSecondsToTest * 50; ++i) {
        digitalWriteFast(sTriggerOutPin, HIGH);
        delayMicroseconds(582); // pulse is as long as echo for 10 cm
        // falling edge starts measurement
        digitalWriteFast(sTriggerOutPin, LOW);
        delay(20);        // wait time for 3,43 meter to let the US pulse vanish
    }
}

#if (defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7) | defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13) | defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5))

volatile unsigned long sUSPulseMicros;
volatile bool sUSValueIsValid = false;
volatile unsigned long sMicrosAtStartOfPulse;
unsigned int sTimeoutMicros;

/*
 * common code for all interrupt handler.
 */
void handlePCInterrupt(uint8_t aPortState) {
    if (aPortState > 0) {
        // start of pulse
        sMicrosAtStartOfPulse = micros();
    } else {
        // end of pulse
        sUSPulseMicros = micros() - sMicrosAtStartOfPulse;
        sUSValueIsValid = true;
    }
#if defined(DEBUG)
// for debugging purposes, echo to PIN 13 (do not forget to set it to OUTPUT!)
// digitalWrite(13, aPortState);
#endif
}
#endif // USE_PIN_CHANGE_INTERRUPT_D0_TO_D7 ...

#if defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7)
/*
 * pin change interrupt for D0 to D7 here.
 */
ISR (PCINT2_vect) {
// read pin
//    uint8_t tPortState = digitalReadFast(sEchoInPin);//(*portInputRegister(digitalPinToPort(sEchoInPin))) & bit((digitalPinToPCMSKbit(sEchoInPin)));
    handlePCInterrupt(digitalReadFast(sEchoInPin));
}
#endif

#if defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13)
/*
 * pin change interrupt for D8 to D13 here.
 * state of pin is echoed to output 13 for debugging purpose
 */
ISR (PCINT0_vect) {
// check pin
    uint8_t tPortState = (*portInputRegister(digitalPinToPort(sEchoInPin))) & bit((digitalPinToPCMSKbit(sEchoInPin)));
    handlePCInterrupt(tPortState);
}
#endif

#if defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5)
/*
 * pin change interrupt for A0 to A5 here.
 * state of pin is echoed to output 13 for debugging purpose
 */
ISR (PCINT1_vect) {
// check pin
    uint8_t tPortState = (*portInputRegister(digitalPinToPort(sEchoInPin))) & bit((digitalPinToPCMSKbit(sEchoInPin)));
    handlePCInterrupt(tPortState);
}
#endif

#if (defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7) | defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13) | defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5))

void startUSDistanceAsCentimeterWithCentimeterTimeoutNonBlocking(unsigned int aTimeoutCentimeter) {
// need minimum 10 usec Trigger Pulse
    digitalWrite(sTriggerOutPin, HIGH);
    sUSValueIsValid = false;
    sTimeoutMicros = ((aTimeoutCentimeter * 233) + 2) / 4; // = * 58.25 (rounded by using +1)
    *digitalPinToPCMSK(sEchoInPin) |= bit(digitalPinToPCMSKbit(sEchoInPin));// enable pin for pin change interrupt
// the 2 registers exists only once!
    PCICR |= bit(digitalPinToPCICRbit(sEchoInPin));// enable interrupt for the group
    PCIFR |= bit(digitalPinToPCICRbit(sEchoInPin));// clear any outstanding interrupt
    sUSPulseMicros = 0;
    sMicrosAtStartOfPulse = 0;

#if defined(DEBUG)
    delay(2); // to see it on scope
#else
    delayMicroseconds(10);
#endif
// falling edge starts measurement and generates first interrupt
    digitalWrite(sTriggerOutPin, LOW);
}

/*
 * Used to check by polling.
 * If ISR interrupts these code, everything is fine, even if we get a timeout and a no null result
 * since we are interested in the result and not in very exact interpreting of the timeout.
 */
bool isUSDistanceMeasureFinished() {
    if (sUSValueIsValid) {
        sUSDistanceCentimeter = getCentimeterFromUSMicroSeconds(sUSPulseMicros);
        return true;
    }

    if (sMicrosAtStartOfPulse != 0) {
        if ((micros() - sMicrosAtStartOfPulse) >= sTimeoutMicros) {
            // Timeout happened, value will be 0
            *digitalPinToPCMSK(sEchoInPin) &= ~(bit(digitalPinToPCMSKbit(sEchoInPin)));// disable pin for pin change interrupt
            return true;
        }
    }
    return false;
}
#endif // USE_PIN_CHANGE_INTERRUPT_D0_TO_D7 ...
#endif //  _HCSR04_HPP
