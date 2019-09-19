/*
 *  HCSR04.cpp
 *
 *  US Sensor (HC-SR04) functions especially non blocking functions using pin change interrupts
 *
 *  Copyright (C) 2016  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>
#include "HCSR04.h"

uint8_t sTriggerOutPin;
uint8_t sEchoInPin;
bool isInitialized = false;

void initUSDistancePins(uint8_t aTriggerOutPin, uint8_t aEchoInPin) {
    sTriggerOutPin = aTriggerOutPin;
    sEchoInPin = aEchoInPin;
    pinMode(aTriggerOutPin, OUTPUT);
    pinMode(sEchoInPin, INPUT);
    isInitialized = true;
}

/*
 * Start of standard blocking implementation using pulseInLong() since PulseIn gives wrong (too small) results :-(
 */
unsigned int getUSDistance(unsigned int aTimeoutMicros) {
    if (!isInitialized) {
        return 0;
    }

// need minimum 10 usec Trigger Pulse
    digitalWrite(sTriggerOutPin, HIGH);
#ifdef DEBUG
    delay(2); // to see it on scope
#else
    delayMicroseconds(10);
#endif
// falling edge starts measurement
    digitalWrite(sTriggerOutPin, LOW);

    /*
     * Get echo length. 58,48 us per centimeter (forth and back)
     * => 50cm gives 2900 us, 2m gives 11900 us
     */
    unsigned long tUSPulseMicros = pulseInLong(sEchoInPin, HIGH, aTimeoutMicros);
    if (tUSPulseMicros == 0) {
// timeout happened
        tUSPulseMicros = aTimeoutMicros;
    }
    return tUSPulseMicros;
}

unsigned int getCentimeterFromUSMicroSeconds(unsigned int aDistanceMicros) {
    // The reciprocal of formula in getUSDistanceAsCentiMeterWithCentimeterTimeout()
    return (aDistanceMicros * 10L) / 585;
}

/*
 * @return  Distance in centimeter (time in us/58.5)
 *          aTimeoutMicros/58.5 if timeout happens
 *          0 if pins are not initialized
 *          timeout of 5850 micros is equivalent to 1m
 */
unsigned int getUSDistanceAsCentiMeter(unsigned int aTimeoutMicros) {
    unsigned int tDistanceMicros = getUSDistance(aTimeoutMicros);
    if (tDistanceMicros == 0) {
// timeout happened
        tDistanceMicros = aTimeoutMicros;
    }
    return (getCentimeterFromUSMicroSeconds(tDistanceMicros));
}

// 58,48 us per centimeter (forth and back)
unsigned int getUSDistanceAsCentiMeterWithCentimeterTimeout(unsigned int aTimeoutCentimeter) {
// The reciprocal of formula in getCentimeterFromUSMicroSeconds()
    unsigned int tTimeoutMicros = ((aTimeoutCentimeter * 117) + 1) / 2; // = * 58.5 (rounded by using +1)
    return getUSDistanceAsCentiMeter(tTimeoutMicros);
}

/*
 * The NON BLOCKING version only blocks for ca. 12 microseconds for code + generation of trigger pulse
 * Be sure to have the right interrupt vector below.
 * check with: while (!isUSDistanceMeasureFinished()) {<do something> };
 * Result is in sUSDistanceCentimeter;
 */

// Comment out the line according to the sEchoInPin if using the non blocking version
// or define it as symbol for the compiler e.g. -DUSE_PIN_CHANGE_INTERRUPT_D0_TO_D7
//#define USE_PIN_CHANGE_INTERRUPT_D0_TO_D7  // using PCINT2_vect - PORT D
//#define USE_PIN_CHANGE_INTERRUPT_D8_TO_D13 // using PCINT0_vect - PORT B - Pin 13 is feedback output
//#define USE_PIN_CHANGE_INTERRUPT_A0_TO_A5  // using PCINT1_vect - PORT C
#if (defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7) | defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13) | defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5))

unsigned int sUSDistanceCentimeter;
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
#ifdef DEBUG
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
    uint8_t tPortState = (*portInputRegister(digitalPinToPort(sEchoInPin))) & bit((digitalPinToPCMSKbit(sEchoInPin)));
    handlePCInterrupt(tPortState);
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

void startUSDistanceAsCentiMeterWithCentimeterTimeoutNonBlocking(unsigned int aTimeoutCentimeter) {
// need minimum 10 usec Trigger Pulse
    digitalWrite(sTriggerOutPin, HIGH);
    sUSValueIsValid = false;
    sTimeoutMicros = aTimeoutCentimeter * 59;
    *digitalPinToPCMSK(sEchoInPin) |= bit(digitalPinToPCMSKbit(sEchoInPin));// enable pin for pin change interrupt
// the 2 registers exists only once!
    PCICR |= bit(digitalPinToPCICRbit(sEchoInPin));// enable interrupt for the group
    PCIFR |= bit(digitalPinToPCICRbit(sEchoInPin));// clear any outstanding interrupt
    sUSPulseMicros = 0;
    sMicrosAtStartOfPulse = 0;

#ifdef DEBUG
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
        int tDistance = ((sUSPulseMicros * 10) / 585);
        sUSDistanceCentimeter = tDistance;
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
