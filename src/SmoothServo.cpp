/*
 * SmoothServo.cpp
 *
 *  Enables smooth movement from one servo position to another.
 *  Interface is in degree but internally microseconds are used, since the resolution is better and we avoid the map function on every Servo.write()
 *  The blocking functions wait for 20 ms since this is the default refresh time of the used Servo library.
 *
 *  The "synchronized" functions allow for 2 servos (e.g. on a gimbal) to move synchronized, which means both movements will take the same time.
 *  The result of synchronizing is a linear movement of the combined servos.
 *
 *  If you want to move more servos synchronously you simply call startMoveTo() for every servo
 *  and overwrite each millisForCompleteMove with the biggest value found in all servos.
 *  Then call update() for all servos every 20 milliseconds.
 *  Or if you use interrupts, then just write your own handleTimer_COMPB_Interrupt() function.
 *
 *  Until now only one timer is supported, which means not more than 12 servos are supported.
 *
 *  Copyright (C) 2019  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of SmoothServo.
 *  SmoothServo is free software: you can redistribute it and/or modify
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

#include <Arduino.h>

#include "SmoothServo.h"

// enable this to see information on each call
//#define TRACE
//#define DEBUG

// Enable this if you want to measure timing by toggling pin12 on an arduino
//#define MEASURE_TIMING
#ifdef MEASURE_TIMING
#include "digitalWriteFast.h"
#define TIMING_PIN 12
#endif

static SmoothServo * sPointerToServoForISR;

SmoothServo::SmoothServo() { // @suppress("Class members should be properly initialized")
    MicrosecondsForServo0Degree = DEFAULT_MICROSECONDS_FOR_0_DEGREE;
    MicrosecondsForServo180Degree = DEFAULT_MICROSECONDS_FOR_180_DEGREE;
    sPointerToServoForISR = this;
    Servo();
#ifdef MEASURE_TIMING
    pinMode(TIMING_PIN, OUTPUT);
#endif
}

uint8_t SmoothServo::attach(int aPin) {
    return Servo::attach(aPin);
}

uint8_t SmoothServo::attach(int aPin, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree) {
    MicrosecondsForServo0Degree = aMicrosecondsForServo0Degree;
    MicrosecondsForServo180Degree = aMicrosecondsForServo180Degree;
    return Servo::attach(aPin, MicrosecondsForServo0Degree, aMicrosecondsForServo180Degree);
}

void SmoothServo::setSyncronizedServo(SmoothServo * aSyncronizedServo) {
    SyncronizedServo = aSyncronizedServo;
    // OK this is the master servo
    sPointerToServoForISR = this;
}

void SmoothServo::write(int aValue) {
    Servo::write(aValue);
    currentMicroseconds = readMicroseconds();
#ifdef TRACE
    Serial.print(F("write:"));
    Serial.println(aValue);
#endif
}

void SmoothServo::writeMicroseconds(int aValue) {
    currentMicroseconds = aValue;
    Servo::writeMicroseconds(aValue);
#ifdef TRACE
    Serial.print(F("us="));
    Serial.println(aValue);
#endif
}

uint8_t SmoothServo::MicrosecondsToDegree(uint16_t aMicroseconds) {
    return map(aMicroseconds, MicrosecondsForServo0Degree, MicrosecondsForServo180Degree, 0, 180);
}

uint16_t SmoothServo::DegreeToMicroseconds(uint8_t aDegree) {
    return map(aDegree, 0, 180, MicrosecondsForServo0Degree, MicrosecondsForServo180Degree);
}

/*
 * Blocking move
 */
void SmoothServo::moveTo(uint8_t aDegree, int aDegreesPerSecond) {
    startMoveTo(aDegree, aDegreesPerSecond);
    do {
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!update());

}

/*
 * Blocking move
 */
void SmoothServo::moveToSyncronized(uint8_t aDegree, int aSyncronizedServoDegree, int aDegreesPerSecond) {
    startMoveToSyncronized(aDegree, aSyncronizedServoDegree, aDegreesPerSecond);
    do {
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!updateSyncronized());
}

/*
 * sets up all the values needed for a smooth move to new value
 * returns false if servo was still moving
 */
bool SmoothServo::startMoveTo(uint8_t aDegree, int aDegreesPerSecond, bool doUpdateByInterrupt) {

    endMicroseconds = DegreeToMicroseconds(aDegree);

    startMicroseconds = currentMicroseconds;
    uint8_t tCurrentAngle = MicrosecondsToDegree(currentMicroseconds);

    deltaMicroseconds = endMicroseconds - currentMicroseconds;
    millisForCompleteMove = abs(aDegree - tCurrentAngle) * 1000L / aDegreesPerSecond;

    millisAtStartMove = millis();
    if (doUpdateByInterrupt) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        TIFR5 |= _BV(OCF5B);     // clear any pending interrupts;
        TIMSK5 |= _BV(OCIE5B);// enable the output compare B interrupt
        OCR5B = ((clockCyclesPerMicrosecond() * REFRESH_INTERVAL) / 8) - 100;// update values 100us before the new servo period starts
#else
        TIFR1 |= _BV(OCF1B);     // clear any pending interrupts;
        TIMSK1 |= _BV(OCIE1B); // enable the output compare B interrupt
        OCR1B = ((clockCyclesPerMicrosecond() * REFRESH_INTERVAL) / 8) - 100; // update values 100us before the new servo period starts
#endif
    }

#ifdef DEBUG
    Serial.print(F("Current="));
    Serial.print(tCurrentAngle);
    Serial.print(F(" New="));
    Serial.print(aDegree);
    Serial.print(F(" Delta="));
    int16_t tDelta;
    if (deltaMicroseconds >= 0) {
        tDelta = MicrosecondsToDegree(deltaMicroseconds + MicrosecondsForServo0Degree);
    } else {
        tDelta = -MicrosecondsToDegree(MicrosecondsForServo0Degree - deltaMicroseconds);
    }
    Serial.print(tDelta);
    Serial.print(F(" Speed="));
    Serial.print(aDegreesPerSecond);
    Serial.print(F(" millisForCompleteMove="));
    Serial.print(millisForCompleteMove);
    Serial.println();
#endif
    bool tReturnValue = !servoMoves;
    servoMoves = true;
    return tReturnValue;
}

/*
 * sets up all the values needed for a smooth move for two servos to new values
 *
 * If you want to move more servos synchronously you simply call startMoveTo() for every servo
 * and overwrite each millisForCompleteMove with the biggest value found in each servo.
 * Then call update() for all servos every 20 milliseconds.
 */
bool SmoothServo::startMoveToSyncronized(uint8_t aDegree, int aSyncronizedServoDegree, int aDegreesPerSecond,
bool doUpdateByInterrupt) {
    if (servoMoves || SyncronizedServo->servoMoves) {
        return false;
    }
    startMoveTo(aDegree, aDegreesPerSecond);
    SyncronizedServo->startMoveTo(aSyncronizedServoDegree, aDegreesPerSecond);
    /*
     * Take the longer duration in order to move both servos synchronously
     */
    if (millisForCompleteMove > SyncronizedServo->millisForCompleteMove) {
        SyncronizedServo->millisForCompleteMove = millisForCompleteMove;
    } else {
        millisForCompleteMove = SyncronizedServo->millisForCompleteMove;
    }
    if (doUpdateByInterrupt) {
        TIFR1 |= _BV(OCF1B);     // clear any pending interrupts;
        TIMSK1 |= _BV(OCIE1B); // enable the output compare B interrupt
        // TODO only once????
        OCR1B = ((clockCyclesPerMicrosecond() * REFRESH_INTERVAL) / 8) - 4;
    }
    return true;
}

void SmoothServo::writeSyncronized(int aValue, int aSyncronizedServoValue) {
    write(aValue);
    SyncronizedServo->write(aSyncronizedServoValue);
}

/*
 * returns true if endAngle was reached
 */
bool SmoothServo::update() {

    uint32_t tMillisSinceStart = millis() - millisAtStartMove;
    if (tMillisSinceStart >= millisForCompleteMove) {
        // end reached -> write end position and return true
        writeMicroseconds(endMicroseconds);
        servoMoves = false;
        return true;
    }
    /*
     * new position is: start position + total delta * (millis_done / millis_total aka "percentage of completion")
     * 40 us to compute
     */
    uint16_t tNewMicroseconds = startMicroseconds + ((deltaMicroseconds * (int32_t) tMillisSinceStart) / millisForCompleteMove);

    /*
     * Write new position only if changed
     */
    if (tNewMicroseconds != currentMicroseconds) {
        writeMicroseconds(tNewMicroseconds);
    }
    return false;
}

/*
 * returns true if endAngle was reached
 */
bool SmoothServo::updateSyncronized() {
    update();
    return SyncronizedServo->update();
}

bool SmoothServo::isMoving() {
    return servoMoves;
}

void SmoothServo::print() {
    Serial.print(F("Current="));
    Serial.print(MicrosecondsToDegree(currentMicroseconds));
    Serial.print('|');
    Serial.print(currentMicroseconds);

    Serial.print(F(" NewEnd="));
    Serial.print(MicrosecondsToDegree(endMicroseconds));
    Serial.print('|');
    Serial.print(endMicroseconds);

    Serial.print(F(" Delta="));
    int16_t tDelta;
    if (deltaMicroseconds >= 0) {
        tDelta = MicrosecondsToDegree(deltaMicroseconds + MicrosecondsForServo0Degree);
    } else {
        tDelta = -MicrosecondsToDegree(MicrosecondsForServo0Degree - deltaMicroseconds);
    }
    Serial.print(tDelta);
    Serial.print('|');
    Serial.print(deltaMicroseconds);

    Serial.print(F(" millisForCompleteMove="));
    Serial.print(millisForCompleteMove);
    Serial.print(" this=0x");
    Serial.print((uint16_t) this, HEX);
    Serial.print(F(" Synchronized=0x"));
    Serial.print((uint16_t) SyncronizedServo, HEX);
    Serial.println();
}

void disableSmoothServoInterrupt() {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TIMSK5 &= ~(_BV(OCIE5B)); // disable the output compare B interrupt
#else
    TIMSK1 &= ~(_BV(OCIE1B)); // disable the output compare B interrupt
#endif
}

/*
 * 60us for single servo
 * 20us for last interrupt
 */
__attribute__((weak)) void handleTimer_COMPB_Interrupt() {
#ifdef MEASURE_TIMING
    digitalWriteFast(TIMING_PIN, HIGH);
#endif
    bool tServosStopped = sPointerToServoForISR->update();
    if (sPointerToServoForISR->SyncronizedServo != NULL) {
        sPointerToServoForISR->SyncronizedServo->update();
        tServosStopped = sPointerToServoForISR->SyncronizedServo->update() && tServosStopped;
    }
    if (tServosStopped) {
        // disable only if both servos stopped. This enables independent movements of 2 (synchronized) servos with this interrupt handler.
        disableSmoothServoInterrupt();
    }
#ifdef MEASURE_TIMING
    digitalWriteFast(TIMING_PIN, LOW);
#endif
}

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
ISR(TIMER5_COMPB_vect) {
    handleTimer_COMPB_Interrupt();
}
#else
ISR(TIMER1_COMPB_vect) {
    handleTimer_COMPB_Interrupt();
}
#endif
