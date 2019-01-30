/*
 * ServoEasing.cpp
 *
 *  Enables smooth movement from one servo position to another.
 *  Linear as well as other ease movements (e.g. cubic) for all servos attached to the Arduino Servo library are provided.
 *  Interface is in degree but internally microseconds are used, since the resolution is better and we avoid the map function on every Servo.write()
 *  The blocking functions wait for 20 ms since this is the default refresh time of the used Servo library.
 *
 *  The "synchronized" functions allow for 2 servos (e.g. on a pan/tilt module) to move synchronized, which means both movements will take the same time.
 *  The result of synchronizing is a linear movement of the combined servos.
 *
 *  If you want to move more servos synchronously you simply call startEaseTo() for every servo
 *  and overwrite each millisForCompleteMove with the biggest value found in all servos.
 *  Then call update() for all servos every 20 milliseconds.
 *  Or if you use interrupts, then just write your own handleTimer_COMPB_Interrupt() function.
 *
 *  Until now only one timer is supported, which means not more than 12 servos are supported.
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
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

#include "ServoEasing.h"

// Enable this to see information on each call
// There should be no library which uses Serial, so enable it only for debugging purposes
// #define TRACE

// Enable this if you want to measure timing by toggling pin12 on an arduino
//#define MEASURE_TIMING
#ifdef MEASURE_TIMING
#include "digitalWriteFast.h"
#define TIMING_PIN 12
#endif

static ServoEasing * sPointerToServoForISR;

ServoEasing::ServoEasing() { // @suppress("Class members should be properly initialized")
    MicrosecondsForServo0Degree = DEFAULT_MICROSECONDS_FOR_0_DEGREE;
    MicrosecondsForServo180Degree = DEFAULT_MICROSECONDS_FOR_180_DEGREE;
    trimMicroseconds = 0;
    EasingType = EASE_LINEAR;
    SimpleUserEaseInFunction = NULL;
    UserEaseInOutFunction = NULL;
    sPointerToServoForISR = this;
#ifndef USE_LEIGHTWEIGHT_SERVO_LIB
    Servo();
#endif
#ifdef MEASURE_TIMING
    pinMode(TIMING_PIN, OUTPUT);
#endif
}

/*
 * Return 0/false if not pin 9 or 10 else return aPin
 * Pin number != 9 results in using pin 10.
 */
uint8_t ServoEasing::attach(int aPin) {
#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
    if(aPin != 9 && aPin != 10) {
        return false;
    }
    ServoPin = aPin;
    return aPin;
#else
    return Servo::attach(aPin);
#endif
}

uint8_t ServoEasing::attach(int aPin, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree) {
    MicrosecondsForServo0Degree = aMicrosecondsForServo0Degree;
    MicrosecondsForServo180Degree = aMicrosecondsForServo180Degree;
#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
    if(aPin != 9 && aPin != 10) {
        return false;
    }
    ServoPin = aPin;
    return aPin;
#else
    return Servo::attach(aPin, aMicrosecondsForServo0Degree, aMicrosecondsForServo180Degree);
#endif
}

void ServoEasing::setTrim(int8_t aTrim) {
    setTrimMicroseconds(DegreeToMicroseconds(aTrim));
}

void ServoEasing::setTrimMicroseconds(int16_t aTrimMicroseconds) {
    trimMicroseconds = aTrimMicroseconds;
}

void ServoEasing::setEasingType(uint8_t aEasingType) {
    EasingType = aEasingType;
}

void ServoEasing::registerSimpleUserEaseInFunction(float (*aSimpleUserEaseInFunction)(float aPercentageOfCompletion)) {
    SimpleUserEaseInFunction = aSimpleUserEaseInFunction;
}

void ServoEasing::registerUserEaseInOutFunction(float (*aUserEaseInOutFunction)(float aPercentageOfCompletion, bool aInFirstHalf)) {
    UserEaseInOutFunction = aUserEaseInOutFunction;
}

void ServoEasing::setSynchronizedServo(ServoEasing * aSynchronizedServo) {
    SynchronizedServo = aSynchronizedServo;
    // OK this is the master servo
    sPointerToServoForISR = this;
}

void ServoEasing::write(int aValue) {
    if (aValue < DEFAULT_MICROSECONDS_FOR_0_DEGREE) { // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
        aValue = DegreeToMicroseconds(aValue);
    }
    writeMicroseconds(aValue);
#ifdef TRACE
    Serial.print(F("write:"));
    Serial.println(aValue);
#endif
}

/*
 * Adds trim value and call LeightweightServo or Servo library for generating the pulse
 */
void ServoEasing::writeMicroseconds(int aValue) {
    aValue += trimMicroseconds;
    currentMicroseconds = aValue;
#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
    writeMicrosecondsLightweightServo(aValue, (ServoPin == 9));
#else
    Servo::writeMicroseconds(aValue); // needs 7 us
#endif
#ifdef TRACE
    // 240 us
    Serial.print(F("us="));
    Serial.println(aValue);
#endif
}

uint8_t ServoEasing::MicrosecondsToDegree(uint16_t aMicroseconds) {
    return map(aMicroseconds, MicrosecondsForServo0Degree, MicrosecondsForServo180Degree, 0, 180);
}

uint16_t ServoEasing::DegreeToMicroseconds(uint8_t aDegree) {
    return map(aDegree, 0, 180, MicrosecondsForServo0Degree, MicrosecondsForServo180Degree);
}

/*
 * Blocking move
 * aDegreesPerSecond can range from 1 to the physically maximum value of 450
 */
void ServoEasing::easeTo(uint8_t aDegree, uint16_t aDegreesPerSecond) {
    startEaseTo(aDegree, aDegreesPerSecond);
    do {
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!update());

}

void ServoEasing::easeToD(uint8_t aDegree, uint16_t aMillisForMove) {
    startEaseToD(aDegree, aMillisForMove);
    do {
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!update());

}

void ServoEasing::easeTo(struct ServoMove aMovement) {
    startEaseTo(aMovement.Degree, aMovement.SpeedOrDuration.DegreesPerSecond, false);
    do {
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!update());
}

/*
 * Blocking move
 */
void ServoEasing::easeToSynchronized(uint8_t aDegree, uint8_t aSynchronizedServoDegree, uint16_t aDegreesPerSecond) {
    startEaseToSynchronized(aDegree, aSynchronizedServoDegree, aDegreesPerSecond);
    do {
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!updateSynchronized());
}

bool ServoEasing::startEaseTo(struct ServoMove aMovement, bool doUpdateByInterrupt) {
    return startEaseTo(aMovement.Degree, aMovement.SpeedOrDuration.DegreesPerSecond, doUpdateByInterrupt);
}

/*
 * sets up all the values needed for a smooth move to new value
 * returns false if servo was still moving
 */
bool ServoEasing::startEaseTo(uint8_t aDegree, uint16_t aDegreesPerSecond, bool doUpdateByInterrupt) {
    uint8_t tCurrentAngle = MicrosecondsToDegree(currentMicroseconds);
    uint16_t tMillisForCompleteMove = abs(aDegree - tCurrentAngle) * 1000L / aDegreesPerSecond;
    return startEaseToD(aDegree, tMillisForCompleteMove, doUpdateByInterrupt);
}

bool ServoEasing::startEaseToD(uint8_t aDegree, uint16_t aMillisForMove, bool doUpdateByInterrupt) {
    millisForCompleteMove = aMillisForMove;
    startMicroseconds = currentMicroseconds;
    endMicroseconds = DegreeToMicroseconds(aDegree);
    deltaMicroseconds = endMicroseconds - currentMicroseconds;
    millisAtStartMove = millis();

    if (doUpdateByInterrupt) {
        enableServoEasingInterrupt();
    }
#ifdef DEBUG
// There should be no library which uses Serial, so enable it only for debugging purposes
    printDynamic();
#endif
    bool tReturnValue = !servoMoves;
    servoMoves = true;
    return tReturnValue;
}

/*
 * sets up all the values needed for a smooth move for two servos to new values
 *
 * If you want to move more servos synchronously you simply call startEaseTo() for every servo
 * and overwrite each millisForCompleteMove with the biggest value found in each servo.
 * Then call update() for all servos every 20 milliseconds.
 */
bool ServoEasing::startEaseToDSynchronized(uint8_t aDegree, uint8_t aSynchronizedServoDegree, uint16_t aMillisForMove,
        bool doUpdateByInterrupt) {
    if (servoMoves || SynchronizedServo->servoMoves) {
        return false;
    }
    startEaseToD(aDegree, aMillisForMove);
    SynchronizedServo->startEaseToD(aSynchronizedServoDegree, aMillisForMove);
    synchronizeServosAndStartInterrupt(doUpdateByInterrupt);
    return true;
}

bool ServoEasing::startEaseToSynchronized(uint8_t aDegree, uint8_t aSynchronizedServoDegree, uint16_t aDegreesPerSecond,
        bool doUpdateByInterrupt) {
    if (servoMoves || SynchronizedServo->servoMoves) {
        return false;
    }
    startEaseTo(aDegree, aDegreesPerSecond);
    SynchronizedServo->startEaseTo(aSynchronizedServoDegree, aDegreesPerSecond);
    synchronizeServosAndStartInterrupt(doUpdateByInterrupt);
    return true;
}

void ServoEasing::synchronizeServosAndStartInterrupt(bool doUpdateByInterrupt) {
    /*
     * Synchronize start time to avoid race conditions at the end of movement
     */
    millisAtStartMove = SynchronizedServo->millisAtStartMove;

    /*
     * Take the longer duration in order to move both servos synchronously
     */
    if (millisForCompleteMove > SynchronizedServo->millisForCompleteMove) {
        SynchronizedServo->millisForCompleteMove = millisForCompleteMove;
    } else {
        millisForCompleteMove = SynchronizedServo->millisForCompleteMove;
    }
    if (doUpdateByInterrupt) {
        enableServoEasingInterrupt();
    }
}

void ServoEasing::writeSynchronized(int aValue, int aSynchronizedServoValue) {
    write(aValue);
    SynchronizedServo->write(aSynchronizedServoValue);
}

/*
 * returns true if endAngle was reached
 */
bool ServoEasing::update() {

    uint32_t tMillisSinceStart = millis() - millisAtStartMove;
    if (tMillisSinceStart >= millisForCompleteMove) {
        // end reached -> write end position and return true
        writeMicroseconds(endMicroseconds);
        servoMoves = false;
        return true;
    }

    uint16_t tNewMicroseconds;
    if (EasingType == EASE_LINEAR) {
        /*
         * Use faster non float arithmetic
         * Linear movement: new position is: start position + total delta * (millis_done / millis_total aka "percentage of completion")
         * 40 us to compute
         */
        tNewMicroseconds = startMicroseconds + ((deltaMicroseconds * (int32_t) tMillisSinceStart) / millisForCompleteMove);
    } else {
        /*
         * Non linear movement -> use floats
         */
        float tPercentageOfCompletion = (float) tMillisSinceStart / millisForCompleteMove; // tPercentageOfCompletion is from 0.0 to 1.0
        float tEaseResult = 0.0; // expected values are from 0.0 to 0.5
        /*
         * For the first half of movement (move IN) we use: start + delta * fn(percent)
         * Since the second half is the mirrored first half we can use: end - delta * fn(1-percent)
         */
        bool inFirstHalf = tMillisSinceStart < (millisForCompleteMove / 2);
        if (!inFirstHalf) {
            tPercentageOfCompletion = 1 - tPercentageOfCompletion;
        }

        switch (EasingType) {
        case EASE_USER:
            if (SimpleUserEaseInFunction != NULL) {
                tEaseResult = SimpleUserEaseInFunction(tPercentageOfCompletion);
            } else if (UserEaseInOutFunction != NULL) {
                tEaseResult = UserEaseInOutFunction(tPercentageOfCompletion, inFirstHalf);
            }
            break;
        case EASE_QUADRATIC:
            /*
             * Use only the function for the first half of movement and mirror the result for the second half of movement
             * Quadratic movement: IN ->start + delta * (percentage * percentage * 2)
             *                     OUT ->end - delta * ((1-percentage) * (1-percentage) * 2)
             */
            tEaseResult = QuadraticEaseIn(tPercentageOfCompletion);
            break;
        case EASE_CUBIC:
            tEaseResult = CubicEaseIn(tPercentageOfCompletion);
            break;
        case EASE_QUARTIC:
            tEaseResult = QuarticEaseIn(tPercentageOfCompletion);
            break;
#ifndef KEEP_LIBRARY_SMALL
        case EASE_SINE:
            tEaseResult = SineEaseIn(tPercentageOfCompletion);
            break;
        case EASE_CIRCULAR:
            tEaseResult = CircularEaseIn(tPercentageOfCompletion);
            break;
        case EASE_BACK:
            tEaseResult = BackEaseIn(tPercentageOfCompletion);
            break;
        case EASE_ELASTIC:
            tEaseResult = ElasticEaseIn(tPercentageOfCompletion);
            break;
        case EASE_BOUNCE:
            tEaseResult = EaseOutBounce(tPercentageOfCompletion, inFirstHalf);
            break;
#endif
        default:
            break;
        }

        uint16_t tDeltaMicroseconds = deltaMicroseconds * tEaseResult;

        if (inFirstHalf) {
            // First half: increase velocity
            tNewMicroseconds = startMicroseconds + tDeltaMicroseconds;
        } else {
            // Second half: decrease velocity
            tNewMicroseconds = endMicroseconds - tDeltaMicroseconds;
        }
    }

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
bool ServoEasing::updateSynchronized() {
    // check both servos to avoid race conditions (the servos may have slightly different end milliseconds).
    bool tServosStopped = update();
    return SynchronizedServo->update() && tServosStopped;
}

bool ServoEasing::isMoving() {
    return servoMoves;
}

uint8_t ServoEasing::getCurrentAngle() {
    return MicrosecondsToDegree(currentMicroseconds);
}

uint16_t ServoEasing::getMillisForCompleteMove() {
    return millisForCompleteMove;
}

#ifdef DEBUG
void ServoEasing::print() {
    printDynamic();
    printStatic();
}
// There should be no library which uses Serial, so enable it only for debugging purposes
void ServoEasing::printDynamic() {
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
    Serial.println(millisForCompleteMove);
}

void ServoEasing::printStatic() {
    Serial.print(F("0="));
    Serial.print(MicrosecondsForServo0Degree);
    Serial.print(F(" 180="));
    Serial.print(MicrosecondsForServo180Degree);

    Serial.print(" this=0x");
    Serial.print((uint16_t) this, HEX);
    Serial.print(F(" Synchronized=0x"));
    Serial.println((uint16_t) SynchronizedServo, HEX);
}
#endif

void enableServoEasingInterrupt() {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TIFR5 |= _BV(OCF5B);     // clear any pending interrupts;
    TIMSK5 |= _BV(OCIE5B);// enable the output compare B interrupt
    OCR5B = ((clockCyclesPerMicrosecond() * REFRESH_INTERVAL) / 8) - 100;// update values 100us before the new servo period starts
#else
    TIFR1 |= _BV(OCF1B);     // clear any pending interrupts;
    TIMSK1 |= _BV(OCIE1B);   // enable the output compare B interrupt
#ifndef USE_LEIGHTWEIGHT_SERVO_LIB
    // update values 100us before the new servo period starts
    OCR1B = ((clockCyclesPerMicrosecond() * REFRESH_INTERVAL) / 8) - 100;
#endif
#endif
}

void disableServoEasingInterrupt() {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TIMSK5 &= ~(_BV(OCIE5B)); // disable the output compare B interrupt
#else
    TIMSK1 &= ~(_BV(OCIE1B)); // disable the output compare B interrupt
#endif
}

__attribute__((weak)) void handleTimer_COMPB_Interrupt() {

    bool tServosStopped = sPointerToServoForISR->update();
    if (sPointerToServoForISR->SynchronizedServo != NULL) {
        tServosStopped = sPointerToServoForISR->SynchronizedServo->update() && tServosStopped;
    }
    if (tServosStopped) {
        // disable only if both servos stopped. This enables independent movements of 2 (synchronized) servos with this interrupt handler.
        disableServoEasingInterrupt();
    }
}

/*
 * 60us for single servo
 * 20us for last interrupt
 * The first servo pulse starts just after this interrupt routine has finished
 */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
ISR(TIMER5_COMPB_vect) {
    handleTimer_COMPB_Interrupt();
}
#else
ISR(TIMER1_COMPB_vect) {
#ifdef MEASURE_TIMING
    digitalWriteFast(TIMING_PIN, HIGH);
#endif
    handleTimer_COMPB_Interrupt();
#ifdef MEASURE_TIMING
    digitalWriteFast(TIMING_PIN, LOW);
#endif
}
#endif

/************************************
 * Sample easing functions
 ***********************************/
/*
 * The simplest non linear easing function
 */
float QuadraticEaseIn(float aPercentageOfCompletion) {
    return (2 * aPercentageOfCompletion * aPercentageOfCompletion);
}

float CubicEaseIn(float aPercentageOfCompletion) {
    return (2 * aPercentageOfCompletion * QuadraticEaseIn(aPercentageOfCompletion));
}

float QuarticEaseIn(float aPercentageOfCompletion) {
    return QuadraticEaseIn(QuadraticEaseIn(aPercentageOfCompletion));
}

/*
 * Take half of negative cosines of first quadrant
 * Is behaves almost like QUADRATIC
 */
float SineEaseIn(float aPercentageOfCompletion) {
    return 0.5 * (1 - cos(aPercentageOfCompletion * M_PI));
}

/*
 * It is very fast in the middle!
 * see: https://easings.net/#easeInOutCirc
 * and https://github.com/warrenm/AHEasing/blob/master/AHEasing/easing.c
 */
float CircularEaseIn(float aPercentageOfCompletion) {
    return 0.5 * (1 - sqrt(1 - 4 * aPercentageOfCompletion * aPercentageOfCompletion));
}

/*
 * see: https://easings.net/#easeInOutBack
 * and https://github.com/warrenm/AHEasing/blob/master/AHEasing/easing.c
 */
float BackEaseIn(float aPercentageOfCompletion) {
    aPercentageOfCompletion = aPercentageOfCompletion * 2;
    return 0.5
            * ((aPercentageOfCompletion * aPercentageOfCompletion * aPercentageOfCompletion)
                    - (aPercentageOfCompletion * sin(aPercentageOfCompletion * M_PI)));
}

/*
 * see: https://easings.net/#easeInOutElastic
 * and https://github.com/warrenm/AHEasing/blob/master/AHEasing/easing.c
 */
float ElasticEaseIn(float aPercentageOfCompletion) {
    return 0.5 * sin(13 * M_PI_2 * (2 * aPercentageOfCompletion)) * pow(2, 10 * ((2 * aPercentageOfCompletion) - 1));
}

/*
 * see: https://easings.net/de#easeOutBounce
 * and https://github.com/warrenm/AHEasing/blob/master/AHEasing/easing.c
 */
float EaseOutBounce(float aPercentageOfCompletion, bool aInFirstHalf) {
// convert aPercentageOfCompletion to 0 to 1 range
    if (!aInFirstHalf) {
        aPercentageOfCompletion = 1 - aPercentageOfCompletion;
    }

    float tRetval;
    if (aPercentageOfCompletion < 4 / 11.0) {
        tRetval = (121 * aPercentageOfCompletion * aPercentageOfCompletion) / 16.0;
    } else if (aPercentageOfCompletion < 8 / 11.0) {
        tRetval = (363 / 40.0 * aPercentageOfCompletion * aPercentageOfCompletion) - (99 / 10.0 * aPercentageOfCompletion)
                + 17 / 5.0;
    } else if (aPercentageOfCompletion < 9 / 10.0) {
        tRetval = (4356 / 361.0 * aPercentageOfCompletion * aPercentageOfCompletion) - (35442 / 1805.0 * aPercentageOfCompletion)
                + 16061 / 1805.0;
    } else {
        tRetval = (54 / 5.0 * aPercentageOfCompletion * aPercentageOfCompletion) - (513 / 25.0 * aPercentageOfCompletion)
                + 268 / 25.0;
    }

// pre compensate for later mirroring :-(
    if (!aInFirstHalf) {
        tRetval = 1 - tRetval;
    }
    return tRetval;
}
