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
 *  Internally only microseconds (if using Servo library) or units (if using PCA9685 expander) are used to speed up things.
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
#if defined(MEASURE_TIMING)
#include "digitalWriteFast.h"
#define TIMING_PIN 12
#endif

/*
 * list to hold all ServoEasing Objects in order to move them together
 * Cannot use "static servo_t servos[MAX_SERVOS];" from Servo library since it is static :-(
 */
uint8_t sServoCounter = 0;
ServoEasing * sServoArray[MAX_SERVOS];
// used to move all servos
uint8_t sServoNextPositionArray[MAX_SERVOS];

#if defined(USE_PCA9685_SERVO_EXPANDER)
// Constructor with I2C address needed
ServoEasing::ServoEasing(uint8_t aPCA9685I2CAddress, TwoWire *aI2CClass) { // @suppress("Class members should be properly initialized")
    mPCA9685I2CAddress = aPCA9685I2CAddress;
    mI2CClass = aI2CClass;

    mServo0DegreeMicrosecondsOrUnits = DEFAULT_PCA9685_UNITS_FOR_0_DEGREE;
    mServo180DegreeMicrosecondsOrUnits = DEFAULT_PCA9685_UNITS_FOR_180_DEGREE;
    mTrimMicrosecondsOrUnits = 0;
    mEasingType = EASE_LINEAR;
    mOperateServoReverse = false;

    mUserEaseInFunction = NULL;

    // Put this object into list of servos
    if (sServoCounter < MAX_SERVOS) {
        sServoArray[sServoCounter] = this;
        mServoIndex = sServoCounter;
        sServoCounter++;
    }

#if defined(MEASURE_TIMING)
    pinMode(TIMING_PIN, OUTPUT);
#endif
}

void ServoEasing::PCA9685Init() {
    // initialize I2C
    mI2CClass->begin();
    mI2CClass->setClock(800000);// 1000000 does not work - maybe because of parasitic breadboard capacities
    // Send software reset to expander
    mI2CClass->beginTransmission(PCA9685_GENERAL_CALL_ADDRESS);
    mI2CClass->write(PCA9685_SOFTWARE_RESET);
    mI2CClass->endTransmission();
    // set to 20 ms period
    I2CWriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_SLEEP));// go to sleep
    I2CWriteByte(PCA9685_PRESCALE_REGISTER, PCA9685_PRESCALER_FOR_20_MS);// set the prescaler
    I2CWriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_AUTOINCREMENT));// reset sleep and enable auto increment
    delay(2);// 500 us according to datasheet
}

void ServoEasing::I2CWriteByte(uint8_t aAddress, uint8_t aData) {
    mI2CClass->beginTransmission(mPCA9685I2CAddress);
    mI2CClass->write(aAddress);
    mI2CClass->write(aData);
    mI2CClass->endTransmission();
}

void ServoEasing::setPWM(uint16_t aOffUnits) {
    mI2CClass->beginTransmission(mPCA9685I2CAddress);
    // +2 since we we do not set the begin, it is fixed at 0
    mI2CClass->write((PCA9685_FIRST_PWM_REGISTER + 2) + 4 * mServoPin);
    mI2CClass->write(aOffUnits);
    mI2CClass->write(aOffUnits >> 8);
    mI2CClass->endTransmission();
}

uint16_t ServoEasing::MicrosecondsToPCA9685Units(uint16_t aMicroseconds) {
    /*
     * 4096 units per 20 milliseconds
     */
    return ((4096L * aMicroseconds) / REFRESH_INTERVAL);
}

#else
// Constructor without I2C address
ServoEasing::ServoEasing() // @suppress("Class members should be properly initialized")
#if not defined(USE_LEIGHTWEIGHT_SERVO_LIB) && not defined(USE_PCA9685_SERVO_EXPANDER)
:
        Servo()
#endif
{
    mServo0DegreeMicrosecondsOrUnits = DEFAULT_MICROSECONDS_FOR_0_DEGREE;
    mServo180DegreeMicrosecondsOrUnits = DEFAULT_MICROSECONDS_FOR_180_DEGREE;
    mTrimMicrosecondsOrUnits = 0;
    mEasingType = EASE_LINEAR;
    mOperateServoReverse = false;

    mUserEaseInFunction = NULL;

    // Put this object into list of servos
    if (sServoCounter < MAX_SERVOS) {
        sServoArray[sServoCounter] = this;
        mServoIndex = sServoCounter;
        sServoCounter++;
    }

#if defined(MEASURE_TIMING)
    pinMode(TIMING_PIN, OUTPUT);
#endif
}
#endif

/*
 * Return 0/false if not pin 9 or 10 else return aPin
 * Pin number != 9 results in using pin 10.
 * For PCA9685 expansion return true only if channel number between 0 and 15 since PCA9685 has only 16 channels
 */
uint8_t ServoEasing::attach(int aPin) {
    mServoPin = aPin;

#if defined(USE_LEIGHTWEIGHT_SERVO_LIB)
    if(aPin != 9 && aPin != 10) {
        return false;
    }
    return aPin;
#elif defined(USE_PCA9685_SERVO_EXPANDER)
    if (mServoIndex == 0) {
        PCA9685Init();
    }
    return (aPin <= PCA9685_MAX_CHANNELS);
#else
    return Servo::attach(aPin);
#endif
}

uint8_t ServoEasing::attach(int aPin, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree) {

    mServoPin = aPin;
#if defined(USE_PCA9685_SERVO_EXPANDER)
    mServo0DegreeMicrosecondsOrUnits = MicrosecondsToPCA9685Units(aMicrosecondsForServo0Degree);
    mServo180DegreeMicrosecondsOrUnits = MicrosecondsToPCA9685Units(aMicrosecondsForServo180Degree);
#else
    mServo0DegreeMicrosecondsOrUnits = aMicrosecondsForServo0Degree;
    mServo180DegreeMicrosecondsOrUnits = aMicrosecondsForServo180Degree;
#endif

#if defined(USE_PCA9685_SERVO_EXPANDER)
    if (mServoIndex == 0) {
        PCA9685Init();
    }
    return (aPin <= PCA9685_MAX_CHANNELS);
#elif defined(USE_LEIGHTWEIGHT_SERVO_LIB)
    if(aPin != 9 && aPin != 10) {
        return false;
    }
    return aPin;
#else
    return Servo::attach(aPin, aMicrosecondsForServo0Degree, aMicrosecondsForServo180Degree);
#endif
}

// Set a flag which is only used at writeMicrosecondsOrUnits
void ServoEasing::setReverseOperation(bool aOperateServoReverse) {
    mOperateServoReverse = aOperateServoReverse;
}

uint16_t ServoEasing::getSpeed() {
    return mSpeed;
}

void ServoEasing::setSpeed(uint16_t aDegreesPerSecond) {
    mSpeed = aDegreesPerSecond;
}

void ServoEasing::setTrim(int8_t aTrim) {
    if (aTrim >= 0) {
        setTrimMicrosecondsOrUnits(DegreeToMicrosecondsOrUnits(aTrim) - mServo0DegreeMicrosecondsOrUnits);
    } else {
        setTrimMicrosecondsOrUnits(-(DegreeToMicrosecondsOrUnits(-aTrim) - mServo0DegreeMicrosecondsOrUnits));
    }
}

void ServoEasing::setTrimMicrosecondsOrUnits(int16_t aTrimMicrosecondsOrUnits) {
    mTrimMicrosecondsOrUnits = aTrimMicrosecondsOrUnits;
    writeMicrosecondsOrUnits(mCurrentMicrosecondsOrUnits);
}

void ServoEasing::setEasingType(uint8_t aEasingType) {
    mEasingType = aEasingType;
}

uint8_t ServoEasing::getEasingType() {
    return (mEasingType);
}

void ServoEasing::registerUserEaseInFunction(float (*aUserEaseInFunction)(float aPercentageOfCompletion)) {
    mUserEaseInFunction = aUserEaseInFunction;
}

void ServoEasing::write(int aValue) {
#if defined(TRACE)
    Serial.print(F("write "));
    Serial.print(aValue);
    Serial.print(' ');
#endif
    if (aValue < 400) { // treat values less than 400 as angles in degrees (valid values in microseconds are handled as microseconds)
        aValue = DegreeToMicrosecondsOrUnits(aValue);
    }
    writeMicrosecondsOrUnits(aValue);
}

/*
 * Adds trim value and call LeightweightServo or Servo library for generating the pulse
 * Before sending to the underlying Servo library trim and reverse is applied
 */
void ServoEasing::writeMicrosecondsOrUnits(int aValue) {
    mCurrentMicrosecondsOrUnits = aValue;

#if defined(TRACE)
    Serial.print(mServoIndex);
    Serial.print('/');
    Serial.print(mServoPin);
    Serial.print(F(" us/u="));
    Serial.print(aValue);
    if (mTrimMicrosecondsOrUnits != 0) {
        Serial.print(" t=");
        Serial.print(aValue + mTrimMicrosecondsOrUnits);
    }
#endif // TRACE

    // Apply trim - this is the only place mTrimMicrosecondsOrUnits is evaluated
    aValue += mTrimMicrosecondsOrUnits;
    // Apply reverse - this is the only place mOperateServoReverse is evaluated
    if (mOperateServoReverse) {
        aValue = mServo180DegreeMicrosecondsOrUnits - (aValue - mServo0DegreeMicrosecondsOrUnits);
#if defined(TRACE)
        Serial.print(F(" r="));
        Serial.print(aValue);
#endif
    }

#if defined(TRACE)
    Serial.println();
#endif

#if defined(USE_LEIGHTWEIGHT_SERVO_LIB)
    writeMicrosecondsLightweightServo(aValue, (mServoPin == 9));
#elif defined(USE_PCA9685_SERVO_EXPANDER)
    setPWM(aValue);
#else
    Servo::writeMicroseconds(aValue); // needs 7 us
#endif

}

uint8_t ServoEasing::MicrosecondsOrUnitsToDegree(uint16_t aMicrosecondsOrUnits) {
    /*
     * Formula for microseconds:
     * (aMicrosecondsOrUnits - mServo0DegreeMicrosecondsOrUnits) * (180 / 1856) // 1856 = 180 - 0 degree micros
     * Formula for PCA9685 units
     * (aMicrosecondsOrUnits - mServo0DegreeMicrosecondsOrUnits) * (180 / 380) // 380 = 180 - 0 degree units
     * Formula for both without rounding
     * map(aMicrosecondsOrUnits, mServo0DegreeMicrosecondsOrUnits, mServo180DegreeMicrosecondsOrUnits, 0, 180)
     */

    // map with rounding
    int32_t tResult = aMicrosecondsOrUnits - mServo0DegreeMicrosecondsOrUnits;
#if defined(USE_PCA9685_SERVO_EXPANDER)
    tResult = (tResult * 180) + 190;
#else
    tResult = (tResult * 180) + 928;
#endif
    return (tResult / (mServo180DegreeMicrosecondsOrUnits - mServo0DegreeMicrosecondsOrUnits));

}

uint16_t ServoEasing::DegreeToMicrosecondsOrUnits(uint8_t aDegree) {
    // For microseconds and PCA9685 units:
    return map(aDegree, 0, 180, mServo0DegreeMicrosecondsOrUnits, mServo180DegreeMicrosecondsOrUnits);
}

void ServoEasing::easeTo(uint8_t aDegree) {
    easeTo(aDegree, mSpeed);
}

/*
 * Blocking move without interrupt
 * aDegreesPerSecond can range from 1 to the physically maximum value of 450
 */
void ServoEasing::easeTo(uint8_t aDegree, uint16_t aDegreesPerSecond) {
    startEaseTo(aDegree, aDegreesPerSecond, false);
    do {
        // First do the delay, then check for update, since we are likely called directly after start and there is nothing to move yet
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!update());
}

void ServoEasing::easeToD(uint8_t aDegree, uint16_t aMillisForMove) {
    startEaseToD(aDegree, aMillisForMove, false);
    do {
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!update());
}

bool ServoEasing::setEaseTo(uint8_t aDegree) {
    return startEaseTo(aDegree, mSpeed, false);
}

/*
 * Sets easing parameter, but do not start
 * returns false if servo was still moving
 */
bool ServoEasing::setEaseTo(uint8_t aDegree, uint16_t aDegreesPerSecond) {
    return startEaseTo(aDegree, aDegreesPerSecond, false);
}

bool ServoEasing::startEaseTo(uint8_t aDegree) {
    return startEaseTo(aDegree, mSpeed, true);
}

/*
 * sets up all the values needed for a smooth move to new value
 * returns false if servo was still moving
 */
bool ServoEasing::startEaseTo(uint8_t aDegree, uint16_t aDegreesPerSecond, bool aStartUpdateByInterrupt) {
    uint8_t tCurrentAngle = MicrosecondsOrUnitsToDegree(mCurrentMicrosecondsOrUnits);
    if (aDegree == tCurrentAngle) {
        // no effective movement -> return
        return !mServoMoves;
    }
    uint16_t tMillisForCompleteMove = abs(aDegree - tCurrentAngle) * 1000L / aDegreesPerSecond;
    if ((mEasingType & CALL_STYLE_MASK) == CALL_STYLE_BOUNCING_OUT_IN) {
        // bouncing has double movement
        tMillisForCompleteMove *= 2;
    }
    return startEaseToD(aDegree, tMillisForCompleteMove, aStartUpdateByInterrupt);
}

/*
 * Sets easing parameter, but do not start
 */
bool ServoEasing::setEaseToD(uint8_t aDegree, uint16_t aMillisForMove) {
    return startEaseToD(aDegree, aMillisForMove, false);
}

/*
 * returns false if servo was still moving
 */
bool ServoEasing::startEaseToD(uint8_t aDegree, uint16_t aMillisForMove, bool aStartUpdateByInterrupt) {
    // writw the position also to sServoNextPositionArray
    sServoNextPositionArray[mServoIndex] = aDegree;
    mEndMicrosecondsOrUnits = DegreeToMicrosecondsOrUnits(aDegree);
    uint16_t tCurrentMicroseconds = mCurrentMicrosecondsOrUnits;
    mDeltaMicrosecondsOrUnits = mEndMicrosecondsOrUnits - tCurrentMicroseconds;

    mMillisForCompleteMove = aMillisForMove;
    mStartMicrosecondsOrUnits = tCurrentMicroseconds;

    if ((mEasingType & CALL_STYLE_MASK) == CALL_STYLE_BOUNCING_OUT_IN) {
        // bouncing has same end position as start position
        mEndMicrosecondsOrUnits = tCurrentMicroseconds;
    }

    mMillisAtStartMove = millis();

#if defined(TRACE)
    printDynamic(&Serial, true);
#elif defined(DEBUG)
    printDynamic(&Serial);
#endif

    bool tReturnValue = !mServoMoves;

// Check after printDynamic() to see the values
    if (mDeltaMicrosecondsOrUnits != 0) {
        mServoMoves = true;
        if (aStartUpdateByInterrupt) {
            enableServoEasingInterrupt();
        }
    }
    return tReturnValue;
}

float ServoEasing::callEasingFunction(float aPercentageOfCompletion) {
    uint8_t tEasingType = mEasingType & EASE_TYPE_MASK;

    switch (tEasingType) {

    case EASE_USER_DIRECT:
        if (mUserEaseInFunction != NULL) {
            return mUserEaseInFunction(aPercentageOfCompletion);
        } else {
            return 0.0;
        }

    case EASE_QUADRATIC_IN:
        return QuadraticEaseIn(aPercentageOfCompletion);
    case EASE_CUBIC_IN:
        return CubicEaseIn(aPercentageOfCompletion);
    case EASE_QUARTIC_IN:
        return QuarticEaseIn(aPercentageOfCompletion);
#ifndef KEEP_LIBRARY_SMALL
    case EASE_SINE_IN:
        return SineEaseIn(aPercentageOfCompletion);
    case EASE_CIRCULAR_IN:
        return CircularEaseIn(aPercentageOfCompletion);
    case EASE_BACK_IN:
        return BackEaseIn(aPercentageOfCompletion);
    case EASE_ELASTIC_IN:
        return ElasticEaseIn(aPercentageOfCompletion);
    case EASE_BOUNCE_OUT:
        return EaseOutBounce(aPercentageOfCompletion);
#endif
    default:
        return 0.0;
    }
}

/*
 * returns true if endAngle was reached / servo stopped
 */
bool ServoEasing::update() {

    if (!mServoMoves) {
        return true;
    }

    uint32_t tMillisSinceStart = millis() - mMillisAtStartMove;
    if (tMillisSinceStart >= mMillisForCompleteMove) {
        // end of time reached -> write end position and return true
        writeMicrosecondsOrUnits(mEndMicrosecondsOrUnits);
        mServoMoves = false;
        return true;
    }

    uint16_t tNewMicrosecondsOrUnits;
    if (mEasingType == EASE_LINEAR) {
        /*
         * Use faster non float arithmetic
         * Linear movement: new position is: start position + total delta * (millis_done / millis_total aka "percentage of completion")
         * 40 us to compute
         */
        tNewMicrosecondsOrUnits = mStartMicrosecondsOrUnits
                + ((mDeltaMicrosecondsOrUnits * (int32_t) tMillisSinceStart) / mMillisForCompleteMove);
    } else {
        /*
         * Non linear movement -> use floats
         * Compute tPercentageOfCompletion - from 0.0 to 1.0
         * The expected result of easing function is from 0.0 to 1.0
         */
        float tPercentageOfCompletion = (float) tMillisSinceStart / mMillisForCompleteMove;
        float tEaseResult = 0.0;

        uint8_t tCallStyle = mEasingType & CALL_STYLE_MASK; // Values are CALL_STYLE_DIRECT, CALL_STYLE_OUT, CALL_STYLE_IN_OUT, CALL_STYLE_BOUNCING_OUT_IN

        if (tCallStyle == CALL_STYLE_DIRECT) {
            // Use IN function direct: Call with PercentageOfCompletion | 0.0 to 1.0. Result is from 0.0 to 1.0
            tEaseResult = callEasingFunction(tPercentageOfCompletion);

        } else if (tCallStyle == CALL_STYLE_OUT) {
            // Use IN function to generate OUT function: Call with (1 - PercentageOfCompletion) | 1.0 to 0.0. Result = (1 - result)
            tEaseResult = 1.0 - (callEasingFunction(1.0 - tPercentageOfCompletion));

        } else {
            if (tPercentageOfCompletion <= 0.5) {
                if (tCallStyle == CALL_STYLE_IN_OUT) {
                    // In the first half, call with (2 * PercentageOfCompletion) | 0.0 to 1.0. Result = (0.5 * result)
                    tEaseResult = 0.5 * (callEasingFunction(2.0 * tPercentageOfCompletion));
                }
                if (tCallStyle == CALL_STYLE_BOUNCING_OUT_IN) {
                    // In the first half, call with (1 - (2 * PercentageOfCompletion)) | 1.0 to 0.0. Result = (1 - result) -> call OUT function faster.
                    tEaseResult = 1.0 - (callEasingFunction(1.0 - (2.0 * tPercentageOfCompletion)));
                }
            } else {
                if (tCallStyle == CALL_STYLE_IN_OUT) {
                    // In the second half, call with (2 - (2 * PercentageOfCompletion)) | 1.0 to 0.0. Result = ( 1- (0.5 * result))
                    tEaseResult = 1.0 - (0.5 * (callEasingFunction(2.0 - (2.0 * tPercentageOfCompletion))));
                }
                if (tCallStyle == CALL_STYLE_BOUNCING_OUT_IN) {
                    // In the second half, call with ((2 * PercentageOfCompletion) - 1) | 0.0 to 1.0. Result = (1- result) -> call OUT function faster and backwards.
                    tEaseResult = 1.0 - (callEasingFunction((2.0 * tPercentageOfCompletion) - 1.0));
                }
            }
        }

        uint16_t tDeltaMicroseconds = mDeltaMicrosecondsOrUnits * tEaseResult;
        tNewMicrosecondsOrUnits = mStartMicrosecondsOrUnits + tDeltaMicroseconds;
    }

    /*
     * Write new position only if changed
     */
    if (tNewMicrosecondsOrUnits != mCurrentMicrosecondsOrUnits) {
        writeMicrosecondsOrUnits(tNewMicrosecondsOrUnits);
    }
    return false;
}

bool ServoEasing::isMoving() {
    return mServoMoves;
}

uint8_t ServoEasing::getCurrentAngle() {
    return MicrosecondsOrUnitsToDegree(mCurrentMicrosecondsOrUnits);
}

uint16_t ServoEasing::getEndMicrosecondsOrUnits() {
    return mEndMicrosecondsOrUnits;
}

uint16_t ServoEasing::getDeltaMicrosecondsOrUnits() {
    return mDeltaMicrosecondsOrUnits;
}

uint16_t ServoEasing::getMillisForCompleteMove() {
    return mMillisForCompleteMove;
}

void ServoEasing::print(Stream * aSerial) {
    printDynamic(aSerial);
    printStatic(aSerial);
}

/*
 * Prints values which may change from move to move.
 */
void ServoEasing::printDynamic(Stream * aSerial, bool doExtendedOutput) {
// pin is static but it is needed for identifying the servo
    aSerial->print(mServoIndex);
    aSerial->print('/');
    aSerial->print(mServoPin);
    aSerial->print(F(": "));

    aSerial->print(MicrosecondsOrUnitsToDegree(mCurrentMicrosecondsOrUnits));
    if (doExtendedOutput) {
        aSerial->print('|');
        aSerial->print(mCurrentMicrosecondsOrUnits);
    }

    aSerial->print(F(" -> "));
    aSerial->print(MicrosecondsOrUnitsToDegree(mEndMicrosecondsOrUnits));
    if (doExtendedOutput) {
        aSerial->print('|');
        aSerial->print(mEndMicrosecondsOrUnits);
    }

    aSerial->print(F(" = "));
    int16_t tDelta;
    if (mDeltaMicrosecondsOrUnits >= 0) {
        tDelta = MicrosecondsOrUnitsToDegree(mDeltaMicrosecondsOrUnits + mServo0DegreeMicrosecondsOrUnits);
    } else {
        tDelta = -MicrosecondsOrUnitsToDegree(mServo0DegreeMicrosecondsOrUnits - mDeltaMicrosecondsOrUnits);
    }
    aSerial->print(tDelta);
    if (doExtendedOutput) {
        aSerial->print('|');
        aSerial->print(mDeltaMicrosecondsOrUnits);
    }

    aSerial->print(F(" in "));
    aSerial->print(mMillisForCompleteMove);
    aSerial->print(F(" ms"));

    if (doExtendedOutput) {
        aSerial->print(F(" mMillisAtStartMove="));
        aSerial->print(mMillisAtStartMove);
    }

    aSerial->println();
}

/*
 * Prints values which normally does NOT change from move to move.
 */
void ServoEasing::printStatic(Stream * aSerial) {

    aSerial->print(F("0="));
    aSerial->print(mServo0DegreeMicrosecondsOrUnits);
    aSerial->print(F(" 180="));
    aSerial->print(mServo180DegreeMicrosecondsOrUnits);

    aSerial->print(F(" trim="));
    if (mTrimMicrosecondsOrUnits >= 0) {
        aSerial->print(MicrosecondsOrUnitsToDegree(mTrimMicrosecondsOrUnits + mServo0DegreeMicrosecondsOrUnits));
    } else {
        aSerial->print(-MicrosecondsOrUnitsToDegree(mServo0DegreeMicrosecondsOrUnits - mTrimMicrosecondsOrUnits));
    }
    aSerial->print('|');
    aSerial->print(mTrimMicrosecondsOrUnits);

    aSerial->print(F(" reverse="));
    aSerial->print(mOperateServoReverse);

    aSerial->print(F(" type=0x"));
    aSerial->print(mEasingType, HEX);

    aSerial->print(F(" MAX_SERVOS="));
    aSerial->print(MAX_SERVOS);

    aSerial->print(" this=0x");
    aSerial->println((uint16_t) this, HEX);
}

void enableServoEasingInterrupt() {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#if defined(USE_PCA9685_SERVO_EXPANDER)
    TODO
    // set timer 1 to 20 ms
    TCCR1A = _BV(WGM11);// FastPWM Mode mode TOP (20ms) determined by ICR1 - non-inverting Compare Output mode OC1A+OC1B
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);// set prescaler to 8, FastPWM mode mode bits WGM13 + WGM12
    ICR1 = 40000;// set period to 20 ms
#endif

    TIFR5 |= _BV(OCF5B);     // clear any pending interrupts;
    TIMSK5 |= _BV(OCIE5B);// enable the output compare B interrupt
    OCR5B = ((clockCyclesPerMicrosecond() * REFRESH_INTERVAL) / 8) - 100;// update values 100us before the new servo period starts
#else

#if defined(USE_PCA9685_SERVO_EXPANDER)
//    // set timer 1 to 20 ms
    TCCR1A = _BV(WGM11);// FastPWM Mode mode TOP (20ms) determined by ICR1 - non-inverting Compare Output mode OC1A+OC1B
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);// set prescaler to 8, FastPWM mode mode bits WGM13 + WGM12
    ICR1 = 40000;// set period to 20 ms
#endif

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

/*
 * Update all servos from list and check if all servos have stopped.
 */
__attribute__((weak)) void handleTimer_COMPB_Interrupt() {
#if defined(USE_PCA9685_SERVO_EXPANDER)
    // Otherwise it will hang forever in I2C transfer
    sei();
#endif
    if (updateAllServos()) {
        // disable interrupt only if all servos stopped. This enables independent movements of servos with this interrupt handler.
        disableServoEasingInterrupt();
    }
}

/*
 * 60us for single servo + 160 us per servo if using I2C e.g.for PCA9685 expander at 400000Hz or + 100 at 800000Hz
 * 20us for last interrupt
 * The first servo pulse starts just after this interrupt routine has finished
 */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
ISR(TIMER5_COMPB_vect) {
    handleTimer_COMPB_Interrupt();
}
#else
ISR(TIMER1_COMPB_vect) {
#if defined(MEASURE_TIMING)
    digitalWriteFast(TIMING_PIN, HIGH);
#endif
    handleTimer_COMPB_Interrupt();
#if defined(MEASURE_TIMING)
    digitalWriteFast(TIMING_PIN, LOW);
#endif
}
#endif

/************************************
 * ServoEasing list functions
 ***********************************/

void setEasingTypeForAllServos(uint8_t aEasingType) {
    uint8_t tServoIndex = 0;
    while (sServoArray[tServoIndex] != NULL && tServoIndex < MAX_SERVOS) {
        sServoArray[tServoIndex]->mEasingType = aEasingType;
        tServoIndex++;
    }
}

void setEaseToForAllServosSynchronizeAndStartInterrupt() {
    setEaseToForAllServos();
    synchronizeAllServosAndStartInterrupt();
}

void setEaseToForAllServosSynchronizeAndStartInterrupt(uint16_t aDegreesPerSecond) {
    setEaseToForAllServos(aDegreesPerSecond);
    synchronizeAllServosAndStartInterrupt();
}

void synchronizeAndEaseToArrayPositions() {
    setEaseToForAllServos();
    synchronizeAllServosStartAndWaitForAllServosToStop();
}

void synchronizeAndEaseToArrayPositions(uint16_t aDegreesPerSecond) {
    setEaseToForAllServos(aDegreesPerSecond);
    synchronizeAllServosStartAndWaitForAllServosToStop();
}

void printArrayPositions(Stream * aSerial) {
    uint8_t tServoIndex = 0;
    aSerial->print(F("ServoNextPositionArray="));
    while (sServoArray[tServoIndex] != NULL && tServoIndex < MAX_SERVOS) {
        aSerial->print(sServoNextPositionArray[tServoIndex]);
        aSerial->print(F(" | "));
        tServoIndex++;
    }
    aSerial->println();
}

void setSpeedForAllServos(uint16_t aDegreesPerSecond) {
    uint8_t tServoIndex = 0;
    while (sServoArray[tServoIndex] != NULL && tServoIndex < MAX_SERVOS) {
        sServoArray[tServoIndex]->mSpeed = aDegreesPerSecond;
        tServoIndex++;
    }
}

void setDegreeForAllServos(uint8_t aNumberOfValues, va_list * aDegreeValues) {
    for (uint8_t tServoIndex = 0; tServoIndex < aNumberOfValues; ++tServoIndex) {
        sServoNextPositionArray[tServoIndex] = va_arg(*aDegreeValues, uint16_t);
    }
}

void setDegreeForAllServos(uint8_t aNumberOfValues, ...) {
    va_list aDegreeValues;
    va_start(aDegreeValues, aNumberOfValues);
    setDegreeForAllServos(aNumberOfValues, &aDegreeValues);
    va_end(aDegreeValues);
}

/*
 * Sets target position using content of sServoNextPositionArray
 * returns false if one servo was still moving
 */
bool setEaseToForAllServos() {
    uint8_t tServoIndex = 0;
    bool tOneServoIsMoving = false;
    while (sServoArray[tServoIndex] != NULL && tServoIndex < MAX_SERVOS) {
        tOneServoIsMoving = sServoArray[tServoIndex]->setEaseTo(sServoNextPositionArray[tServoIndex],
                sServoArray[tServoIndex]->mSpeed) || tOneServoIsMoving;
        tServoIndex++;
    }
    return tOneServoIsMoving;
}

bool setEaseToForAllServos(uint16_t aDegreesPerSecond) {
    uint8_t tServoIndex = 0;
    bool tOneServoIsMoving = false;
    while (sServoArray[tServoIndex] != NULL && tServoIndex < MAX_SERVOS) {
        tOneServoIsMoving = sServoArray[tServoIndex]->setEaseTo(sServoNextPositionArray[tServoIndex], aDegreesPerSecond)
                || tOneServoIsMoving;
        tServoIndex++;
    }
    return tOneServoIsMoving;
}

bool isOneServoMoving() {
    uint8_t tServoIndex = 0;
    while (sServoArray[tServoIndex] != NULL && tServoIndex < MAX_SERVOS) {
        if (sServoArray[tServoIndex]->mServoMoves) {
            return true;
        }
        tServoIndex++;
    }
    return false;
}

void stopAllServos() {
    void disableServoEasingInterrupt();
    uint8_t tServoIndex = 0;
    while (sServoArray[tServoIndex] != NULL && tServoIndex < MAX_SERVOS) {
        sServoArray[tServoIndex]->mServoMoves = false;
        tServoIndex++;
    }
}

/*
 * returns true if all Servos reached endAngle / stopped
 */
bool updateAllServos() {
    uint8_t tServoIndex = 0;
    bool tAllServosStopped = true;
    while (sServoArray[tServoIndex] != NULL && tServoIndex < MAX_SERVOS) {
        tAllServosStopped = sServoArray[tServoIndex]->update() && tAllServosStopped;
        tServoIndex++;
    }
    return tAllServosStopped;
}

void updateAndWaitForAllServosToStop() {
    do {
        // First do the delay, then check for update, since we are likely called directly after start and there is nothing to move yet
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!updateAllServos());
}

void synchronizeAllServosStartAndWaitForAllServosToStop() {
    synchronizeAllServosAndStartInterrupt(false);
    updateAndWaitForAllServosToStop();
}

/*
 * Take the longer duration in order to move all servos synchronously
 */
void synchronizeAllServosAndStartInterrupt(bool aStartUpdateByInterrupt) {
    /*
     * Find maximum duration and one start time
     */
    uint8_t tServoIndex = 0;
    uint16_t tMaxMillisForCompleteMove = 0;
    uint32_t tMillisAtStartMove = 0;

    while (sServoArray[tServoIndex] != NULL && tServoIndex < MAX_SERVOS) {
        if (sServoArray[tServoIndex]->mServoMoves) {
            tMillisAtStartMove = sServoArray[tServoIndex]->mMillisAtStartMove;
            if (sServoArray[tServoIndex]->mMillisForCompleteMove > tMaxMillisForCompleteMove) {
                tMaxMillisForCompleteMove = sServoArray[tServoIndex]->mMillisForCompleteMove;
            }
        }
        tServoIndex++;
    }

#if defined(TRACE)
    Serial.print(F("ServoEasingCounter="));
    Serial.print(sServoCounter);
    Serial.print(F(" MillisAtStartMove="));
    Serial.print(tMillisAtStartMove);
    Serial.print(F(" MaxMillisForCompleteMove="));
    Serial.println(tMaxMillisForCompleteMove);
#endif

    /*
     * Set maximum duration and start time to all servos
     * Synchronize start time to avoid race conditions at the end of movement
     */
    tServoIndex = 0;
    while (sServoArray[tServoIndex] != NULL && tServoIndex < MAX_SERVOS) {
        if (sServoArray[tServoIndex]->mServoMoves) {
            sServoArray[tServoIndex]->mMillisAtStartMove = tMillisAtStartMove;
            sServoArray[tServoIndex]->mMillisForCompleteMove = tMaxMillisForCompleteMove;
        }
        tServoIndex++;
    }

    if (aStartUpdateByInterrupt) {
        enableServoEasingInterrupt();
    }
}

/************************************
 * Included easing functions
 * Input is from 0.0 to 1.0 and output is from 0.0 to 1.0
 ***********************************/
float (*sEaseFunctionArray[])(
        float aPercentageOfCompletion) = {&QuadraticEaseIn, &CubicEaseIn, &QuarticEaseIn, &SineEaseIn, &CircularEaseIn, &BackEaseIn, &ElasticEaseIn,
            &EaseOutBounce};
/*
 * The simplest non linear easing function
 */
float QuadraticEaseIn(float aPercentageOfCompletion) {
    return (aPercentageOfCompletion * aPercentageOfCompletion);
}

float CubicEaseIn(float aPercentageOfCompletion) {
    return (aPercentageOfCompletion * QuadraticEaseIn(aPercentageOfCompletion));
}

float QuarticEaseIn(float aPercentageOfCompletion) {
    return QuadraticEaseIn(QuadraticEaseIn(aPercentageOfCompletion));
}

/*
 * Take half of negative cosines of first quadrant
 * Is behaves almost like QUADRATIC
 */
float SineEaseIn(float aPercentageOfCompletion) {
    return sin((aPercentageOfCompletion - 1) * M_PI_2) + 1;
}

/*
 * It is very fast in the middle!
 * see: https://easings.net/#easeInOutCirc
 * and https://github.com/warrenm/AHEasing/blob/master/AHEasing/easing.c
 */
float CircularEaseIn(float aPercentageOfCompletion) {
    return 1 - sqrt(1 - (aPercentageOfCompletion * aPercentageOfCompletion));
}

/*
 * see: https://easings.net/#easeInOutBack
 * and https://github.com/warrenm/AHEasing/blob/master/AHEasing/easing.c
 */
float BackEaseIn(float aPercentageOfCompletion) {
    return (aPercentageOfCompletion * aPercentageOfCompletion * aPercentageOfCompletion)
            - (aPercentageOfCompletion * sin(aPercentageOfCompletion * M_PI));
}

/*
 * see: https://easings.net/#easeInOutElastic
 * and https://github.com/warrenm/AHEasing/blob/master/AHEasing/easing.c
 */
float ElasticEaseIn(float aPercentageOfCompletion) {
    return sin(13 * M_PI_2 * aPercentageOfCompletion) * pow(2, 10 * (aPercentageOfCompletion-1));
}

/*
 * !!! ATTENTION !!! we have only the out function implemented
 * see: https://easings.net/de#easeOutBounce
 * and https://github.com/warrenm/AHEasing/blob/master/AHEasing/easing.c
 */
float EaseOutBounce(float aPercentageOfCompletion) {
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
    return tRetval;
}
