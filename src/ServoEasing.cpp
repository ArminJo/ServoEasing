/*
 * ServoEasing.cpp
 *
 *  Enables smooth movement from one servo position to another.
 *  Linear as well as other ease movements (e.g. cubic) for all servos attached to the Arduino Servo library are provided.
 *  Interface is in degree but internally only microseconds (if using Servo library) or units (if using PCA9685 expander) are used,
 *  since the resolution is better and we avoid the map function on every Servo.write().
 *  The blocking functions wait for 20 ms since this is the default refresh time of the used Servo library.
 *
 *  The AVR Servo library supports only one timer, which means not more than 12 servos are supported using this library.
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

#if defined(ESP8266) || defined(ESP32)
#include "Ticker.h" // for ServoEasingInterrupt functions
Ticker Timer20ms;
#elif defined(STM32_HIGH_DENSITY)
#include <HardwareTimer.h> // 8 timers and 8. timer is used for tone()
HardwareTimer Timer20ms(7);
#elif defined(__STM32F1__)
#include <HardwareTimer.h>
#  if defined(STM32_HIGH_DENSITY)
HardwareTimer Timer20ms(7);  // 8 timers and 8. timer is used for tone()
#  else
/*
 * Use timer 3 for ServoEasingInterrupt functions.
 * Timer 3 blocks PA6, PA7, PB0, PB1, so if you need one them as Servo output, you must choose another timer.
 */
HardwareTimer Timer20ms(3);  // 4 timers and 4. timer is used for tone()
#  endif
#elif defined(__SAM3X8E__)  // Arduino DUE
/*
 * Timer 0 to 5 are used by Servo library (by defining handlers)
 *
 * Timer 6 is TC2 channel 0
 * Timer 7 is TC2 channel 1
 * Timer 8 is TC2 channel 2
 *
 * We use timer 8 here
 */
#define TC_FOR_20_MS_TIMER  	TC2
#define CHANNEL_FOR_20_MS_TIMER 2
#define ID_TC_FOR_20_MS_TIMER   ID_TC8
#define IRQn_FOR_20_MS_TIMER    TC8_IRQn
#define HANDLER_FOR_20_MS_TIMER TC8_Handler
#endif

// Enable this to see information on each call
// There should be no library which uses Serial, so enable it only for debugging purposes
//#define TRACE
//#define DEBUG
// Propagate debug level
#ifdef TRACE
#define DEBUG
#endif

// Enable this to generate output for Arduino Serial Plotter (Ctrl-Shift-L)
//#define PRINT_FOR_SERIAL_PLOTTER

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
uint8_t sServoArrayMaxIndex = 0; // maximum index of an attached servo in sServoArray[]
ServoEasing * sServoArray[MAX_EASING_SERVOS];
// used to move all servos
int sServoNextPositionArray[MAX_EASING_SERVOS];

#if defined(USE_PCA9685_SERVO_EXPANDER)
#  if ! defined _BV
#  define _BV(bit) (1 << (bit))
#  endif
// Constructor with I2C address needed
ServoEasing::ServoEasing(uint8_t aPCA9685I2CAddress, TwoWire *aI2CClass) { // @suppress("Class members should be properly initialized")
	mPCA9685I2CAddress = aPCA9685I2CAddress;
	mI2CClass = aI2CClass;

	// On an ESP8266 it was NOT initialized to 0 :-(.
	mTrimMicrosecondsOrUnits = 0;
	mSpeed = 0;
	mServoMoves = false;
	mOperateServoReverse = false;

#ifndef PROVIDE_ONLY_LINEAR_MOVEMENT
	mEasingType = EASE_LINEAR;
	mUserEaseInFunction = NULL;
#endif

#if defined(MEASURE_TIMING)
	pinMode(TIMING_PIN, OUTPUT);
#endif
}

void ServoEasing::I2CInit() {
// Initialize I2C
	mI2CClass->begin();
	mI2CClass->setClock(I2C_CLOCK_FREQUENCY);// 1000000 does not work for me, maybe because of parasitic breadboard capacities
}
/*
 * Initialize I2C and software reset all PCA9685 expanders
 */
void ServoEasing::PCA9685Reset() {
	// Send software reset to expander(s)
	mI2CClass->beginTransmission(PCA9685_GENERAL_CALL_ADDRESS);
	mI2CClass->write(PCA9685_SOFTWARE_RESET);
	mI2CClass->endTransmission();
}

/*
 * Set expander to 20 ms period and wait 2 milliseconds
 */
void ServoEasing::PCA9685Init() {
	// Set expander to 20 ms period
	I2CWriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_SLEEP));// go to sleep
	I2CWriteByte(PCA9685_PRESCALE_REGISTER, PCA9685_PRESCALER_FOR_20_MS);// set the prescaler
	I2CWriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_AUTOINCREMENT));// reset sleep and enable auto increment
	delay(2);// 500 us according to datasheet
}

void ServoEasing::I2CWriteByte(uint8_t aAddress, uint8_t aData) {
	mI2CClass->beginTransmission(mPCA9685I2CAddress);
	mI2CClass->write(aAddress);
	mI2CClass->write(aData);
#if defined(DEBUG)
	if (mI2CClass->endTransmission()) {
		Serial.print('-');
	}
#else
	mI2CClass->endTransmission();
#endif
}

/*
 * aPWMValueAsUnits - The point in the 4096-part cycle, where the output goes OFF (LOW). On is fixed at 0.
 * Useful values are from 111 (111.411 = 544 us) to 491 (491.52 = 2400 us)
 * 4096 means output is signal fully off
 */
void ServoEasing::setPWM(uint16_t aPWMOffValueAsUnits) {
	mI2CClass->beginTransmission(mPCA9685I2CAddress);
	// +2 since we we do not set the begin, it is fixed at 0
	mI2CClass->write((PCA9685_FIRST_PWM_REGISTER + 2) + 4 * mServoPin);
	mI2CClass->write(aPWMOffValueAsUnits);
	mI2CClass->write(aPWMOffValueAsUnits >> 8);
#if defined(DEBUG)
	if (mI2CClass->endTransmission()) {
		Serial.print('#');
	}
#else
	mI2CClass->endTransmission();
#endif
}

/*
 * Here you can specify an on/start value for the pulse in order not to start all pulses at the same time
 */
void ServoEasing::setPWM(uint16_t aPWMOnValueAsUnits, uint16_t aPWMPulseDurationAsUnits) {
	mI2CClass->beginTransmission(mPCA9685I2CAddress);
	// +2 since we we do not set the begin, it is fixed at 0
	mI2CClass->write((PCA9685_FIRST_PWM_REGISTER) + 4 * mServoPin);
	mI2CClass->write(aPWMOnValueAsUnits);
	mI2CClass->write(aPWMOnValueAsUnits >> 8);
	mI2CClass->write(aPWMOnValueAsUnits + aPWMPulseDurationAsUnits);
	mI2CClass->write((aPWMOnValueAsUnits + aPWMPulseDurationAsUnits) >> 8);
#if defined(DEBUG)
	uint8_t tError = mI2CClass->endTransmission();
	if (tError) {
		// i have seen this at my ESP32 module :-( - but it is no buffer overflow.
		Serial.print((char) (tError + '0'));// Error enum i2c_err_t: I2C_ERROR_ACK = 2, I2C_ERROR_TIMEOUT = 3
	}
#else
	mI2CClass->endTransmission();
#endif
}
int ServoEasing::MicrosecondsToPCA9685Units(int aMicroseconds) {
	/*
	 * 4096 units per 20 milliseconds => aMicroseconds / 4.8828
	 */
	return ((4096L * aMicroseconds) / REFRESH_INTERVAL);
}

#else // defined(USE_PCA9685_SERVO_EXPANDER)
// Constructor without I2C address
ServoEasing::ServoEasing() // @suppress("Class members should be properly initialized")
#if ! defined(USE_LEIGHTWEIGHT_SERVO_LIB) && not defined(USE_PCA9685_SERVO_EXPANDER)
:
		Servo()
#endif
{
	// On an ESP8266 it was NOT initialized to 0 :-(.
	mTrimMicrosecondsOrUnits = 0;
	mSpeed = 0;
	mServoMoves = false;
	mOperateServoReverse = false;

#ifndef PROVIDE_ONLY_LINEAR_MOVEMENT
	mEasingType = EASE_LINEAR;
	mUserEaseInFunction = NULL;
#endif

#if defined(MEASURE_TIMING)
	pinMode(TIMING_PIN, OUTPUT);
#endif
}
#endif // defined(USE_PCA9685_SERVO_EXPANDER)

/*
 * If USE_LEIGHTWEIGHT_SERVO_LIB is enabled:
 *      Return 0/false if not pin 9 or 10 else return aPin
 *      Pin number != 9 results in using pin 10.
 * If USE_PCA9685_SERVO_EXPANDER is enabled:
 *      Return true only if channel number is between 0 and 15 since PCA9685 has only 16 channels, else returns false
 * Else return servoIndex / internal channel number
 */
uint8_t ServoEasing::attach(int aPin) {
	return attach(aPin, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);
}

// Here no units accepted, only microseconds!
uint8_t ServoEasing::attach(int aPin, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree) {
	return attach(aPin, aMicrosecondsForServo0Degree, aMicrosecondsForServo180Degree, 0, 180);
}

/*
 * @param aMicrosecondsForServoLowDegree no units accepted, only microseconds!
 * @param aServoLowDegree can be negative. For this case an appropriate trim value is added, since this is the only way to handle negative values.
 * If USE_LEIGHTWEIGHT_SERVO_LIB is enabled:
 *      Return 0/false if not pin 9 or 10 else return aPin
 *      Pin number != 9 results in using pin 10.
 * Else return servoIndex / internal channel number
 */
uint8_t ServoEasing::attach(int aPin, int aMicrosecondsForServoLowDegree, int aMicrosecondsForServoHighDegree, int aServoLowDegree,
		int aServoHighDegree) {
	/*
	 * Get the 0 and 180 degree values.
	 */
	int tMicrosecondsForServo0Degree = map(0, aServoLowDegree, aServoHighDegree, aMicrosecondsForServoLowDegree,
			aMicrosecondsForServoHighDegree);
	int tMicrosecondsForServo180Degree = map(180, aServoLowDegree, aServoHighDegree, aMicrosecondsForServoLowDegree,
			aMicrosecondsForServoHighDegree);

	mServoPin = aPin;
#if defined(USE_PCA9685_SERVO_EXPANDER)
	mServo0DegreeMicrosecondsOrUnits = MicrosecondsToPCA9685Units(tMicrosecondsForServo0Degree);
	mServo180DegreeMicrosecondsOrUnits = MicrosecondsToPCA9685Units(tMicrosecondsForServo180Degree);
#else
	mServo0DegreeMicrosecondsOrUnits = tMicrosecondsForServo0Degree;
	mServo180DegreeMicrosecondsOrUnits = tMicrosecondsForServo180Degree;
#endif

	/*
	 * Now put this servo instance into list of servos
	 */
	mServoIndex = INVALID_SERVO; // flag indicating an invalid servo index
	for (uint8_t tServoIndex = 0; tServoIndex < MAX_EASING_SERVOS; ++tServoIndex) {
		if (sServoArray[tServoIndex] == NULL) {
			sServoArray[tServoIndex] = this;
			mServoIndex = tServoIndex;
			if (tServoIndex > sServoArrayMaxIndex) {
				sServoArrayMaxIndex = tServoIndex;
			}
			break;
		}
	}

#if defined(TRACE)
	Serial.print("Index=");
	Serial.print(mServoIndex);
	Serial.print(" pin=");
	Serial.print(mServoPin);
	Serial.print(" low=");
	Serial.print(aServoLowDegree);
	Serial.print('|');
	Serial.print(aMicrosecondsForServoLowDegree);
	Serial.print(" high=");
	Serial.print(aServoHighDegree);
	Serial.print('|');
	Serial.print(aMicrosecondsForServoHighDegree);
	Serial.print(' ');
	printStatic(&Serial);
#endif

#if defined(USE_PCA9685_SERVO_EXPANDER)
	if (mServoIndex == 0) {
		I2CInit();      	// init only once
		PCA9685Reset();// reset only once
	}
	PCA9685Init(); // initialize at every attach is simpler but initializing once for every board would be sufficient.

	return mServoIndex;
#elif defined(USE_LEIGHTWEIGHT_SERVO_LIB)
	if(aPin != 9 && aPin != 10) {
		return false;
	}
	return aPin;
#else
	if (mServoIndex == INVALID_SERVO) {
		return INVALID_SERVO;
	}
#  ifdef ARDUINO_ARCH_APOLLO3
	Servo::attach(aPin, tMicrosecondsForServo0Degree, tMicrosecondsForServo180Degree);
	return aPin; // Sparkfun apollo3 Servo library has no return value for attach :-(
#  else
	return Servo::attach(aPin, tMicrosecondsForServo0Degree, tMicrosecondsForServo180Degree);
#  endif
#endif
}

void ServoEasing::detach() {
	if (mServoIndex != INVALID_SERVO) {
		sServoArray[mServoIndex] = NULL;
		if (mServoIndex == sServoArrayMaxIndex) {
			// if servo with highest index in array was detached, compute new sServoArrayMaxIndex
			do {
				sServoArrayMaxIndex--;
				if (sServoArrayMaxIndex == 0) {
					break;
				}
			} while (sServoArray[sServoArrayMaxIndex] == NULL);
		}

#if defined(USE_PCA9685_SERVO_EXPANDER)
		setPWM(0); // set signal fully off
#elif defined(USE_LEIGHTWEIGHT_SERVO_LIB)
		deinitLightweightServoPin9_10(mServoPin == 9); // disable output and change to input
#else
		Servo::detach();
#endif
	}
	mServoMoves = false; // safety net to enable right update handling if accidentally called
	mServoIndex = INVALID_SERVO;
}

/*
 * Reverse means, that values for 180 and 0 degrees are swapped by: aValue = mServo180DegreeMicrosecondsOrUnits - (aValue - mServo0DegreeMicrosecondsOrUnits)
 * Be careful, if you specify different end values, it may not behave, as you expect.
 * For this case better use the attach function with 5 parameter.
 * This flag is only used at writeMicrosecondsOrUnits()
 */
void ServoEasing::setReverseOperation(bool aOperateServoReverse) {
	mOperateServoReverse = aOperateServoReverse;
}

uint16_t ServoEasing::getSpeed() {
	return mSpeed;
}

void ServoEasing::setSpeed(uint16_t aDegreesPerSecond) {
	mSpeed = aDegreesPerSecond;
}

/*
 * Trim value is always added to the degree/units/microseconds value requested
 */
void ServoEasing::setTrim(int aTrimDegrees, bool aDoWrite) {
	if (aTrimDegrees >= 0) {
		setTrimMicrosecondsOrUnits(DegreeToMicrosecondsOrUnits(aTrimDegrees) - mServo0DegreeMicrosecondsOrUnits, aDoWrite);
	} else {
		setTrimMicrosecondsOrUnits(-(DegreeToMicrosecondsOrUnits(-aTrimDegrees) - mServo0DegreeMicrosecondsOrUnits), aDoWrite);
	}
}

/*
 * Trim value is always added to the degree/units/microseconds value requested
 * It is only used/added at writeMicrosecondsOrUnits()
 */
void ServoEasing::setTrimMicrosecondsOrUnits(int aTrimMicrosecondsOrUnits, bool aDoWrite) {
	mTrimMicrosecondsOrUnits = aTrimMicrosecondsOrUnits;
	if (aDoWrite) {
		writeMicrosecondsOrUnits(mCurrentMicrosecondsOrUnits);
	}
}

#ifndef PROVIDE_ONLY_LINEAR_MOVEMENT
void ServoEasing::setEasingType(uint8_t aEasingType) {
	mEasingType = aEasingType;
}

uint8_t ServoEasing::getEasingType() {
	return (mEasingType);
}

void ServoEasing::registerUserEaseInFunction(float (*aUserEaseInFunction)(float aPercentageOfCompletion)) {
	mUserEaseInFunction = aUserEaseInFunction;
}
#endif

void ServoEasing::write(int aValue) {
#if defined(TRACE)
	Serial.print(F("write "));
	Serial.print(aValue);
	Serial.print(' ');
#endif
	/*
	 * Check for valid initialization of servo.
	 */
	if (mServoIndex == INVALID_SERVO) {
#if defined(TRACE)
		Serial.print(F("Error: detached servo"));
#endif
		return;
	}
	if (aValue < 400) { // treat values less than 400 as angles in degrees (valid values in microseconds are handled as microseconds)
		sServoNextPositionArray[mServoIndex] = aValue;
		aValue = DegreeToMicrosecondsOrUnits(aValue);
	}
	writeMicrosecondsOrUnits(aValue);
}

/*
 * Before sending the value to the underlying Servo library, trim and reverse is applied
 */
void ServoEasing::writeMicrosecondsOrUnits(int aValue) {
	/*
	 * Check for valid initialization of servo.
	 */
	if (mServoIndex == INVALID_SERVO) {
#if defined(TRACE)
		Serial.print(F("Error: detached servo"));
#endif
		return;
	}

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
#if defined(PRINT_FOR_SERIAL_PLOTTER)
	Serial.print(' ');
	Serial.print(aValue);
#endif

// Apply trim - this is the only place mTrimMicrosecondsOrUnits is evaluated
	aValue += mTrimMicrosecondsOrUnits;
// Apply reverse, values for 0 to 180 are swapped if reverse - this is the only place mOperateServoReverse is evaluated
// (except in the DegreeToMicrosecondsOrUnitsWithTrimAndReverse() function for external testing purposes)
	if (mOperateServoReverse) {
		aValue = mServo180DegreeMicrosecondsOrUnits - (aValue - mServo0DegreeMicrosecondsOrUnits);
#if defined(TRACE)
		Serial.print(F(" r="));
		Serial.print(aValue);
#endif
	}

#if defined(USE_LEIGHTWEIGHT_SERVO_LIB)
	writeMicrosecondsLightweightServo(aValue, (mServoPin == 9));
#elif defined(USE_PCA9685_SERVO_EXPANDER)
#  if defined(TRACE)
	Serial.print(F(" s="));
	Serial.print((mServoPin >> 1) << 8);
#  endif
	/*
	 * Distribute the servo start time over the first half of the 20 ms period.
	 * Could not use the complete period, since the last (15.) pulse is then clipped at 256 units.
	 * Needs additional 18 bytes Flash
	 */
	setPWM(((mServoPin >> 1) << 8), aValue);
#else
	Servo::writeMicroseconds(aValue); // needs 7 us
#endif

#if defined(TRACE)
	Serial.println();
#endif
}

int ServoEasing::MicrosecondsOrUnitsToDegree(int aMicrosecondsOrUnits) {
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

int ServoEasing::DegreeToMicrosecondsOrUnits(int aDegree) {
// For microseconds and PCA9685 units:
	return map(aDegree, 0, 180, mServo0DegreeMicrosecondsOrUnits, mServo180DegreeMicrosecondsOrUnits);
}

/*
 * Mainly for testing, since trim and reverse are applied at each write.
 */
int ServoEasing::DegreeToMicrosecondsOrUnitsWithTrimAndReverse(int aDegree) {
// For microseconds and PCA9685 units:
	int tResultValue = map(aDegree, 0, 180, mServo0DegreeMicrosecondsOrUnits, mServo180DegreeMicrosecondsOrUnits);
	tResultValue += mTrimMicrosecondsOrUnits;
	if (mOperateServoReverse) {
		tResultValue = mServo180DegreeMicrosecondsOrUnits - (tResultValue - mServo0DegreeMicrosecondsOrUnits);
	}
	return tResultValue;
}

void ServoEasing::easeTo(int aDegree) {
	easeTo(aDegree, mSpeed);
}

/*
 * Blocking move without interrupt
 * aDegreesPerSecond can range from 1 to the physically maximum value of 450
 */
void ServoEasing::easeTo(int aDegree, uint16_t aDegreesPerSecond) {
	startEaseTo(aDegree, aDegreesPerSecond, false);
	do {
		// First do the delay, then check for update, since we are likely called directly after start and there is nothing to move yet
		delay(REFRESH_INTERVAL / 1000); // 20 ms - REFRESH_INTERVAL is in Microseconds
	} while (!update());
}

void ServoEasing::easeToD(int aDegree, uint16_t aMillisForMove) {
	startEaseToD(aDegree, aMillisForMove, false);
	do {
		delay(REFRESH_INTERVAL / 1000); // 20 ms - REFRESH_INTERVAL is in Microseconds
	} while (!update());
}

bool ServoEasing::setEaseTo(int aDegree) {
	return startEaseTo(aDegree, mSpeed, false);
}

/*
 * Sets easing parameter, but do not start interrupt
 * returns false if servo was still moving
 */
bool ServoEasing::setEaseTo(int aDegree, uint16_t aDegreesPerSecond) {
	return startEaseTo(aDegree, aDegreesPerSecond, false);
}

/*
 * Starts interrupt for update()
 */
bool ServoEasing::startEaseTo(int aDegree) {
	return startEaseTo(aDegree, mSpeed, true);
}

/*
 * sets up all the values needed for a smooth move to new value
 * returns false if servo was still moving
 */
bool ServoEasing::startEaseTo(int aDegree, uint16_t aDegreesPerSecond, bool aStartUpdateByInterrupt) {
	int tCurrentAngle = MicrosecondsOrUnitsToDegree(mCurrentMicrosecondsOrUnits);
	if (aDegree == tCurrentAngle) {
		// no effective movement -> return
		return !mServoMoves;
	}
	if (aDegreesPerSecond == 0) {
#if defined(DEBUG)
		Serial.println(F("Speed is 0 -> set to 1"));
#endif
		aDegreesPerSecond = 1;
	}
	uint16_t tMillisForCompleteMove = abs(aDegree - tCurrentAngle) * 1000L / aDegreesPerSecond;

#ifndef PROVIDE_ONLY_LINEAR_MOVEMENT
	if ((mEasingType & CALL_STYLE_MASK) == CALL_STYLE_BOUNCING_OUT_IN) {
		// bouncing has double movement, so take double time
		tMillisForCompleteMove *= 2;
	}
#endif
	return startEaseToD(aDegree, tMillisForCompleteMove, aStartUpdateByInterrupt);
}

/*
 * Sets easing parameter, but do not start
 */
bool ServoEasing::setEaseToD(int aDegree, uint16_t aMillisForMove) {
	return startEaseToD(aDegree, aMillisForMove, false);
}

/*
 * returns false if servo was still moving
 */
bool ServoEasing::startEaseToD(int aDegree, uint16_t aMillisForMove, bool aStartUpdateByInterrupt) {
	/*
	 * Check for valid initialization of servo.
	 */
	if (mServoIndex == INVALID_SERVO) {
#if defined(TRACE)
		Serial.print(F("Error: detached servo"));
#endif
		return true;
	}
// write the position also to sServoNextPositionArray
	sServoNextPositionArray[mServoIndex] = aDegree;
	mEndMicrosecondsOrUnits = DegreeToMicrosecondsOrUnits(aDegree);
	int tCurrentMicrosecondsOrUnits = mCurrentMicrosecondsOrUnits;
	mDeltaMicrosecondsOrUnits = mEndMicrosecondsOrUnits - tCurrentMicrosecondsOrUnits;

	mMillisForCompleteMove = aMillisForMove;
	mStartMicrosecondsOrUnits = tCurrentMicrosecondsOrUnits;

#ifndef PROVIDE_ONLY_LINEAR_MOVEMENT
	if ((mEasingType & CALL_STYLE_MASK) == CALL_STYLE_BOUNCING_OUT_IN) {
		// bouncing has same end position as start position
		mEndMicrosecondsOrUnits = tCurrentMicrosecondsOrUnits;
	}
#endif

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

/*
 * returns true if endAngle was reached / servo stopped
 */
#ifdef PROVIDE_ONLY_LINEAR_MOVEMENT
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
	/*
	 * Use faster non float arithmetic
	 * Linear movement: new position is: start position + total delta * (millis_done / millis_total aka "percentage of completion")
	 * 40 us to compute
	 */
	uint16_t tNewMicrosecondsOrUnits = mStartMicrosecondsOrUnits
	+ ((mDeltaMicrosecondsOrUnits * (int32_t) tMillisSinceStart) / mMillisForCompleteMove);
	/*
	 * Write new position only if changed
	 */
	if (tNewMicrosecondsOrUnits != mCurrentMicrosecondsOrUnits) {
		writeMicrosecondsOrUnits(tNewMicrosecondsOrUnits);
	}
	return false;
}

#else // PROVIDE_ONLY_LINEAR_MOVEMENT
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

	int tNewMicrosecondsOrUnits;
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
		 * or from EASE_FUNCTION_DEGREE_OFFSET to EASE_FUNCTION_DEGREE_OFFSET + 180 for direct degree result
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

		if (tEaseResult >= 2) {
			tNewMicrosecondsOrUnits = DegreeToMicrosecondsOrUnits(tEaseResult - EASE_FUNCTION_DEGREE_INDICATOR_OFFSET + 0.5);
		} else {
			int tDeltaMicroseconds = mDeltaMicrosecondsOrUnits * tEaseResult;
			tNewMicrosecondsOrUnits = mStartMicrosecondsOrUnits + tDeltaMicroseconds;
		}
	}

	/*
	 * Write new position only if changed
	 */
	if (tNewMicrosecondsOrUnits != mCurrentMicrosecondsOrUnits) {
		writeMicrosecondsOrUnits(tNewMicrosecondsOrUnits);
	}
#if defined(PRINT_FOR_SERIAL_PLOTTER)
// call it anyway
	else {
		writeMicrosecondsOrUnits(tNewMicrosecondsOrUnits);
	}
#endif
	return false;
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
#ifndef KEEP_SERVO_EASING_LIBRARY_SMALL
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

#endif //PROVIDE_ONLY_LINEAR_MOVEMENT

bool ServoEasing::isMoving() {
	return mServoMoves;
}

/*
 * Call yield here, so the user do not need not care for it in long running loops.
 * This has normally no effect for AVR code, but is at least needed for ESP code.
 */
bool ServoEasing::isMovingAndCallYield() {
	yield();
	return mServoMoves;
}

int ServoEasing::getCurrentAngle() {
	return MicrosecondsOrUnitsToDegree(mCurrentMicrosecondsOrUnits);
}

int ServoEasing::getEndMicrosecondsOrUnits() {
	return mEndMicrosecondsOrUnits;
}

/*
 * Not used internally
 */
int ServoEasing::getEndMicrosecondsOrUnitsWithTrim() {
	return mEndMicrosecondsOrUnits + mTrimMicrosecondsOrUnits;
}

int ServoEasing::getDeltaMicrosecondsOrUnits() {
	return mDeltaMicrosecondsOrUnits;
}

int ServoEasing::getMillisForCompleteMove() {
	return mMillisForCompleteMove;
}

void ServoEasing::print(Print * aSerial, bool doExtendedOutput) {
	printDynamic(aSerial, doExtendedOutput);
	printStatic(aSerial);
}

/*
 * Prints values which may change from move to move.
 */
void ServoEasing::printDynamic(Print * aSerial, bool doExtendedOutput) {
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
	int tDelta;
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

	aSerial->print(F(" with speed="));
	aSerial->print(mSpeed);

	if (doExtendedOutput) {
		aSerial->print(F(" mMillisAtStartMove="));
		aSerial->print(mMillisAtStartMove);
	}

	aSerial->println();
}

/*
 * Prints values which normally does NOT change from move to move.
 * call with
 */
void ServoEasing::printStatic(Print * aSerial) {

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

#ifndef PROVIDE_ONLY_LINEAR_MOVEMENT
	aSerial->print(F(" easingType=0x"));
	aSerial->print(mEasingType, HEX);
#endif

#if defined(USE_PCA9685_SERVO_EXPANDER)
	aSerial->print(F(" PCA9685I2CAddress=0x"));
	aSerial->print(mPCA9685I2CAddress, HEX);
	aSerial->print(" &Wire=0x");

#  if __PTRDIFF_WIDTH__ == 16
//    aSerial->print((uintptr_t) mI2CClass, HEX); // defined since C++11
	aSerial->print((uint16_t) mI2CClass, HEX);
#  else
	aSerial->print((uint32_t) mI2CClass, HEX);
#  endif
#endif

	aSerial->print(F(" MAX_EASING_SERVOS="));
	aSerial->print(MAX_EASING_SERVOS);

	aSerial->print(" this=0x");
//#ifdef __ets__
#if __PTRDIFF_WIDTH__ == 16
	aSerial->println((uint16_t) this, HEX);
#else
	aSerial->println((uint32_t) this, HEX);
#endif
}

/*
 * Clips the unsigned degree value and handles unsigned underflow.
 * returns 0 if aDegreeToClip >= 218
 * returns 180 if 180 <= aDegreeToClip < 218
 */
int clipDegreeSpecial(uint8_t aDegreeToClip) {
	if (aDegreeToClip) {
		return aDegreeToClip;
	}
	if (aDegreeToClip < 218) {
		return 180;
	}
	return 0;
}

/*
 * Update all servos from list and check if all servos have stopped.
 * Can not call yield() here, since we are in an ISR context here.
 * Defined weak in order to be able to overwrite it.
 */
__attribute__((weak)) void handleServoTimerInterrupt() {
#if defined(USE_PCA9685_SERVO_EXPANDER)
// Otherwise it will hang forever in I2C transfer
	interrupts();
#endif
	if (updateAllServos()) {
		// disable interrupt only if all servos stopped. This enables independent movements of servos with this interrupt handler.
		disableServoEasingInterrupt();
	}
}

/*
 * Timer1 is used for the Arduino Servo library.
 * To have non blocking easing functions its unused channel B is used to generate an interrupt 100 us before the end of the 20 ms Arduino Servo refresh period.
 * This interrupt then updates all servo values for the next refresh period.
 * First interrupt is triggered not directly, but after 20 ms, since we are often called here at the time of the last interrupt of the preceding servo move.
 */
void enableServoEasingInterrupt() {
#if defined(__AVR__)
#  if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#    if defined(USE_PCA9685_SERVO_EXPANDER)
// set timer 1 to 20 ms
	TCCR5A = _BV(WGM11);// FastPWM Mode mode TOP (20 ms) determined by ICR1 - non-inverting Compare Output mode OC1A+OC1B
	TCCR5B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);// set prescaler to 8, FastPWM mode mode bits WGM13 + WGM12
	ICR5 = 40000;// set period to 20 ms
#    endif

	TIFR5 |= _BV(OCF5B);     // clear any pending interrupts;
	TIMSK5 |= _BV(OCIE5B);// enable the output compare B interrupt
	OCR5B = ((clockCyclesPerMicrosecond() * REFRESH_INTERVAL) / 8) - 100;// update values 100 us before the new servo period starts
#  else // defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#    if defined(USE_PCA9685_SERVO_EXPANDER)
//    // set timer 1 to 20 ms
	TCCR1A = _BV(WGM11);// FastPWM Mode mode TOP (20 ms) determined by ICR1 - non-inverting Compare Output mode OC1A+OC1B
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);// set prescaler to 8, FastPWM mode mode bits WGM13 + WGM12
	ICR1 = 40000;// set period to 20 ms
#    endif

	TIFR1 |= _BV(OCF1B);    // clear any pending interrupts;
	TIMSK1 |= _BV(OCIE1B);// enable the output compare B interrupt
	/*
	 * Misuse the Input Capture Noise Canceler Bit as a flag, that signals that interrupts are enabled again.
	 * It is needed if disableServoEasingInterrupt() is suppressed e.g. by an overwritten handleServoTimerInterrupt() function
	 * because the servo interrupt is used to synchronize e.g. NeoPixel updates.
	 */
	TCCR1B |= _BV(ICNC1);
#    ifndef USE_LEIGHTWEIGHT_SERVO_LIB
// update values 100 us before the new servo period starts
	OCR1B = ((clockCyclesPerMicrosecond() * REFRESH_INTERVAL) / 8) - 100;
#    endif
#  endif

#elif defined(ESP8266) || defined(ESP32)
	Timer20ms.attach_ms(20, handleServoTimerInterrupt);

#elif defined(__STM32F1__) // BluePill
	Timer20ms.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
	Timer20ms.setPeriod(20000); // 20000 microsecond period
	Timer20ms.setCompare(TIMER_CH1, Timer20ms.getOverflow() - 1);// trigger interrupt next period
	Timer20ms.attachInterrupt(TIMER_CH1, handleServoTimerInterrupt);
	Timer20ms.refresh();// Set the timer's count to 0 and update the prescaler and overflow values.

#elif defined(__SAM3X8E__)  // Arduino DUE
	pmc_set_writeprotect(false);
	pmc_enable_periph_clk(ID_TC_FOR_20_MS_TIMER);
	NVIC_ClearPendingIRQ(IRQn_FOR_20_MS_TIMER);
	NVIC_EnableIRQ(IRQn_FOR_20_MS_TIMER);

	// MCK/32. Set up the Timer in waveform mode which creates a PWM in UP mode with automatic trigger on RC Compare
	TC_Configure(TC_FOR_20_MS_TIMER, CHANNEL_FOR_20_MS_TIMER, TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);
	TC_SetRC(TC_FOR_20_MS_TIMER, CHANNEL_FOR_20_MS_TIMER, 52500); // 20ms

	TC_Start(TC_FOR_20_MS_TIMER, CHANNEL_FOR_20_MS_TIMER); // Enables the timer clock stopped by TC_Configure() and performs a software reset to start the counting

	// Enable the RC Compare Interrupt
	TC_FOR_20_MS_TIMER->TC_CHANNEL[CHANNEL_FOR_20_MS_TIMER].TC_IER = TC_IER_CPCS;
	// Disable all others.
	TC_FOR_20_MS_TIMER->TC_CHANNEL[CHANNEL_FOR_20_MS_TIMER].TC_IDR = ~TC_IER_CPCS;
#endif
}

void disableServoEasingInterrupt() {
#if defined(__AVR__)
#  if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	TIMSK5 &= ~(_BV(OCIE5B)); // disable the output compare B interrupt
#  else
	TIMSK1 &= ~(_BV(OCIE1B)); // disable the output compare B interrupt
#  endif
#elif defined(ESP8266) || defined(ESP32)
	Timer20ms.detach();
#elif defined(__STM32F1__)
	Timer20ms.setMode(TIMER_CH1, TIMER_DISABLED);
	Timer20ms.detachInterrupt(TIMER_CH1);
#elif defined(__SAM3X8E__)  // Arduino DUE
	NVIC_DisableIRQ(IRQn_FOR_20_MS_TIMER);
#endif
}

/*
 * 60 us for single servo + 160 us per servo if using I2C e.g.for PCA9685 expander at 400000 Hz or + 100 at 800000 Hz
 * 20 us for last interrupt
 * The first servo pulse starts just after this interrupt routine has finished
 */
#if defined(__AVR__)
#  if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
ISR(TIMER5_COMPB_vect) {
	handleServoTimerInterrupt();
}
#  else // defined(__AVR__)
ISR(TIMER1_COMPB_vect) {
#    if defined(MEASURE_TIMING)
	digitalWriteFast(TIMING_PIN, HIGH);
#    endif
	handleServoTimerInterrupt();
#    if defined(MEASURE_TIMING)
	digitalWriteFast(TIMING_PIN, LOW);
#    endif
}
#  endif

#elif defined(__SAM3X8E__)  // Arduino DUE

void HANDLER_FOR_20_MS_TIMER(void) {
#  if defined(MEASURE_TIMING)
	digitalWrite(TIMING_PIN, HIGH);
#  endif
	// clear interrupt
	TC_GetStatus(TC_FOR_20_MS_TIMER, CHANNEL_FOR_20_MS_TIMER); //Clear channel status to fire again the interrupt.
	handleServoTimerInterrupt();
#  if defined(MEASURE_TIMING)
	digitalWrite(TIMING_PIN, LOW);
#  endif
}
#endif // defined(__AVR__)

/************************************
 * ServoEasing list functions
 ***********************************/

#ifndef PROVIDE_ONLY_LINEAR_MOVEMENT
void setEasingTypeForAllServos(uint8_t aEasingType) {
	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		if (sServoArray[tServoIndex] != NULL) {
			sServoArray[tServoIndex]->mEasingType = aEasingType;
		}
	}
}
#endif

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

void printArrayPositions(Print * aSerial) {
//    uint8_t tServoIndex = 0;
	aSerial->print(F("ServoNextPositionArray="));
// AJ 22.05.2019 This does not work with GCC 7.3.0 atmel6.3.1 and -Os
// It drops the tServoIndex < MAX_EASING_SERVOS condition, since  MAX_EASING_SERVOS is equal to the size of sServoArray
// This has only an effect if the whole sServoArray is filled up, i.e we have declared MAX_EASING_SERVOS ServoEasing objects.
//    while (sServoArray[tServoIndex] != NULL && tServoIndex < MAX_EASING_SERVOS) {
//        aSerial->print(sServoNextPositionArray[tServoIndex]);
//        aSerial->print(F(" | "));
//        tServoIndex++;
//    }

// switching conditions cures the bug
//    while (tServoIndex < MAX_EASING_SERVOS && sServoArray[tServoIndex] != NULL) {

// this also does not work
//    for (uint8_t tServoIndex = 0; sServoArray[tServoIndex] != NULL && tServoIndex < MAX_EASING_SERVOS  ; ++tServoIndex) {
//        aSerial->print(sServoNextPositionArray[tServoIndex]);
//        aSerial->print(F(" | "));
//    }
	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		aSerial->print(sServoNextPositionArray[tServoIndex]);
		aSerial->print(F(" | "));
	}
	aSerial->println();
}

void writeAllServos(int aValue) {
	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		if (sServoArray[tServoIndex] != NULL) {
			sServoArray[tServoIndex]->write(aValue);
		}
	}
}

void setSpeedForAllServos(uint16_t aDegreesPerSecond) {
	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		if (sServoArray[tServoIndex] != NULL) {
			sServoArray[tServoIndex]->mSpeed = aDegreesPerSecond;
		}
	}
}

#if defined(va_arg)
/*
 * Sets the sServoNextPositionArray[] of the first aNumberOfServos to the specified values
 */
void setDegreeForAllServos(uint8_t aNumberOfServos, va_list * aDegreeValues) {
	for (uint8_t tServoIndex = 0; tServoIndex < aNumberOfServos; ++tServoIndex) {
		sServoNextPositionArray[tServoIndex] = va_arg(*aDegreeValues, int);
	}
}
#endif

#if defined(va_start)
/*
 * Sets the sServoNextPositionArray[] of the first aNumberOfServos to the specified values
 */
void setDegreeForAllServos(uint8_t aNumberOfServos, ...) {
	va_list aDegreeValues;
	va_start(aDegreeValues, aNumberOfServos);
	setDegreeForAllServos(aNumberOfServos, &aDegreeValues);
	va_end(aDegreeValues);
}
#endif

/*
 * Sets target position using content of sServoNextPositionArray
 * returns false if one servo was still moving
 */
bool setEaseToForAllServos() {
	bool tOneServoIsMoving = false;
	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		if (sServoArray[tServoIndex] != NULL) {
			tOneServoIsMoving = sServoArray[tServoIndex]->setEaseTo(sServoNextPositionArray[tServoIndex],
					sServoArray[tServoIndex]->mSpeed) || tOneServoIsMoving;
		}
	}
	return tOneServoIsMoving;
}

bool setEaseToForAllServos(uint16_t aDegreesPerSecond) {
	bool tOneServoIsMoving = false;
	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		if (sServoArray[tServoIndex] != NULL) {
			tOneServoIsMoving = sServoArray[tServoIndex]->setEaseTo(sServoNextPositionArray[tServoIndex], aDegreesPerSecond)
					|| tOneServoIsMoving;
		}
	}
	return tOneServoIsMoving;
}

bool setEaseToDForAllServos(uint16_t aMillisForMove) {
	bool tOneServoIsMoving = false;
	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		if (sServoArray[tServoIndex] != NULL) {
			tOneServoIsMoving = sServoArray[tServoIndex]->setEaseToD(sServoNextPositionArray[tServoIndex], aMillisForMove)
					|| tOneServoIsMoving;
		}
	}
	return tOneServoIsMoving;
}

bool isOneServoMoving() {
	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		if (sServoArray[tServoIndex] != NULL && sServoArray[tServoIndex]->mServoMoves) {
			return true;
		}
	}
	return false;
}

void stopAllServos() {
	void disableServoEasingInterrupt();
	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		if (sServoArray[tServoIndex] != NULL) {
			sServoArray[tServoIndex]->mServoMoves = false;
		}
	}
}

/*
 * returns true if all Servos reached endAngle / stopped
 */
bool updateAllServos() {
	bool tAllServosStopped = true;
	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		if (sServoArray[tServoIndex] != NULL) {
			tAllServosStopped = sServoArray[tServoIndex]->update() && tAllServosStopped;
		}
	}
#if defined(PRINT_FOR_SERIAL_PLOTTER)
// End of one data set
	Serial.println();
#endif
	return tAllServosStopped;
}

void updateAndWaitForAllServosToStop() {
	do {
		// First do the delay, then check for update, since we are likely called directly after start and there is nothing to move yet
		delay(REFRESH_INTERVAL / 1000); // 20 ms - REFRESH_INTERVAL is in Microseconds
	} while (!updateAllServos());
}

/*
 * returns true if all Servos reached endAngle / stopped
 */
bool delayAndUpdateAndWaitForAllServosToStop(unsigned long aMillisDelay, bool aTerminateDelayIfAllServosStopped) {
	while (true) {
		// First do the delay, then check for update, since we are likely called directly after start and there is nothing to move yet
		if (aMillisDelay > REFRESH_INTERVAL / 1000) {
			aMillisDelay -= REFRESH_INTERVAL / 1000;
			delay(REFRESH_INTERVAL / 1000); // 20 ms - REFRESH_INTERVAL is in Microseconds
			if (updateAllServos() && aTerminateDelayIfAllServosStopped) {
				// terminate delay here and return
				return true;
			}
		} else {
			delay(aMillisDelay);
			return updateAllServos();
		}
	}
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
	uint16_t tMaxMillisForCompleteMove = 0;
	uint32_t tMillisAtStartMove = 0;

	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		if (sServoArray[tServoIndex] != NULL && sServoArray[tServoIndex]->mServoMoves) {
			//process servos which really moves
			tMillisAtStartMove = sServoArray[tServoIndex]->mMillisAtStartMove;
			if (sServoArray[tServoIndex]->mMillisForCompleteMove > tMaxMillisForCompleteMove) {
				tMaxMillisForCompleteMove = sServoArray[tServoIndex]->mMillisForCompleteMove;
			}
		}
	}

#if defined(TRACE)
	Serial.print(F("Number of servos="));
	Serial.print(sServoArrayMaxIndex);
	Serial.print(F(" MillisAtStartMove="));
	Serial.print(tMillisAtStartMove);
	Serial.print(F(" MaxMillisForCompleteMove="));
	Serial.println(tMaxMillisForCompleteMove);
#endif

	/*
	 * Set maximum duration and start time to all servos
	 * Synchronize start time to avoid race conditions at the end of movement
	 */
	for (uint8_t tServoIndex = 0; tServoIndex <= sServoArrayMaxIndex; ++tServoIndex) {
		if (sServoArray[tServoIndex] != NULL && sServoArray[tServoIndex]->mServoMoves) {
			sServoArray[tServoIndex]->mMillisAtStartMove = tMillisAtStartMove;
			sServoArray[tServoIndex]->mMillisForCompleteMove = tMaxMillisForCompleteMove;
		}
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
	return sin(13 * M_PI_2 * aPercentageOfCompletion) * pow(2, 10 * (aPercentageOfCompletion - 1));
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
