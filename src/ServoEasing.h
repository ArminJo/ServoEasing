/*
 * ServoEasing.h
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
 *
 */

#ifndef SERVOEASING_H_
#define SERVOEASING_H_

/*  *****************************************************************************************************************************
 *  To access the library files from your sketch, you have to first use `Sketch/Show Sketch Folder (Ctrl+K)` in the Arduino IDE.
 *  Then navigate to the parallel `libraries` folder and select the library you want to access.
 *  The library files itself are located in the `src` sub-directory.
 *  If you did not yet store the example as your own sketch, then with Ctrl+K you are instantly in the right library folder.
 *  *****************************************************************************************************************************/
/*
 * For use with e.g. the Adafruit PCA9685 16-Channel Servo Driver aOffUnits.
 */
//#define USE_PCA9685_SERVO_EXPANDER
/*
 * If you have only one or two servos, then you can save program space by defining symbol `USE_LEIGHTWEIGHT_SERVO_LIB`.
 * This saves 742 bytes FLASH and 42 bytes RAM.
 * Using Lightweight Servo library (or PCA9685 servo expander) makes the servo pulse generating immune
 * to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
 * If not using the Arduino IDE take care that Arduino Servo library sources are not compiled / included in the project.
 */
//#define USE_LEIGHTWEIGHT_SERVO_LIB
#if ( defined(ESP8266) || defined(ESP32) || defined(__STM32F1__)) && defined(USE_LEIGHTWEIGHT_SERVO_LIB)
#error "No Lightweight Servo Library available (and needed) for ESP boards"
#endif

#if defined(USE_PCA9685_SERVO_EXPANDER) && defined(USE_LEIGHTWEIGHT_SERVO_LIB)
#error "Please define only one of the symbols USE_PCA9685_SERVO_EXPANDER or USE_LEIGHTWEIGHT_SERVO_LIB"
#endif

/*****************************************************************************************
 * Important definition of MAX_EASING_SERVOS !!!
 * If this value is smaller than the amount of servos declared,
 * attach() will return error and detach() will not work as expected.
 * As well as all *AllServos*() functions and isOneServoMoving()
 * won't work correctly! (they will only work for the first MAX_EASING_SERVOS -2 servos)
 *
 * If you do not need these functions, you may define MAX_EASING_SERVOS as 1
 ****************************************************************************************/
#if defined(USE_PCA9685_SERVO_EXPANDER) || defined(USE_LEIGHTWEIGHT_SERVO_LIB)

#if defined(USE_PCA9685_SERVO_EXPANDER)
#include <Wire.h>
#ifndef MAX_EASING_SERVOS
#define MAX_EASING_SERVOS 16 // One PCA9685 has 16 outputs. You must MODIFY this, if you have more than one PCA9685 attached!
#endif

#elif defined(USE_LEIGHTWEIGHT_SERVO_LIB)
#include "LightweightServo.h"
#ifndef MAX_EASING_SERVOS
#define MAX_EASING_SERVOS 2 // default value for UNO etc.
#endif
#endif

#else  // defined(USE_PCA9685_SERVO_EXPANDER) || defined(USE_LEIGHTWEIGHT_SERVO_LIB)
#if defined(ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

#ifndef MAX_EASING_SERVOS
#ifdef MAX_SERVOS
#define MAX_EASING_SERVOS MAX_SERVOS // =12 use default value from Servo.h for UNO etc.
#else
#define MAX_EASING_SERVOS 12 // just take default value from Servo.h for UNO etc.
#endif
#endif
#endif // defined(USE_PCA9685_SERVO_EXPANDER) || defined(USE_LEIGHTWEIGHT_SERVO_LIB)

#if ! defined(REFRESH_INTERVAL)
#define REFRESH_INTERVAL 20000   // (from Servo.h)
#endif
#if ! defined(INVALID_SERVO)
#define INVALID_SERVO    255     // flag indicating an invalid servo index (from Servo.h)
#endif

/*
 * Define `KEEP_SERVO_EASING_LIBRARY_SMALL` if space (1850 Bytes) matters.
 * It disables the SINE, CIRCULAR, BACK, ELASTIC and BOUNCE easings.
 * The saving comes mainly from avoiding the sin() cos() sqrt() and pow() library functions in this code.
 * If you need only one complex easing function and want to save space,
 * you can specify it any time as a user functions. See EaseQuadraticInQuarticOut() function in AsymmetricEasing example line 195.
 */
//#define KEEP_SERVO_EASING_LIBRARY_SMALL
//
/*
 * If you need only the linear movement you may define `PROVIDE_ONLY_LINEAR_MOVEMENT`. This saves additional 1540 Bytes FLASH.
 */
//#define PROVIDE_ONLY_LINEAR_MOVEMENT
//
// Enable this if you want to measure timing by toggling pin12 on an arduino
//#define MEASURE_TIMING
//
// Enable this to generate output for Arduino Serial Plotter (Ctrl-Shift-L)
//#define PRINT_FOR_SERIAL_PLOTTER
//
/*
 * Enable this to see information on each call.
 * Since there should be no library which uses Serial, enable TRACE only for development purposes.
 */
//#define TRACE
//#define DEBUG
// Propagate debug level
#ifdef TRACE
#define DEBUG
#endif
#ifdef DEBUG
#define INFO
#endif
#ifdef INFO
#define WARN
#endif
#ifdef WARN
#define ERROR
#endif

#define VERSION_SERVO_EASING 1.4.3

/*
 * Version 1.4.3 - 12/2019
 * - Improved detach() handling.
 * - Initialize mSpeed explicitly to 0 in constructor. On an ESP8266 it was NOT initialized to 0 :-(.
 *
 * Version 1.4.2 - 11/2019
 * - Improved INVALID_SERVO handling.
 * - Speed 0 (not initialized) handling.
 * - Fixed bug in ThreeServos example.
 *
 * Version 1.4.1 - 11/2019
 * - Improved documentation and definitions for continuous rotating servo. Thanks to Eebel!
 * - Improved support and documentation for generating Arduino Serial Plotter output.
 * - Support of STM32F1 / BluePill boards.
 *
 * Version 1.4.0 - 11/2019
 * - setTrim has additional parameter 'doWrite' which is default 'false' in contrast to older versions, where a write was always performed.
 * - New attach( aPin,  aMicrosecondsForServoLowDegree,  aMicrosecondsForServoHighDegree,  aServoLowDegree,  aServoHighDegree) function for arbitrary mapping of servo degree to servo pulse width.
 * - Order of Servos in 'sServoArray[]' now depends from order of calling attach() and not from order of declaration.
 * - New example for continuous rotating servo.
 * - Support for multiple PCA9685 expander.
 *
 * Version 1.3.1 - 6/2019
 * - Added detach() function.
 *
 * Version 1.3.0 - 6/2019
 * - Added ESP32 support by using ESP32Servo.h and Ticker.h instead of Servo.h timer interrupts.
 * - Changed degree parameter and values from uint8_t to integer to support operating a servo from -90 to + 90 degree with 90 degree trim.
 * - RobotArmControl + QuadrupedControl examples refactored.
 * - Extended SpeedTest example. Now also able to change the width of the refresh period.
 * - Changed "while" to "for" loops to avoid a gcc 7.3.0 atmel6.3.1 bug.
 *
 * Version 1.2 - 5/2019
 * - Added ESP8266 support by using Ticker instead of timer interrupts for ESP.
 * - AsymetricEasing example overhauled.
 *
 * Version 1.1 - 4/2019
 * - corrected sine, circular, back and elastic IN functions.
 * - easeTo() and write() store their degree parameter now also in sServoNextPositionArray.
 * - added setSpeed(), getSpeed(), setSpeedForAllServos().
 * - added easeTo(uint8_t aDegree) and setEaseTo(uint8_t aDegree).
 * - added setEaseToForAllServos(), setEaseToForAllServosSynchronizeAndStartInterrupt(), synchronizeAndEaseToArrayPositions().
 * - added getEndMicrosecondsOrUnits(), getDeltaMicrosecondsOrUnits().
 * - added setDegreeForAllServos(uint8_t aNumberOfValues, va_list * aDegreeValues),setDegreeForAllServos(uint8_t aNumberOfValues, ...).
 * - added compile switch PROVIDE_ONLY_LINEAR_MOVEMENT to save additional 1500 bytes FLASH if enabled.
 * - added convenience function clipDegreeSpecial().
 */

#define DEFAULT_MICROSECONDS_FOR_0_DEGREE 544
#define DEFAULT_MICROSECONDS_FOR_180_DEGREE 2400

#define DEFAULT_PCA9685_UNITS_FOR_0_DEGREE  111 // 111.411 = 544 us
#define DEFAULT_PCA9685_UNITS_FOR_180_DEGREE 491 // 491.52 = 2400 us

/*
 * Definitions for continuous rotating servo - Values are taken from the Parallax Continuous Rotation Servo manual
 * and rely on a stop value of exactly 1500 microseconds.
 * If the stop value of your servo is NOT exactly 1500 microseconds, you must change the value of MICROSECONDS_FOR_ROTATING_SERVO_STOP.
 * My modified MG90 servo has 1630 and 1400 as max.
 *
 * Use attach(PIN, MICROSECONDS_FOR_ROTATING_SERVO_CLOCKWISE_MAX, MICROSECONDS_FOR_ROTATING_SERVO_COUNTER_CLOCKWISE_MAX, 100, -100);
 * Use write(100) for maximum clockwise and write(-100) for maximum counter clockwise rotation.
 */
#define MICROSECONDS_FOR_ROTATING_SERVO_STOP 1500 // Change this value to your servos real stop value

/*
 * Definitions here are only for convenience. You may freely modify them.
 */
#define MICROSECONDS_FOR_ROTATING_SERVO_CLOCKWISE_MAX (MICROSECONDS_FOR_ROTATING_SERVO_STOP - 200)
#define MICROSECONDS_FOR_ROTATING_SERVO_CLOCKWISE_HALF (MICROSECONDS_FOR_ROTATING_SERVO_STOP - 100)
#define MICROSECONDS_FOR_ROTATING_SERVO_CLOCKWISE_QUARTER (MICROSECONDS_FOR_ROTATING_SERVO_STOP - 50)
#define MICROSECONDS_FOR_ROTATING_SERVO_COUNTER_CLOCKWISE_MAX (MICROSECONDS_FOR_ROTATING_SERVO_STOP + 200)
#define MICROSECONDS_FOR_ROTATING_SERVO_COUNTER_CLOCKWISE_HALF (MICROSECONDS_FOR_ROTATING_SERVO_STOP + 100)
#define MICROSECONDS_FOR_ROTATING_SERVO_COUNTER_CLOCKWISE_QUARTER (MICROSECONDS_FOR_ROTATING_SERVO_STOP + 50)

/*
 * The different easing functions:
 *
 * In order to reuse the IN functions for OUT and IN_OUT functions, the following call and result conversions are used internally.
 * 1. Using IN function direct: Call with PercentageOfCompletion | 0.0 to 1.0. Result is from 0.0 to 1.0
 * 2. Using IN function to generate OUT function: Call with (1 - PercentageOfCompletion) | 1.0 to 0.0. Result = (1 - result)
 * 3. Using IN function to generate IN_OUT function:
 *      In the first half, call with (2 * PercentageOfCompletion) | 0.0 to 1.0. Result = (0.5 * result)
 *      In the second half, call with (2 - (2 * PercentageOfCompletion)) | 1.0 to 0.0. Result = ( 1- (0.5 * result))
 * 4. Using IN function to generate bouncing_OUT_IN / mirrored_OUT function, which return to start point (like the upper half of a sine):
 *      In the first half, call with (1 - (2 * PercentageOfCompletion)) | 1.0 to 0.0. Result = (1 - result) -> call OUT function 2 times faster.
 *      In the second half, call with ((2 * PercentageOfCompletion) - 1) | 0.0 to 1.0. Result = (1- result) -> call OUT function 2 times faster and backwards.
 */

/*
 * Values for provided EaseTypes
 * The call style is coded in the upper 3 bits
 */
#define CALL_STYLE_DIRECT           0x00 // == IN
#define CALL_STYLE_OUT              0x20
#define CALL_STYLE_IN_OUT           0x40
#define CALL_STYLE_BOUNCING_OUT_IN  0x60 // Bouncing has double movement, so double time (half speed) is taken for this modes

#define CALL_STYLE_MASK             0xE0 // for future extensions
#define EASE_TYPE_MASK          0x0F

#define EASE_LINEAR             0x00 // No bouncing available

#define EASE_QUADRATIC_IN       0x01
#define EASE_QUADRATIC_OUT      0x21
#define EASE_QUADRATIC_IN_OUT   0x41
#define EASE_QUADRATIC_BOUNCING 0x61

#define EASE_CUBIC_IN           0x02
#define EASE_CUBIC_OUT          0x22
#define EASE_CUBIC_IN_OUT       0x42
#define EASE_CUBIC_BOUNCING     0x62

#define EASE_QUARTIC_IN         0x03
#define EASE_QUARTIC_OUT        0x23
#define EASE_QUARTIC_IN_OUT     0x43
#define EASE_QUARTIC_BOUNCING   0x63

#ifndef KEEP_SERVO_EASING_LIBRARY_SMALL
#define EASE_SINE_IN            0x08
#define EASE_SINE_OUT           0x28
#define EASE_SINE_IN_OUT        0x48
#define EASE_SINE_BOUNCING      0x68

#define EASE_CIRCULAR_IN        0x09
#define EASE_CIRCULAR_OUT       0x29
#define EASE_CIRCULAR_IN_OUT    0x49
#define EASE_CIRCULAR_BOUNCING  0x69

#define EASE_BACK_IN            0x0A
#define EASE_BACK_OUT           0x2A
#define EASE_BACK_IN_OUT        0x4A
#define EASE_BACK_BOUNCING      0x6A

#define EASE_ELASTIC_IN         0x0B
#define EASE_ELASTIC_OUT        0x2B
#define EASE_ELASTIC_IN_OUT     0x4B
#define EASE_ELASTIC_BOUNCING   0x6B

// the coded function is an OUT function
#define EASE_BOUNCE_IN          0x2C // call OUT function inverse
#define EASE_BOUNCE_OUT         0x0C // call OUT function direct
#endif

#define EASE_USER_DIRECT        0x0F
#define EASE_USER_OUT           0x2F
#define EASE_USER_IN_OUT        0x4F
#define EASE_USER_BOUNCING      0x6F
#define EASE_FUNCTION_DEGREE_INDICATOR_OFFSET 256 // Offset to decide if the user function returns degree instead of 0.0 to 1.0. => returns 256 for 0 degree.

// some PCA9685 specific constants
#define PCA9685_GENERAL_CALL_ADDRESS 0x00
#define PCA9685_SOFTWARE_RESET      6
#define PCA9685_DEFAULT_ADDRESS     0x40
#define PCA9685_MAX_CHANNELS        16 // 16 PWM channels on each PCA9685 expansion module
#define PCA9685_MODE1_REGISTER      0x0
#define PCA9685_AUTOINCREMENT       5
#define PCA9685_SLEEP               4
#define PCA9685_FIRST_PWM_REGISTER  0x06
#define PCA9685_PRESCALE_REGISTER   0xFE

#define PCA9685_PRESCALER_FOR_20_MS ((25000000L /(4096L * 50))-1) // = 121 / 0x79 at 50 Hz

class ServoEasing
#if not defined(USE_LEIGHTWEIGHT_SERVO_LIB) && not defined(USE_PCA9685_SERVO_EXPANDER)
        : public Servo
#endif
{
public:

#if defined(USE_PCA9685_SERVO_EXPANDER)
#if defined(ARDUINO_SAM_DUE)
    ServoEasing(uint8_t aPCA9685I2CAddress = PCA9685_DEFAULT_ADDRESS, TwoWire *aI2CClass = &Wire1);
#else
    ServoEasing(uint8_t aPCA9685I2CAddress = PCA9685_DEFAULT_ADDRESS, TwoWire *aI2CClass = &Wire);
#endif
    void PCA9685Reset();
    void PCA9685Init();
    void I2CWriteByte(uint8_t aAddress, uint8_t aData);
    void setPWM(uint16_t aOffUnits);
    void setPWM(uint16_t aPWMOnValueAsUnits, uint16_t aPWMOffValueAsUnits);
    // main mapping function for us to PCA9685 Units (20000/4096 = 4.88 us)
    int MicrosecondsToPCA9685Units(int aMicroseconds);
#else
    ServoEasing();
#endif
    uint8_t attach(int aPin);
    // Here no units accepted, only microseconds!
    uint8_t attach(int aPin, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree);
    uint8_t attach(int aPin, int aMicrosecondsForServoLowDegree, int aMicrosecondsForServoHighDegree, int aServoLowDegree,
            int aServoHighDegree);

    void detach();
    void setReverseOperation(bool aOperateServoReverse);  // You should call it before using setTrim

    void setTrim(int aTrimDegrees, bool aDoWrite = false);
    void setTrimMicrosecondsOrUnits(int aTrimMicrosecondsOrUnits, bool aDoWrite = false);

#ifndef PROVIDE_ONLY_LINEAR_MOVEMENT
    void setEasingType(uint8_t aEasingType);
    uint8_t getEasingType();

    void registerUserEaseInFunction(float (*aUserEaseInFunction)(float aPercentageOfCompletion));

    float callEasingFunction(float aPercentageOfCompletion);    // used in update()
#endif

    void write(int aValue);                         // Apply trim and reverse to the value and write it direct to the Servo library.
    void writeMicrosecondsOrUnits(int aValue);

    void setSpeed(uint16_t aDegreesPerSecond);                  // This speed is taken if no speed argument is given.
    uint16_t getSpeed();
    void easeTo(int aDegree);                                   // blocking move to new position using mLastSpeed
    void easeTo(int aDegree, uint16_t aDegreesPerSecond);       // blocking move to new position using speed
    void easeToD(int aDegree, uint16_t aMillisForMove);         // blocking move to new position using duration

    bool setEaseTo(int aDegree);                                // shortcut for startEaseTo(..,..,false)
    bool setEaseTo(int aDegree, uint16_t aDegreesPerSecond);    // shortcut for startEaseTo(..,..,false)
    bool startEaseTo(int aDegree);                              // shortcut for startEaseTo(aDegree, mSpeed, true)
    bool startEaseTo(int aDegree, uint16_t aDegreesPerSecond, bool aStartUpdateByInterrupt = true);
    bool setEaseToD(int aDegree, uint16_t aDegreesPerSecond);   // shortcut for startEaseToD(..,..,false)
    bool startEaseToD(int aDegree, uint16_t aMillisForMove, bool aStartUpdateByInterrupt = true);
    bool update();

    int getCurrentAngle();
    int getEndMicrosecondsOrUnits();
    int getEndMicrosecondsOrUnitsWithTrim();
    int getDeltaMicrosecondsOrUnits();
    int getMillisForCompleteMove();
    bool isMoving();
    bool isMovingAndCallYield();

    int MicrosecondsOrUnitsToDegree(int aMicrosecondsOrUnits);
    int DegreeToMicrosecondsOrUnits(int aDegree);
    int DegreeToMicrosecondsOrUnitsWithTrimAndReverse(int aDegree);

    void synchronizeServosAndStartInterrupt(bool doUpdateByInterrupt);

    void print(Stream * aSerial, bool doExtendedOutput = true); // Print dynamic and static info
    void printDynamic(Stream * aSerial, bool doExtendedOutput = true);
    void printStatic(Stream * aSerial);

    /*
     * Internally only microseconds (or units (= 4.88 us) if using PCA9685 expander) and not degree are used to speed up things.
     * Other expander or libraries can therefore easily be added.
     */
    volatile int mCurrentMicrosecondsOrUnits; // set by write() and writeMicrosecondsOrUnits(). Needed as start for next move and to avoid unnecessary writes.
    int mStartMicrosecondsOrUnits;  // used with millisAtStartMove to compute currentMicrosecondsOrUnits
    int mEndMicrosecondsOrUnits;    // used once as last value just if movement was finished
    int mDeltaMicrosecondsOrUnits;   // end - start

    /*
     * max speed is 450 degree/sec for SG90 and 540 degree/second for MG90 servos -> see speedTest.cpp
     */
    uint16_t mSpeed; // in DegreesPerSecond only set by setSpeed(int16_t aSpeed);

#ifndef PROVIDE_ONLY_LINEAR_MOVEMENT
    uint8_t mEasingType; // EASE_LINEAR, EASE_QUADRATIC_IN_OUT, EASE_CUBIC_IN_OUT, EASE_QUARTIC_IN_OUT

    float (*mUserEaseInFunction)(float aPercentageOfCompletion);
#endif

    volatile bool mServoMoves;

#if defined(USE_PCA9685_SERVO_EXPANDER)
    uint8_t mPCA9685I2CAddress;
    TwoWire * mI2CClass;
#endif
    uint8_t mServoPin; // pin number or NO_SERVO_ATTACHED_PIN_NUMBER - at least needed for Lightweight Servo Lib

    uint8_t mServoIndex; // Index in sServoArray or INVALID_SERVO if error while attach() or if detached

    uint32_t mMillisAtStartMove;
    uint16_t mMillisForCompleteMove;

    /*
     * Reverse means, that values for 180 and 0 degrees are swapped by: aValue = mServo180DegreeMicrosecondsOrUnits - (aValue - mServo0DegreeMicrosecondsOrUnits)
     * Be careful, if you specify different end values, it may not behave, as you expect.
     * For this case better use the attach function with 5 parameter.
     */
    bool mOperateServoReverse; // true -> direction is reversed
    int mTrimMicrosecondsOrUnits; // This value is always added to the degree/units/microseconds value requested

    int mServo0DegreeMicrosecondsOrUnits;
    int mServo180DegreeMicrosecondsOrUnits;
};

/*
 * Array of all servos to enable synchronized movings
 * Servos are inserted in the order, in which they are attached
 * I use an fixed array and not a list, since accessing an array is much easier and faster.
 * Using an dynamic array may be possible, but in this case we must first malloc(), then memcpy() and then free(), which leads to heap fragmentation.
 */
extern uint8_t sServoCounter;
extern ServoEasing * sServoArray[MAX_EASING_SERVOS];
extern int sServoNextPositionArray[MAX_EASING_SERVOS]; // use int since we want to support negative values

/*
 * Functions working on all servos in the list
 */
void setSpeedForAllServos(uint16_t aDegreesPerSecond);
#if defined(va_arg)
void setDegreeForAllServos(uint8_t aNumberOfValues, va_list * aDegreeValues);
#endif
#if defined(va_start)
void setDegreeForAllServos(uint8_t aNumberOfValues, ...);
#endif

bool setEaseToForAllServos();
bool setEaseToForAllServos(uint16_t aDegreesPerSecond);
bool setEaseToDForAllServos(uint16_t aMillisForMove);
void setEaseToForAllServosSynchronizeAndStartInterrupt();
void setEaseToForAllServosSynchronizeAndStartInterrupt(uint16_t aDegreesPerSecond);
void synchronizeAndEaseToArrayPositions();
void synchronizeAndEaseToArrayPositions(uint16_t aDegreesPerSecond);

void printArrayPositions(Stream * aSerial);
bool isOneServoMoving();
void stopAllServos();
bool updateAllServos();
void synchronizeAllServosAndStartInterrupt(bool aStartUpdateByInterrupt = true);

#ifndef PROVIDE_ONLY_LINEAR_MOVEMENT
void setEasingTypeForAllServos(uint8_t aEasingType);
#endif

// blocking wait functions
void updateAndWaitForAllServosToStop();
void synchronizeAllServosStartAndWaitForAllServosToStop();

void enableServoEasingInterrupt();
void disableServoEasingInterrupt();

int clipDegreeSpecial(uint8_t aDegreeToClip);

/*
 * Included easing functions
 */

float QuadraticEaseIn(float aPercentageOfCompletion);
float CubicEaseIn(float aPercentageOfCompletion);
float QuarticEaseIn(float aPercentageOfCompletion);

float SineEaseIn(float aPercentageOfCompletion);
float CircularEaseIn(float aPercentageOfCompletion);
float BackEaseIn(float aPercentageOfCompletion);
float ElasticEaseIn(float aPercentageOfCompletion);

// Non symmetric functions
float EaseOutBounce(float aPercentageOfCompletion);

extern float (*sEaseFunctionArray[])(float aPercentageOfCompletion);

#endif /* SERVOEASING_H_ */
