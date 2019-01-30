/*
 * ServoEasing.h
 *
 *  Created on: 12.01.2019
 *      Author: Armin
 */

#ifndef SERVOEASING_H_
#define SERVOEASING_H_

/*
 * If you have only one or two servos, then you can save program space by defining symbol `USE_LEIGHTWEIGHT_SERVO_LIB`.
 * This saves 742 bytes FLASH and 42 bytes RAM.
 * Using Lightweight Servo Library makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
 * In the Arduino IDE use `Sketch/Show Sketch Folder ( Ctrl+K)` then navigate to the `src` folder, open ServoEasing.h and outcomment line 20.
 * If not using the Arduino IDE take care that Arduino Servo library sources are not compiled / included in the project.
 *
 */

//#define USE_LEIGHTWEIGHT_SERVO_LIB
#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
#include "LightweightServo.h"
#define REFRESH_INTERVAL 20000
#else
#include <Servo.h>
#endif

/*
 * Define `KEEP_LIBRARY_SMALL` if space (2100 Bytes) matters.
 * The saving comes mainly from avoiding the sin() cos() sqrt() and pow() library functions in this code.
 * If you need one complex easing function, you can specify it any time as a user functions. See AsymmetricEasing example line 58.
 */

//#define KEEP_LIBRARY_SMALL
//
/*
 * Enable this to see information on each call.
 * Since there should be no library which uses Serial, enable it only for debugging purposes.
 */
//#define DEBUG
//
#define DEFAULT_MICROSECONDS_FOR_0_DEGREE 544
#define DEFAULT_MICROSECONDS_FOR_180_DEGREE 2400

#define SG_90_MICROSECONDS_FOR_0_DEGREE 620
#define SG_90_MICROSECONDS_FOR_180_DEGREE 2400

// Values for EaseType
#define EASE_LINEAR      0
#define EASE_QUADRATIC   1
#define EASE_CUBIC       2
#define EASE_QUARTIC     3
#ifndef KEEP_LIBRARY_SMALL
#define EASE_SINE        4
#define EASE_CIRCULAR    5
#define EASE_BACK        6
#define EASE_ELASTIC     7
#define EASE_BOUNCE      8
#endif
#define EASE_USER       20

struct ServoMove {
    uint8_t Degree;
    uint8_t EasingType;
    union {
        uint16_t DegreesPerSecond;
        uint16_t MillisForMove;
    } SpeedOrDuration;
};

class ServoEasing
#ifndef USE_LEIGHTWEIGHT_SERVO_LIB
        : public Servo
#endif
{
public:
    ServoEasing();
    uint8_t attach(int aPin);
    uint8_t attach(int aPin, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree);
    void setTrim(int8_t aTrim);
    void setTrimMicroseconds(int16_t aTrimMicroseconds);
    void setEasingType(uint8_t aEasingType);
    void registerSimpleUserEaseInFunction(float (*aSimpleUserEaseInFunction)(float aPercentageOfCompletion));
    void registerUserEaseInOutFunction(float (*aUserEaseInOutFunction)(float aPercentageOfCompletion, bool aInFirstHalf));

    void setSynchronizedServo(ServoEasing * aSynchronizedServo);

    void write(int aValue); // write value direct to servo
    void writeSynchronized(int aValue, int aSynchronizedServoValue); // write value direct to servo and its synchronized one
    void writeMicroseconds(int aValue);
    void easeTo(uint8_t aDegree, uint16_t aDegreesPerSecond); // blocking move to new position using speed
    void easeToD(uint8_t aDegree, uint16_t aMillisForMove); // blocking move to new position using duration
    void easeTo(struct ServoMove aMovement); // blocking move to new position
    void easeToSynchronized(uint8_t aDegree, uint8_t aSynchronizedServoDegree, uint16_t aDegreesPerSecond);
    void easeToDSynchronized(uint8_t aDegree, uint8_t aSynchronizedServoDegree, uint16_t aMillisForMove);
    bool startEaseTo(uint8_t aDegree, uint16_t aDegreesPerSecond, bool doUpdateByInterrupt = false);
    bool startEaseToD(uint8_t aDegree, uint16_t aMillisForMove, bool doUpdateByInterrupt = false);
    bool startEaseTo(struct ServoMove aMovement, bool doUpdateByInterrupt);
    bool startEaseToSynchronized(uint8_t aDegree, uint8_t aSynchronizedServoDegree, uint16_t aDegreesPerSecond,
            bool doUpdateByInterrupt = false);
    bool startEaseToDSynchronized(uint8_t aDegree, uint8_t aSynchronizedServoDegree, uint16_t aMillisForMove,
            bool doUpdateByInterrupt = false);
    bool update();
    bool updateSynchronized();

    uint8_t getCurrentAngle();
    uint16_t getMillisForCompleteMove();
    bool isMoving();

    uint8_t MicrosecondsToDegree(uint16_t aMicroseconds);
    uint16_t DegreeToMicroseconds(uint8_t aDegree);

    void synchronizeServosAndStartInterrupt(bool doUpdateByInterrupt);

#ifdef DEBUG
    void print(); // Print dynamic and static info - only available for debug purposes
    void printDynamic();// only available for debug purposes
    void printStatic();// only available for debug purposes
#endif

#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
    uint8_t ServoPin; // pin number to decide when to use Leightweight Servo Lib
#endif

    ServoEasing * SynchronizedServo; // The other servo to move synchronized - for more server see description in ServoEasing.cpp

    volatile uint16_t currentMicroseconds; // set by write() and writeMicroseconds(). Needed for next move and to avoid unnecessary writes.
    uint16_t startMicroseconds;  // used with millisAtStartMove to compute currentMicroseconds
    uint16_t endMicroseconds;
    uint16_t trimMicroseconds; // this value will be added for each writeMicroseconds()
    int16_t deltaMicroseconds; // end - start

    uint8_t EasingType; // EASE_LINEAR, EASE_QUADRATIC, EASE_CUBIC, EASE_QUARTIC

    volatile bool servoMoves;

    /*
     * The user only has to specify the IN function for the first half of movement,
     * since for OUT movement we can use the same values for mirroring the movement.
     * aPercentageOfCompletion has range from 0 to 0.5
     * The result is expected from 0 to 0.5
     */
    float (*SimpleUserEaseInFunction)(float aPercentageOfCompletion);

    /*
     * Function for non mirrored moves. The user only can specify independent simple IN and OUT functions.
     * The result of the IN function is mirrored in the second half, so all simple IN functions can directly be used as OUT functions.
     * aPercentageOfCompletion has range from 0 to 0.5
     * The result is expected from 0 to 0.5 if no overshoot intended
     * EaseOutBounce() is an example of such a function as well as the functions in the AsymmetricEasing example.
     */
    float (*UserEaseInOutFunction)(float aPercentageOfCompletion, bool aInFirstHalf);

    uint32_t millisAtStartMove;
    uint16_t millisForCompleteMove;

// since the values in the Servo library are private we need a copy here :-(
    uint16_t MicrosecondsForServo0Degree;
    uint16_t MicrosecondsForServo180Degree;
};

void enableServoEasingInterrupt();
void disableServoEasingInterrupt();

/*
 * Sample easing functions
 */
float QuadraticEaseIn(float aPercentageOfCompletion);
float CubicEaseIn(float aPercentageOfCompletion);
float QuarticEaseIn(float aPercentageOfCompletion);

float SineEaseIn(float aPercentageOfCompletion);
float CircularEaseIn(float aPercentageOfCompletion);
float BackEaseIn(float aPercentageOfCompletion);
float ElasticEaseIn(float aPercentageOfCompletion);
// A non symmetric function
float EaseOutBounce(float aPercentageOfCompletion, bool aInFirstHalf);

#endif /* SERVOEASING_H_ */
