/*
 * SmoothServo.h
 *
 *  Created on: 12.01.2019
 *      Author: Armin
 */

#ifndef SMOOTHSERVO_H_
#define SMOOTHSERVO_H_

#include <Servo.h>

#define DEFAULT_MICROSECONDS_FOR_0_DEGREE 544
#define DEFAULT_MICROSECONDS_FOR_180_DEGREE 2400

class SmoothServo: public Servo {
public:
    SmoothServo();
    uint8_t attach(int aPin);
    uint8_t attach(int aPin, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree);

    void setSyncronizedServo(SmoothServo * aSyncronizedServo);

    void write(int aValue); // write value direct to servo
    void writeSyncronized(int aValue, int aSyncronizedServoValue);
    void writeMicroseconds(int aValue);
    void moveTo(uint8_t aValue, int aDegreesPerSecond); // blocking move to new position
    void moveToSyncronized(uint8_t aDegree, int aSyncronizedServoDegree, int aDegreesPerSecond);
    bool startMoveTo(uint8_t aDegree, int aDegreesPerSecond, bool doUpdateByInterrupt = false);
    bool startMoveToSyncronized(uint8_t aDegree, int aSyncronizedServoDegree, int aDegreesPerSecond, bool doUpdateByInterrupt = false);
    bool update();
    bool updateSyncronized();
    bool isMoving();

    uint8_t MicrosecondsToDegree(uint16_t aMicroseconds);
    uint16_t DegreeToMicroseconds(uint8_t aDegree);

//    void print(); only available for debug purposes

    SmoothServo * SyncronizedServo; // The other servo to move synchronized - for more server see description in SmoothServo.cpp

    volatile uint16_t currentMicroseconds; // set by write() and writeMicroseconds(). Needed for next move and to avoid unnecessary writes.
    uint16_t startMicroseconds;  // used with millisAtStartMove to compute currentMicroseconds
    uint16_t endMicroseconds;
    int16_t deltaMicroseconds; // end - start
    volatile bool servoMoves;

    uint32_t millisAtStartMove;
    uint16_t millisForCompleteMove;

    // since the values in the Servo library are private we need a copy here
    uint16_t MicrosecondsForServo0Degree;
    uint16_t MicrosecondsForServo180Degree;
};

void disableSmoothServoInterrupt();

#endif /* SMOOTHSERVO_H_ */
