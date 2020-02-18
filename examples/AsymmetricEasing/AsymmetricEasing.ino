/*
 * AsymmetricEasing.cpp
 *
 *  Shows asymmetric (end movement is different from start movement) user defined non linear movements for 3 servos synchronously.
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

/*
 * To generate the Arduino plotter output, you must comment out the line #define PRINT_FOR_SERIAL_PLOTTER in ServoEasing.h
 */
#include "ServoEasing.h"

#define VERSION_EXAMPLE "2.0"

#if defined(ESP8266)
const int SERVO1_PIN = 14; // D5
const int SERVO2_PIN = 12; // D6
const int SERVO3_PIN = 13; // D7
const int SPEED_IN_PIN = 0;

#elif defined(ESP32)
const int SERVO1_PIN = 5;
const int SERVO2_PIN = 18;
const int SERVO3_PIN = 19;
const int SPEED_IN_PIN = 36;

#elif defined(__STM32F1__)
const int SERVO1_PIN = PB7;
const int SERVO2_PIN = PB8;
const int SERVO3_PIN = PB9; // Needs timer 4 for Servo library
const int SPEED_IN_PIN = PA0;

#else
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;
const int SERVO3_PIN = 11;
const int SPEED_IN_PIN = A0;
#endif

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;

// forward declarations
float EaseQuadraticInQuarticOut(float aPercentageOfCompletion);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__)
    while (!Serial); //delay for Leonardo, but this loops forever for Maple Serial
#endif
#if defined(SERIAL_USB)
    delay(2000); // To be able to connect Serial monitor after reset and before first printout
#endif
    // Just to know which program is running on my Arduino
#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
#endif

#if defined(ESP32)
    analogReadResolution(10);
#endif

    // Attach servos to pins
#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO1_PIN);
#endif
    if (Servo1.attach(SERVO1_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
    }

#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO2_PIN);
#endif
    if (Servo2.attach(SERVO2_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
    }

    /*
     * Check at least the last call to attach()
     */
#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO3_PIN);
#endif
    if (Servo3.attach(SERVO3_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }

    /**************************************************
     * Set servos to start position.
     * This is the position where the movement starts.
     *************************************************/
    Servo1.write(90);
    Servo2.write(90);
    Servo3.write(90);

    // Wait for servos to reach start position.
    delay(2000);

#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.println(F("Move from 90 to 45 degree in 1 second"));
#endif
    Servo1.startEaseToD(45, 1000);
    Servo2.startEaseToD(45, 1000);
    Servo3.startEaseToD(45, 1000);
    delay(1000);

    Servo1.setEasingType(EASE_USER_DIRECT);
    Servo1.registerUserEaseInFunction(EaseQuadraticInQuarticOut);
    Servo2.setEasingType(EASE_ELASTIC_OUT);
#ifndef KEEP_LIBRARY_SMALL
    Servo3.setEasingType(EASE_BOUNCE_OUT);
#else
    Servo3.setEasingType(EASE_USER_DIRECT);
    Servo3.registerUserEaseInOutFunction(EaseOutBounce);
#endif
    Serial.println("QuadraticInQuarticOut, ElasticOut, BounceOut");

    delay(500);

}

void loop() {

    uint16_t tSpeed = analogRead(SPEED_IN_PIN);
#if defined(__STM32F1__)
    tSpeed = map(tSpeed, 0, 4096, 5, 150); // 12 bit ADC
#else
    tSpeed = map(tSpeed, 0, 1023, 5, 150);
#endif
    setSpeedForAllServos(tSpeed);

    /*
     * Move three servos synchronously without interrupt handler
     */
#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.print(F("Move to 135 degree with "));
    Serial.print(tSpeed);
    Serial.println(F(" degree per second with with updates by own do-while loop"));
#endif
    /*
     * Here we use the allServos functions
     */
    setDegreeForAllServos(3, 135, 135, 135);
    setEaseToForAllServos();
    synchronizeAllServosAndStartInterrupt(false); // false, since we call updateAllServos() manually below

    do {
        // here you can call your own program
        delay(REFRESH_INTERVAL / 1000); // optional 20 ms delay - REFRESH_INTERVAL is in Microseconds
    } while (!updateAllServos());

    /*
     * Move three servos synchronously with interrupt handler
     */
#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.print(F("Move to 45 degree with "));
    Serial.print(tSpeed);
    Serial.println(F(" degree per second using interrupts"));
#endif
    Servo1.setEaseTo(45);
    Servo2.setEaseToD(45, Servo1.mMillisForCompleteMove);
    Servo3.startEaseToD(45, Servo1.mMillisForCompleteMove);
    /*
     * No need to call synchronizeAllServosAndStartInterrupt(), since I know that all durations are the same
     * Since all servos stops at the same time I have to check only one
     * Must call yield here for the ESP boards, since we do not call delay in the loop
     */
    while (Servo3.isMovingAndCallYield()) {
        ; // no delays here to avoid break between forth and back movement
    }
}

/*
 * User defined combined movement
 */
float EaseQuadraticInQuarticOut(float aPercentageOfCompletion) {
    if (aPercentageOfCompletion <= 0.5) {
        // Quadratic IN - output from 0.0 to 0.5
        return (2 * QuadraticEaseIn(aPercentageOfCompletion));
    } else {
        // Quartic OUT - output from 0.5 to 1.0
        return (1.0 - (8 * QuarticEaseIn(1.0 - aPercentageOfCompletion)));
    }
}
