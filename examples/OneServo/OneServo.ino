/*
 * OneServo.cpp
 *
 *  Shows smooth linear movement from one servo position to another.
 *
 *  Copyright (C) 2019-2021  Armin Joachimsmeyer
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

// Must specify this before the include of "ServoEasing.hpp"
//#define USE_PCA9685_SERVO_EXPANDER // Activate this to enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB // Activate this to force additional using of regular servo library.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT // Activate this to disable all but LINEAR movement. Saves up to 1540 bytes FLASH.
#define DISABLE_COMPLEX_FUNCTIONS // Activate this to disable the SINE, CIRCULAR, BACK, ELASTIC and BOUNCE easings. Saves up to 1850 bytes FLASH.
#define MAX_EASING_SERVOS 1
#define ENABLE_MICROS_AS_DEGREE_PARAMETER // Activate this to enable also microsecond values as (target angle) parameter. Requires additional 128 Bytes FLASH.
//#define DEBUG // Activate this to generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER // Activate this to generate the Arduino plotter output.
#include "ServoEasing.hpp"

#include "PinDefinitionsAndMore.h"
/*
 * Pin mapping table for different platforms
 *
 * Platform     Servo1      Servo2      Servo3      Analog
 * -------------------------------------------------------
 * AVR + SAMD   9           10          11          A0
 * ESP8266      14 // D5    12 // D6    13 // D7    0
 * ESP32        5           18          19          A0
 * BluePill     PB7         PB8         PB9         PA0
 * APOLLO3      11          12          13          A3
 */
#define INFO // to see serial output of loop

ServoEasing Servo1;

#define START_DEGREE_VALUE  0 // The degree value written to the servo at time of attach.

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));

    /********************************************************
     * Attach servo to pin and set servos to start position.
     * This is the position where the movement starts.
     *******************************************************/
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO1_PIN);
    if (Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
    }

    // Wait for servo to reach start position.
    delay(500);
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

void loop() {
    // Move slow
#ifdef INFO
    Serial.println(F("Move to 90 degree with 10 degree per second blocking"));
#endif
    Servo1.setSpeed(10);  // This speed is taken if no further speed argument is given.
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_90_DEGREE);
#else
    Servo1.easeTo(90);
#endif

    // Now move faster without any delay between the moves
#ifdef INFO
    Serial.println(F("Move to 180 degree with 30 degree per second using interrupts"));
#endif
    /*
     * Just for demonstration we use degree value here, which is still possible with the ENABLE_MICROS_AS_DEGREE_PARAMETER option
     */
    Servo1.startEaseTo(180, 30);

    /*
     * Now you can run your program while the servo is moving.
     * Just let the LED blink for 3 seconds (90 degrees moving by 30 degrees per second).
     */
    for (int i = 0; i < 15; ++i) {
        blinkLED();
    }

    delay(1000);

#ifdef INFO
    Serial.println(F("Move to 45 degree in one second using interrupts"));
#endif
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
    Servo1.startEaseToD((544 + ((2400 - 544) / 4)), 1000);
#else
    Servo1.startEaseToD(45, 1000);
#endif
    // Blink until servo stops
    while (Servo1.isMoving()) {
        /*
         * Put your own code here
         */
        blinkLED();
    }

    delay(1000);

#ifdef INFO
    Serial.println(F("Move to 135 degree and back nonlinear in one second each using interrupts"));
#endif
    Servo1.setEasingType(EASE_CUBIC_IN_OUT);

    for (int i = 0; i < 2; ++i) {
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
        Servo1.startEaseToD((544 + (((2400 - 544) / 4) * 3)), 1000);
#else
        Servo1.startEaseToD(135, 1000);
#endif
        // isMoving() calls yield for the ESP8266 boards
        while (Servo1.isMoving()) {
            /*
             * Put your own code here
             */
            ; // no delays here to avoid break between forth and back movement
        }
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
        Servo1.startEaseToD((544 + ((2400 - 544) / 4)), 1000);
#else
        Servo1.startEaseToD(45, 1000);
#endif
        while (Servo1.isMoving()) {
            ; // no delays here to avoid break between forth and back movement
        }
    }
    Servo1.setEasingType(EASE_LINEAR);

    delay(1000);

    /*
     * The LED goes on if servo reaches 120 degree
     */
#ifdef INFO
    Serial.println(F("Move to 180 degree with 50 degree per second blocking"));
#endif
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
    Servo1.startEaseTo(DEFAULT_MICROSECONDS_FOR_180_DEGREE, 50);
#else
    Servo1.startEaseTo(180, 50);
#endif
    while (Servo1.getCurrentAngle() < 120) {
        delay(20); // just wait until angle is above 120 degree
    }
    digitalWrite(LED_BUILTIN, HIGH);
    while (Servo1.isMoving()) {
        ; // wait for servo to stop
    }

    delay(1000);

    /*
     * Very fast move. The LED goes off when servo theoretical reaches 90 degree
     */
#ifdef INFO
    Serial.println(F("Move from 180 to 0 degree with 360 degree per second using interrupts"));
#endif
    /*
     * If you activate the line
     * #define ENABLE_MICROS_AS_DEGREE_PARAMETER
     * in ServoEasing.h, you can specify the target angle directly as microseconds here
     */
//    Servo1.startEaseTo(DEFAULT_MICROSECONDS_FOR_0_DEGREE, 360, true);
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
    Servo1.startEaseTo(DEFAULT_MICROSECONDS_FOR_0_DEGREE, 360, true);
#else
    Servo1.startEaseTo(0, 360, true);
#endif
    // Wait for 250 ms. The servo should have moved 90 degree.
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);

#ifdef INFO
    Serial.println(F("Interrupt movement with stop() for 1 second at 90 degree"));
#endif
    /*
     * Demonstrate stop and continue in the middle of a movement
     */
    Servo1.stop();
    delay(1000);
    // continue movement using interrupts
    Servo1.continueWithInterrupts();

    delay(1000);
}
