/*
 * TwoServos.cpp
 *
 *  Shows smooth movement from one servo position to another for 2 servos synchronously.
 *  Operate the first servo from -90 to +90 degree.
 *  This example uses the LightweightServo library. This saves 640 bytes program space compared to using Arduino Servo library.
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
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#define USE_LEIGHTWEIGHT_SERVO_LIB
#include "LightweightServo.hpp" // include sources of LightweightServo library
#endif

//#define PROVIDE_ONLY_LINEAR_MOVEMENT // Activate this to disable all but LINEAR movement. Saves up to 1540 bytes FLASH.
#define DISABLE_COMPLEX_FUNCTIONS // Activate this to disable the SINE, CIRCULAR, BACK, ELASTIC and BOUNCE easings. Saves up to 1850 bytes FLASH.
//#define MAX_EASING_SERVOS 2
//#define ENABLE_MICROS_AS_DEGREE_PARAMETER // Activate this to enable also microsecond values as (target angle) parameter. Requires additional 128 Bytes FLASH.
//#define DEBUG // Activate this to generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER // Activate this to generate the Arduino plotter output.
#include "ServoEasing.hpp"

#ifndef PRINT_FOR_SERIAL_PLOTTER
#define INFO // to see serial text output for loop
#endif

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

ServoEasing Servo1;
ServoEasing Servo2;

#define START_DEGREE_VALUE  0 // The degree value written to the servo at time of attach.

void blinkLED();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
#ifndef PRINT_FOR_SERIAL_PLOTTER
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));

    /************************************************************
     * Attach servo to pin and set servos to start position.
     * This is the position where the movement starts.
     *
     * The order of the attach() determine the position
     * of the Servos in internal ServoEasing::ServoEasingArray[]
     ***********************************************************/
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO1_PIN);
#endif
    Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);

#ifndef PRINT_FOR_SERIAL_PLOTTER
    /*
     * Check at least the last call to attach()
     */
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO2_PIN);
#endif
    if (Servo2.attach(SERVO2_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
        while (true) {
            blinkLED();
        }
    }

#ifdef PRINT_FOR_SERIAL_PLOTTER
    // Print legend for Plotter
    Serial.println("Servo1, Servo2");
#endif
    /*
     * Operate Servo1 from -90 to +90 degree
     * Instead of specifying a trim you can use above:
     *   if (Servo1.attach(SERVO1_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, -90, 90) == INVALID_SERVO) {
     */
    Servo1.setTrim(90);

    setSpeedForAllServos(30);

    // Just wait for servos to reach position.
    delay(500);
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

void loop() {

    /*
     * Move both servos blocking
     */
#ifdef INFO
    Serial.println(F("Move to 0/90 degree with 30 degree per second blocking"));
#endif
    setSpeedForAllServos(30);
    Servo1.setEaseTo(0);
    Servo2.setEaseTo(90);
    synchronizeAllServosStartAndWaitForAllServosToStop();

    /*
     * Now continue faster.
     */
#ifdef INFO
    Serial.println(F("Move to 90/10 degree with up to 60 degree per second using interrupts"));
#endif
    Servo1.setEaseTo(90, 60);
    /*
     * An alternative method to synchronize and start
     * Synchronize by simply using the same duration
     */
    Servo2.startEaseToD(10, Servo1.mMillisForCompleteMove); // This start interrupt for all servos
    /*
     * Now you can run your program while the servos are moving.
     * Just let the LED blink until servos stop.
     */
    while (ServoEasing::areInterruptsActive()) {
        blinkLED();
    }

    /*
     * Move servo1 using cubic easing. Use interrupts for update.
     *  The first servo moves with the specified speed.
     *  The second will be synchronized to slower speed (longer duration, than specified) because it has to move only 80 degree.
     */
#ifdef INFO
    Serial.println(F("Move to 0/90 degree with up to 90 degree per second using interrupts. Use cubic easing for first servo."));
#endif
    Servo1.setEasingType(EASE_CUBIC_IN_OUT);
    /*
     * Another method to specify moves
     * Use the ServoEasingNextPositionArray and then call the appropriate function
     */
    ServoEasing::ServoEasingNextPositionArray[0] = 0;
    ServoEasing::ServoEasingNextPositionArray[1] = 90;
    setEaseToForAllServosSynchronizeAndStartInterrupt(90);

    // Must call yield here for the ESP boards, since we have no delay called
    while (ServoEasing::areInterruptsActive()) {
        ;
    }
    Servo1.setEasingType(EASE_LINEAR);

    delay(300);

    /*
     * Move both servos independently
     */
#ifdef INFO
    Serial.println(F("Move independently to -90/0 degree with 60/80 degree per second using interrupts"));
#endif
    Servo1.setEaseTo(-90, 60);
    Servo2.startEaseTo(0, 80); // This start interrupt for all servos
    // blink until both servos stop
    while (ServoEasing::areInterruptsActive()) {
        blinkLED();
    }

    delay(500);

}
