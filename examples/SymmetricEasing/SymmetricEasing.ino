/*
 * SymmetricEasing.cpp
 *
 *  Shows symmetric (end movement is mirror of start movement) linear, quadratic and cubic movements for 3 servos synchronously.
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
//#define DISABLE_COMPLEX_FUNCTIONS // Activate this to disable the SINE, CIRCULAR, BACK, ELASTIC and BOUNCE easings. Saves up to 1850 bytes FLASH.
//#define MAX_EASING_SERVOS 3
//#define ENABLE_MICROS_AS_DEGREE_PARAMETER // Activate this to enable also microsecond values as (target angle) parameter. Requires additional 128 Bytes FLASH.
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

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;

#define START_DEGREE_VALUE 90

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));
#endif

#if defined(ESP32)
    analogReadResolution(10);
#endif

    /************************************************************
     * Attach servo to pin and set servos to start position.
     * This is the position where the movement starts.
     *
     * The order of the attach() determine the position
     * of the Servos in internal ServoEasing::ServoEasingArray[]
     ***********************************************************/
#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO1_PIN);
#endif
    if (Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE,
    DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
    }

#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO2_PIN);
#endif
    if (Servo2.attach(SERVO2_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE,
    DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
    }

    /*
     * Check at least the last call to attach()
     */
#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO3_PIN);
#endif
    if (Servo3.attach(SERVO3_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE,
    DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }

    // Wait for servos to reach start position.
    delay(2000);

#ifdef PRINT_FOR_SERIAL_PLOTTER
    // Legend for Arduino Serial plotter
    Serial.println();
    Serial.println("Linear, Quadratic, Cubic");
#endif

#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.println(F("Move from 90 to 45 degree in 1 second"));
#endif
    Servo1.startEaseToD(45, 1000);
    Servo2.startEaseToD(45, 1000);
    Servo3.startEaseToD(45, 1000);
    delay(1000);

    Servo1.setEasingType(EASE_LINEAR);
    Servo2.setEasingType(EASE_QUADRATIC_IN_OUT);
    Servo3.setEasingType(EASE_CUBIC_IN_OUT);

//    Servo1.setEasingType(EASE_QUADRATIC_IN_OUT);
//    Servo2.setEasingType(EASE_CUBIC_IN_OUT);
//    Servo3.setEasingType(EASE_SINE_IN_OUT);
//    Serial.println("Quadratic, Cubic, Sine");
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
    Serial.print(Servo1.getSpeed());
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
        delay(REFRESH_INTERVAL_MILLIS); // optional 20ms delay
    } while (!updateAllServos());

    /*
     * Move three servos synchronously with interrupt handler
     */
#ifndef PRINT_FOR_SERIAL_PLOTTER
    Serial.print(F("Move to 45 degree with "));
    Serial.print(Servo1.getSpeed());
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
    while (ServoEasing::areInterruptsActive()) {
        ; // no delays here to avoid break between forth and back movement
    }

}
