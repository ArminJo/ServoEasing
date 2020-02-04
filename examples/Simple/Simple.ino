/*
 * Simple.cpp
 *
 *  Shows smooth linear movement from one servo position to another.
 *  Do not use interrupts, therefore it can run on any platform where the Arduino Servo library is available.
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

#define VERSION_EXAMPLE "1.4"

#if defined(ESP8266)
const int SERVO1_PIN = 14; // D5
#elif defined(ESP32)
const int SERVO1_PIN = 5;
#elif defined(__STM32F1__)
const int SERVO1_PIN = PB9; // Needs timer 4 for Servo library
#elif defined(PART_apollo3)
const int SERVO1_PIN = 4;  // Apollo software for Arduino supports only one servo and no periodic interrupts (2/2020)
#else
const int SERVO1_PIN = 9;
#endif

// for ESP32 LED_BUILTIN is defined as: static const uint8_t LED_BUILTIN = 2;
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif

ServoEasing Servo1;

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
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // Attach servo to pin
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO1_PIN);
    if (Servo1.attach(SERVO1_PIN) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
    }

    /**************************************************
     * Set servo to start position.
     * This is the position where the movement starts.
     *************************************************/
    Servo1.write(0);

    // Wait for servo to reach start position.
    delay(500);

    // Move slow
    Serial.println(F("Move to 90 degree with 20 degree per second blocking"));
    Servo1.easeTo(90, 20);

    delay(1000);

    // Now move faster
    Servo1.setSpeed(40);  // This speed is taken if no speed argument is given.
    Serial.println(F("Move to 180 degree with 40 degree per second blocking using cubic easing"));
    Servo1.setEasingType(EASE_CUBIC_IN_OUT);
    Servo1.easeTo(180);

    delay(2000);

    Serial.println(F("Move back to 0 degree using circular easing and calling update() manually"));
    Servo1.setEasingType(EASE_CIRCULAR_IN_OUT);
    /*
     * Call non blocking function and call update() in the loop below
     */
    Servo1.startEaseTo(0, 40, false);
    /*
     * Now do the updates manually and wait until servo finished
     */
    do {
        /*
         * First do the delay, then check for update, since we are likely called directly after start and there is nothing to move yet.
         *
         * Use delay of 20 ms here, since it is the refresh rate of the servos and all Arduino Servo libraries.
         * You can use smaller delay values, but keep in mind that every 20 ms a new value can be accepted by a servo.
         */
        delay(REFRESH_INTERVAL / 1000);

    } while (!Servo1.update());

    Serial.println(F("End of setup, start of loop."));
}

void loop() {
    Serial.println(F("Move to 135 degree circular"));
    /*
     * Call non blocking function and call update() in the loop below
     */
    Servo1.startEaseTo(135, 40, false);
    do {
        digitalWrite(LED_BUILTIN, HIGH);
        delayAndUpdateAndWaitForAllServosToStop(100);
        digitalWrite(LED_BUILTIN, LOW);
        delayAndUpdateAndWaitForAllServosToStop(100);
    } while (Servo1.isMoving());

    Serial.println(F("Move to 45 degree circular"));
    Servo1.startEaseTo(45, 40, false);
    /*
     * Call non blocking function and call update() in the loop below
     */
    do {
        digitalWrite(LED_BUILTIN, HIGH);
        delayAndUpdateAndWaitForAllServosToStop(100);
        digitalWrite(LED_BUILTIN, LOW);
        delayAndUpdateAndWaitForAllServosToStop(100);
    } while (Servo1.isMoving());
}
