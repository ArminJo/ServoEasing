/*
 * Simple.cpp
 *
 *  Shows smooth linear movement from one servo position to another.
 *  Do not use interrupts, therefore it can run on any platform where the Arduino Servo library is available.
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

#define START_DEGREE_VALUE  0 // The degree value written to the servo at time of attach.

void toggleLED_BUILTIN_Every10thCall();

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

    // Move slow
    Serial.println(F("Move to 90 degree with 20 degree per second blocking"));
    Servo1.easeTo(90, 20);

    delay(1000);

    // Now move faster
    Servo1.setSpeed(40);  // This speed is taken if no speed argument is given.
    Serial.println(F("Move to 180 degree using cubic easing with 40 degree per second blocking"));
    Servo1.setEasingType(EASE_CUBIC_IN_OUT);
    Servo1.easeTo(180);

    delay(2000);
    Servo1.setEasingType(EASE_CIRCULAR_IN_OUT);
    Serial.println(F("End of setup, start of loop."));
}

void loop() {

    Serial.println(F("Move to 135 degree circular with 40 degree per second blocking"));
    Servo1.easeTo(135); // Speed was specified above by Servo1.setSpeed(40)

    Serial.println(F("Move to 45 degree circular with 40 degree per second using interrupts"));
    /*
     * Use interrupts for moving and wait until moving stops
     */
    Servo1.startEaseTo(45);
    do {
        /*
         * Put your own code here
         */
        toggleLED_BUILTIN_Every10thCall();

        /*
         * First do the delay, then check for moving, since we are likely called directly after start and there is nothing to move yet.
         *
         * Use delay of 20 ms here, since it is the refresh rate of the servos and all Arduino Servo libraries.
         * You can use smaller delay values, but keep in mind that every 20 ms a new value is sent to the servo.
         */
        delay(REFRESH_INTERVAL_MILLIS);

    } while (Servo1.isMoving());
}

/*
 * Enable low delays for slow blink
 */
void toggleLED_BUILTIN_Every10thCall() {
    static int tCount = 0;
    if (++tCount == 10) {
        tCount = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
