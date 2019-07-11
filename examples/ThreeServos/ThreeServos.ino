/*
 * ThreeServos.cpp
 *
 *  Shows smooth movement from one servo position to another for 3 servos synchronously.
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

#define VERSION_EXAMPLE "1.1"

#if defined(ESP8266)
const int SERVO1_PIN = 14; // D5
const int SERVO2_PIN = 12; // D6
const int SERVO3_PIN = 13; // D7
#elif defined(ESP32)
const int SERVO1_PIN = 5;
const int SERVO2_PIN = 18;
const int SERVO3_PIN = 19;
#else
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;
const int SERVO3_PIN = 11;
#endif

// for ESP32 LED_BUILTIN is defined as static const uint8_t LED_BUILTIN = 2;
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial)
        ; //delay for Leonardo
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // Attach servos to pins
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO1_PIN);
    if (Servo1.attach(SERVO1_PIN) == false) {
        Serial.println(F("Error attaching servo"));
    }
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO2_PIN);
    if (Servo1.attach(SERVO2_PIN) == false) {
        Serial.println(F("Error attaching servo"));
    }
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO3_PIN);
    if (Servo1.attach(SERVO3_PIN) == false) {
        Serial.println(F("Error attaching servo"));
    }

    // Set servos to start position.
    Servo1.write(0);
    Servo2.write(0);
    Servo3.write(0);

    // Just wait for servos to reach position
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
     * Move three servos synchronously without interrupt handler
     */
    Serial.println(F("Move to 90/90/180 degree with 20 degree per second with own update loop"));
    setSpeedForAllServos(20);
    Servo1.setEaseTo(90);
    Servo2.setEaseTo(90);
    Servo3.setEaseTo(180);
    synchronizeAllServosAndStartInterrupt(false); // do not start interrupt

    do {
        // here you can call your own program
        delay(REFRESH_INTERVAL / 1000); // optional 20ms delay - REFRESH_INTERVAL is in Microseconds
    } while (!updateAllServos());

    delay(1000);

    /*
     * Move three servos synchronously with interrupt handler
     */
    Serial.println(F("Move to 180/180/0 degree with 30 degree per second using interrupts"));
    sServoNextPositionArray[0] = 180;
    sServoNextPositionArray[1] = 180;
    sServoNextPositionArray[2] = 0;
    setEaseToForAllServosSynchronizeAndStartInterrupt(30);
    /*
     * Now you can run your program while the servos are moving.
     * Just let the LED blink until servos stop.
     * Since all servos stops at the same time I have to check only one
     */
    while (Servo1.isMoving()) {
        blinkLED();
    }

    delay(1000);

    /*
     * Move first and second servo synchronously with interrupt handler
     */
    Serial.println(F("Move to 90/90 degree with 40 degree per second using interrupts"));
    Servo1.setEaseTo(90, 80);
    Servo2.startEaseToD(90, Servo1.mMillisForCompleteMove);
    // No timing synchronization needed :-)
    // blink until servo stops
    while (Servo2.isMoving()) {
        blinkLED();
    }

    delay(1000);

    // Move only third servo
    Serial.println(F("Move third to 90 degree with 80 degree per second blocking"));
    Servo3.easeTo(90, 80);

    delay(1000);

    /*
     * Move all 3 servos independently
     */
    Serial.println(F("Move independently to 0/0/0 degree with 80/40/20 degree per second using interrupts"));
    Servo1.setEaseTo(0, 80);
    Servo2.setEaseTo(0, 40);
    Servo3.startEaseTo(0, 20); // Start interrupt for all servos. No synchronization here since the servos should move independently.
    // Blink until servos stops
    while (Servo1.isMoving() || Servo2.isMoving() || Servo3.isMoving()) {
        blinkLED();
    }

    delay(2000);

}
