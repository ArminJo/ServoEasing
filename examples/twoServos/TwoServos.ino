/*
 * TwoServos.cpp
 *
 *  Shows smooth movement from one servo position to another for 2 servos synchronously.
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

#define VERSION_EXAMPLE "1.0"

const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;

ServoEasing Servo1;
ServoEasing Servo2;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // Attach servos to pins
    Serial.println(F("Attach servos"));
    Servo1.attach(SERVO1_PIN, SG_90_MICROSECONDS_FOR_0_DEGREE, SG_90_MICROSECONDS_FOR_180_DEGREE);
    Servo2.attach(SERVO2_PIN, SG_90_MICROSECONDS_FOR_0_DEGREE, SG_90_MICROSECONDS_FOR_180_DEGREE);

    // Set servo to synchronize for servo1
    Servo1.setSynchronizedServo(&Servo2);

    // Set servos to start position.
    Servo1.write(0);
    Servo2.write(0);

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
    // Move both servos blocking
    Serial.println(F("Move to 90/90 degree with 30 degree per second blocking"));
    Servo1.easeToSynchronized(90, 90, 30);

    // Now move faster. The first servo moves with the specified speed, the second must move slower in order to be synchronized.
    Serial.println(F("Move to 180/10 degree with 60 degree per second using interrupts"));
    Servo1.startEaseToSynchronized(180, 10, 60, true);
    /*
     * Now you can run your program while the servos are moving.
     * Just let the LED blink until servos stop.
     */
    while (Servo1.isMoving()) {
        blinkLED();
    }

    Servo1.setEasingType(EASE_CUBIC);
    // Move servo1 using cubic easing. Use interrupts for update
    Serial.println(F("Move to 90/90 degree with 90 degree per second using interrupts. Use cubic easing for first servo."));
    Servo1.startEaseToSynchronized(90, 90, 90, true);
    while (Servo1.isMoving()) {
        ;
    }
    Servo1.setEasingType(EASE_LINEAR);

    delay(300);

    // Move both servos independently
    Serial.println(F("Move independently to 0/0 degree with 80/60 degree per second using interrupts"));
    Servo1.startEaseTo(0, 80);
    Servo2.startEaseTo(0, 60, true); // Interrupts work here for both, because Servo2 it is specified in setSynchronizedServo(). -> see ThreeServos example.
    // blink until both servos stop
    while (Servo1.isMoving() || Servo2.isMoving()) {
        blinkLED();
    }

    delay(500);

}
