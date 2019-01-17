/*
 * TwoServos.cpp
 *
 *  Shows smooth movement from one servo position to another for 2 servos synchronously.
 *
 *  Copyright (C) 2019  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of SmoothServo.
 *  SmoothServo is free software: you can redistribute it and/or modify
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

#include <SmoothServo.h>

#define VERSION_EXAMPLE "1.0"

const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;

SmoothServo Servo1;
SmoothServo Servo2;

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // attach servos to pins
    Serial.println(F("Attach servos"));
    Servo1.attach(SERVO1_PIN);
    Servo2.attach(SERVO2_PIN);
    // set servo to synchronize for servo1
    Servo1.setSyncronizedServo(&Servo2);
    // set servos to start position.
    Servo1.write(0);
    Servo2.write(0);
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
    Serial.print(F("Move to 90/90 degree with 20 degree per second"));
    Servo1.moveToSyncronized(90, 90, 20);
    delay(1000);

    // Now move faster. The first servo moves with the specified speed, the second must move slower in order to be synchronized.    Serial.print(F("Move to 90/90 degree with 20 degree per second"));
    Serial.print(F("Move to 180/180 degree with 40 degree per second"));
    Servo1.startMoveToSyncronized(180, 135, 40, true);
    /*
     * Now you can run your program while the servos are moving.
     * Just let the LED blink until servos stop.
     */
    while (Servo1.isMoving()) {
        blinkLED();
    }
    delay(1000);

    // move only first servo in interrupt mode
    Serial.print(F("Move first to 90 degree with 80 degree per second"));
    Servo1.startMoveTo(90, 80, true);
    // blink until servo stops
    while (Servo1.isMoving()) {
        blinkLED();
    }
    delay(1000);

    // move only second servo in blocking mode
    Serial.print(F("Move second to 90 degree with 80 degree per second"));
    Servo2.moveTo(90, 80);
    delay(1000);

    // move both independently
    Serial.print(F("Move to 0/0 degree with 40/20 degree per second"));
    Servo1.startMoveTo(0, 40);
    Servo2.startMoveTo(0, 20, true); // Interrupts work here, because Servo2 it is specified in setSyncronizedServo(). -> see ThreeServos example.
    // blink until servos stops
    while (Servo2.isMoving()) {
        blinkLED();
    }
    delay(2000);

}
