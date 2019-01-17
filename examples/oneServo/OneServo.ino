/*
 * OneServo.cpp
 *
 *  Shows smooth movement from one servo position to another.
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

SmoothServo Servo1;

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // attach servo to pin
    Serial.println(F("Attach servo"));
    Servo1.attach(SERVO1_PIN);
    // set servo to start position.
    Servo1.write(0);
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
    Serial.print(F("Move to 90 degree with 10 degree per second"));
    Servo1.moveTo(90, 10);

    // Now move faster
    Serial.print(F("Move to 180 degree with 30 degree per second"));
    Servo1.startMoveTo(180, 30, true);
    /*
     * Now you can run your program while the servo is moving.
     * Just let the LED blink for 3 seconds (90 degrees moving by 30 degrees per second).
     */
    for (int i = 0; i < 15; ++i) {
        blinkLED();
    }
    delay(1000);

    Serial.print(F("Move to 90 degree with 80 degree per second"));
    Servo1.startMoveTo(90, 80, true);
    // blink until servo stops
    while (Servo1.isMoving()) {
        blinkLED();
    }
    delay(1000);

    /*
     * Now measure how fast the servo can move. The LED goes off when servo reaches 90 degree
     */
    Serial.print(F("Move to 0 degree with 360 degree per second"));
    Servo1.startMoveTo(0, 360, true);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}
