/*
 * DummyServo.cpp
 *
 *  Dummy servo library as example for a user provided servo library,
 *  which is activated by #define USE_USER_PROVIDED_SERVO_LIB.
 *
 *  Copyright (C) 2024  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

/*
 * We only need to implement:
 *  Servo                       - Class for manipulating servo motors connected to Arduino pins.
 *  attach(pin, min, max)       - Attaches to a pin setting min and max values in microseconds
 *  writeMicroseconds(value)    - Sets the servo pulse width in microseconds
 *  detach()                    - Stops an attached servo from pulsing its i/o pin.
 */

#include <Arduino.h>
#include "DummyServo.h"

Servo::Servo() {
}

uint8_t Servo::attach(int pin, int min, int max) {
    Serial.print(F("Called attach("));
    Serial.print(pin);
    Serial.print(F(", "));
    Serial.print(min);
    Serial.print(F(", "));
    Serial.print(max);
    Serial.println(')');
    return pin;
}

void Servo::detach() {
    Serial.println(F("Called detach()"));
}
void Servo::writeMicroseconds(int value) {
    Serial.print(F("Called writeMicroseconds("));
    Serial.print(value);
    Serial.println(')');
}

