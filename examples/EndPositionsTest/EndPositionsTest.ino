/*
 * EndPositionsTest.cpp
 *
 *  Allows to determine the exact end position of the servo under test.
 *
 *  Turn the potentiometer until the servo begins to move and see the serial output for the value at this position.
 *
 *  My SG90 servos have 620 and 2400 micro seconds for 0 and 180 degree at 4.8 Volt
 *  My Modelcraft RS-2 servo has 535 and 2400 micro seconds for 0 and 180 degree at 4.8 Volt
 *  My M-1504 servo has 545 and 2325 micro seconds for 0 and 180 degree at 4.6 Volt
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

#include <Servo.h>

#include "ADCUtils.h" // for get getVCCVoltageMillivolt

#define VERSION_EXAMPLE "1.0"

const int SERVO_UNDER_TEST_PIN = 9;

// Attach the sliding contact of the potentiometer here
const int POSITION_ANALOG_INPUT_PIN = A1;
// If this pin is pulled low, determine the upper limit of the servo
const int DETERMINE_UPPER_LIMIT_PIN = 4;
bool determineLowerLimit;

// Value if potentiometer is at 0 Volt. Maximum adjustable value is 200us more
#define ZERO_DEGREE_MINIMUM_VALUE_MICROS 500
// Value if potentiometer is at 5 Volt (VSS). Minimum adjustable value is 200us less
#define AT_180_DEGREE_MAXIMUM_VALUE_MICROS 2500

#define MICROS_FOR_MINIMAL_MOVEMENT 30 // > 25

Servo ServoUnderTest;

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(DETERMINE_UPPER_LIMIT_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // attach servo to pin
    ServoUnderTest.attach(SERVO_UNDER_TEST_PIN);
    // set servo to start position.
    ServoUnderTest.write(0);
    delay(500);
}

void loop() {
    int tMicrosToModify = analogRead(POSITION_ANALOG_INPUT_PIN) / 5;
    determineLowerLimit = digitalRead(DETERMINE_UPPER_LIMIT_PIN);
    int tVoltageMillivolts = getVCCVoltageMillivolt();
    int tMicrosForTest;
    if (determineLowerLimit) {
        Serial.print("0 degree ");
        digitalWrite(LED_BUILTIN, LOW);
        tMicrosForTest = ZERO_DEGREE_MINIMUM_VALUE_MICROS + tMicrosToModify;
    } else {
        Serial.print("180 degree ");
        digitalWrite(LED_BUILTIN, HIGH);
        tMicrosForTest = AT_180_DEGREE_MAXIMUM_VALUE_MICROS - tMicrosToModify;
    }
    ServoUnderTest.writeMicroseconds(tMicrosForTest);
    Serial.print(tMicrosForTest);
    Serial.print("us at ");
    // since the values may depend from the supply voltage, print this value too,
    // but be careful, it may not be exact due to the tolerance of the internal bandgap reference
    Serial.print(tVoltageMillivolts);
    Serial.println(" millivolts");

    delay(80); // multiple of 20
    if (determineLowerLimit) {
        tMicrosForTest += MICROS_FOR_MINIMAL_MOVEMENT;
    } else {
        tMicrosForTest -= MICROS_FOR_MINIMAL_MOVEMENT;
    }
    ServoUnderTest.writeMicroseconds(tMicrosForTest);
    delay(80); // multiple of 20
}
