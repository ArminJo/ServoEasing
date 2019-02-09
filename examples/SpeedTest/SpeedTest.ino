/*
 *  SpeedTest.cpp
 *
 *  With the potentiometer at pin A0 choose the mode to test.
 *  The modes 1 to 6 make a sweep with different intervals.
 *
 *  The interval/speed is determined by the other potentiometer at pin A1
 *  If the servo makes no breaks, but reaches 0 and 180 degree then you have specified the smallest delay / fastest speed for this stepping
 *
 * These are the fastest values for my SG90 servos at 5V (4.2 with servo active)
 *   180 degree 400 ms  -> 450 degree per second
 *   90 degree 300 ms   -> 300 degree per second
 *   45 degree 180 ms
 *   30 degree 150 ms
 *   20 degree 130 ms
 *   10 degree 80 ms    -> 125 degree per second
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

#define MODE_ANALOG_INPUT_PIN A0
#define SPEED_OR_POSITION_ANALOG_INPUT_PIN A1

const int SERVO_UNDER_TEST_PIN = 9;
Servo ServoUnderTest;

/*
 * CHANGE THESE TO REFLECT THE CORRECT VALUES OF YOUR SERVO
 *
 * 620 and 2400 are the values for my sg90 servos determined with EndPositionTest example
 * 544 suits better for standard servos
 */
#define ZERO_DEGREE_VALUE_MICROS 610
//#define ZERO_DEGREE_VALUE_MICROS 544
//#define ZERO_DEGREE_VALUE_MICROS 620
#define AT_180_DEGREE_VALUE_MICROS 2400

void doSwipe(uint8_t aDegreePerStep);

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial); //delay for Leonardo
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // attach servo to pin 9
    ServoUnderTest.attach(SERVO_UNDER_TEST_PIN, ZERO_DEGREE_VALUE_MICROS, AT_180_DEGREE_VALUE_MICROS);
    ServoUnderTest.write(90);
    delay(2000);
}

void loop() {
    int tPulseMicros;

    int tVoltageMillivolts = getVCCVoltageMillivolt();
    Serial.print(tVoltageMillivolts);
    Serial.println(" millivolts");

    int tMode = analogRead(MODE_ANALOG_INPUT_PIN);
    int tSpeedOrPosition = analogRead(SPEED_OR_POSITION_ANALOG_INPUT_PIN);

    tMode = tMode >> 7; // gives values 0-7
    Serial.print("Mode=");
    Serial.print(tMode);
    Serial.print(' ');

    switch (tMode) {
    case 0:
        // direct position from 0 to 180 degree
        tPulseMicros = map(tSpeedOrPosition, 0, 1023, 500, 2500);
        Serial.print("direct position: micros=");
        Serial.println(tPulseMicros);
        ServoUnderTest.writeMicroseconds(tPulseMicros);
        delay(100);
        break;

    case 1:
        doSwipe(180);
        break;

    case 2:
        doSwipe(90);
        break;

    case 3:
        doSwipe(45);
        break;

    case 4:
        doSwipe(30);
        break;

    case 5:
        doSwipe(20);
        break;

    case 6:
        doSwipe(10);
        break;

    case 7:
        // Test for fast reaction to write
        tSpeedOrPosition = ((tSpeedOrPosition >> 7) * 20) + 20; // gives values 20, 40 - 160
        Serial.print("fast reaction: 85 -> 90 -> 95 -> 90 delay=");
        Serial.println(tSpeedOrPosition);

        ServoUnderTest.write(85);
        delay(tSpeedOrPosition);
        ServoUnderTest.write(90);
        delay(tSpeedOrPosition);
        ServoUnderTest.write(95);
        delay(tSpeedOrPosition);
        ServoUnderTest.write(90);
        delay(tSpeedOrPosition);
        break;

    default:
        Serial.println("invalid mode!");
        break;
    }
}

/*
 * Do a swipe and use different intervals
 */
void doSwipe(uint8_t aDegreePerStep) {
    // print delay once for info here, but be aware, it can be changed during the sweep to fasten the measurement process
    int tDelayMillis = analogRead(SPEED_OR_POSITION_ANALOG_INPUT_PIN);
    tDelayMillis = map(tDelayMillis, 0, 1023, aDegreePerStep, aDegreePerStep * 10);
    Serial.print("swipe: degree per step=");
    Serial.print(aDegreePerStep);
    Serial.print(" , delay=");
    Serial.print(tDelayMillis);
    Serial.println(" ms");

    uint8_t tDegree = 0;
    while (tDegree < 180) {
        ServoUnderTest.write(tDegree); // starts with 0
        tDelayMillis = analogRead(SPEED_OR_POSITION_ANALOG_INPUT_PIN);
        delay(map(tDelayMillis, 0, 1023, aDegreePerStep, aDegreePerStep * 10));
        tDegree += aDegreePerStep;
    }
    while (tDegree > 0) {
        ServoUnderTest.write(tDegree);
        tDelayMillis = analogRead(SPEED_OR_POSITION_ANALOG_INPUT_PIN);
        delay(map(tDelayMillis, 0, 1023, aDegreePerStep, aDegreePerStep * 10));
        tDegree -= aDegreePerStep;
    }
}
