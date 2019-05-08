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
 *    90 degree 300 ms   -> 300 degree per second
 *    45 degree 180 ms
 *    30 degree 150 ms
 *    20 degree 130 ms
 *    10 degree 80 ms    -> 125 degree per second
 *
 *  MG90Sservo
 *   180 degree 330 ms -> 540 degree per second
 *    90 degree 220 ms
 *    45 degree 115 ms
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
 * 620 and 2400 are the values for some of my sg90 servos determined with EndPositionTest example
 * 544 suits better for standard servos
 */
#define ZERO_DEGREE_VALUE_MICROS 544
//#define ZERO_DEGREE_VALUE_MICROS 620
#define AT_180_DEGREE_VALUE_MICROS 2400

void doSwipe(uint8_t aDegreePerStep);

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial)
        ; //delay for Leonardo
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    Serial.print(F("Value for 0 degree="));
    Serial.print(ZERO_DEGREE_VALUE_MICROS);
    Serial.print(F("us. Value for 180 degree="));
    Serial.print(AT_180_DEGREE_VALUE_MICROS);
    Serial.println(F("us."));

    // attach servo to pin 9
    ServoUnderTest.attach(SERVO_UNDER_TEST_PIN, ZERO_DEGREE_VALUE_MICROS, AT_180_DEGREE_VALUE_MICROS);
    ServoUnderTest.write(90);
    delay(2000);
}

void loop() {
    int tPulseMicros;

    int tVoltageMillivolts = getVCCVoltageMillivolt();

    Serial.print(F("VCC="));
    Serial.print(tVoltageMillivolts);
    Serial.print(F(" mV"));

    int tMode = analogRead(MODE_ANALOG_INPUT_PIN);
    int tSpeedOrPosition = analogRead(SPEED_OR_POSITION_ANALOG_INPUT_PIN);

    tMode = tMode >> 7; // gives values 0-7
    Serial.print(F(" Mode="));
    Serial.print(tMode);
    Serial.print(' ');

    switch (tMode) {
    case 0:
        // direct position from 0 to 180 degree. Choose bigger range just to be sure both ends are reached.
        tPulseMicros = map(tSpeedOrPosition, 0, 1023, ZERO_DEGREE_VALUE_MICROS - 100, AT_180_DEGREE_VALUE_MICROS + 200);
        ServoUnderTest.writeMicroseconds(tPulseMicros);
        Serial.print(F("direct position: micros="));
        Serial.print(tPulseMicros);
        Serial.print(F(" degree="));
        Serial.println(ServoUnderTest.read());
        delay(200);
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
        tSpeedOrPosition = (((1023 - tSpeedOrPosition) >> 7) * 20) + 20; // gives values 20, 40 to 160
        Serial.print(F("fast reaction: 85 -> 90 -> 95 -> 90 delay="));
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
        Serial.println(F("invalid mode!"));
        break;
    }
}

int readDelay(uint8_t aDegreePerStep) {
    int tDelayMillis = analogRead(SPEED_OR_POSITION_ANALOG_INPUT_PIN);
    // speed is 1/delay so invert map function
    return (map(tDelayMillis, 1023, 0, aDegreePerStep, aDegreePerStep * 10));
}

/*
 * Do a swipe and use different intervals
 */
void doSwipe(uint8_t aDegreePerStep) {
    // print delay once for info here, but be aware that it can be changed during the sweep to fasten the measurement process
    int tDelayMillis = readDelay(aDegreePerStep);
    Serial.print(F("swipe: degree per step="));
    Serial.print(aDegreePerStep);
    Serial.print(F(" , delay="));
    Serial.print(tDelayMillis);
    Serial.println(F(" ms"));

    uint8_t tDegree = 0;
    while (tDegree < 180) {
        ServoUnderTest.write(tDegree); // starts with 0
        // read again to enable fast reaction to changed value
        delay(readDelay(aDegreePerStep));
        tDegree += aDegreePerStep;
    }
    while (tDegree > 0) {
        ServoUnderTest.write(tDegree);
        // read again to enable fast reaction to changed value
        delay(readDelay(aDegreePerStep));
        tDegree -= aDegreePerStep;
    }
}
