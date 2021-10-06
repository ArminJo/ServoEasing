/*
 *  SpeedTest.cpp
 *
 * This example gives you a feeling how fast your servo can move, what the end position values are and which refresh rate they accept.
 * It starts with setting the servo to 90 degree, to easily put your servos in a reference position.
 * This example does not use the ServoEasing functions.
 *
 *  With the potentiometer at pin A0 choose the mode to test.
 *  The modes 1 to 6 make a sweep with different intervals.
 *
 *  The interval/speed is determined by the other potentiometer at pin A1.
 *  If the servo makes no breaks, but reaches 0 and 180 degree then you have specified the smallest delay / fastest speed for this stepping.
 *
 *  By using the lightweight servo library it is also possible to modify the servo refresh interval.
 *  With the potentiometer at pin A3 you can change the refresh interval of the servo pulse between 2.5 and 20 ms.
 *  The layout of pins for this potentiometer is chosen to be able to directly put this potentiometer at the breadboard without additional wiring.
 *
 * These are the fastest values for my SG90 servos at 5 volt (4.2 with servo active)
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

#define VERSION_EXAMPLE "1.0"

#if defined(__AVR__)
#include "ADCUtils.h" // for get getVCCVoltageMillivolt
/*
 * By using LightweightServo library we can change the refresh period.
 * ... and discover that at least SG90 and MG90 servos accept a refresh period down to 2.5 ms!
 */
//#define USE_LEIGHTWEIGHT_SERVO_LIB  // Only available for AVR
#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
#include "LightweightServo.hpp"
#endif
#endif

#if defined(ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

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

#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
#define REFRESH_PERIOD_ANALOG_INPUT_PIN A3
#else
Servo ServoUnderTest;
#endif

/*
 * CHANGE THESE TO REFLECT THE CORRECT VALUES OF YOUR SERVO
 *
 * 480 to 620 and 2400 to 2500 are the values for some of my sg90 servos determined with EndPositionTest example
 * 544 suits better for standard servos
 */
#define ZERO_DEGREE_VALUE_MICROS 544
//#define ZERO_DEGREE_VALUE_MICROS 620
#define AT_180_DEGREE_VALUE_MICROS 2400

void doSwipe(uint8_t aDegreePerStep);

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
    // Enable servo refresh period potentiometer between A1 and A5
    // initialize the analog pins as an digital outputs.
    pinMode(A1, OUTPUT);
    pinMode(A5, OUTPUT);
    digitalWrite(A1, LOW);
    digitalWrite(A5, HIGH);
#endif

    Serial.println(F("Value for 0 degree=" STR(ZERO_DEGREE_VALUE_MICROS) "us. Value for 180 degree=" STR(AT_180_DEGREE_VALUE_MICROS) "us."));

    /*
     * Attach servo to pin 9
     * Set the servo to 90 degree for 3 seconds and show this by lighting the internal LED.
     */
#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
    write9(90);
#else
    ServoUnderTest.attach(SERVO_UNDER_TEST_PIN, ZERO_DEGREE_VALUE_MICROS, AT_180_DEGREE_VALUE_MICROS);
    ServoUnderTest.write(90);
#endif
    digitalWrite(LED_BUILTIN, HIGH);
    delay(3000);
    digitalWrite(LED_BUILTIN, LOW);
}

void printVCCAndMode(int aVCCMillivolt, uint8_t aMode) {
    Serial.print(F("VCC="));
    Serial.print(aVCCMillivolt);
    Serial.print(F(" mV"));
    Serial.print(F(" Mode="));
    Serial.print(aMode);
    Serial.print(' ');
}

void loop() {
    int tPulseMicros;
    static int sLastPulseMicros;

#if defined(__AVR__)
    int tVoltageMillivolts = getVCCVoltageMillivolt();
#else
    int tVoltageMillivolts = 3333; // Dummy value
#endif

    // required to switch ADC reference
    analogRead(MODE_ANALOG_INPUT_PIN);
    delay(5);
    int tMode = analogRead(MODE_ANALOG_INPUT_PIN);

    int tSpeedOrPosition = analogRead(SPEED_OR_POSITION_ANALOG_INPUT_PIN);

#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
    /*
     * Set refresh period from 2.5 to 20 ms
     */
    int tPeriod = analogRead(REFRESH_PERIOD_ANALOG_INPUT_PIN);
    setLightweightServoRefreshRate(map(tPeriod, 0, 1023, 2500, 20000));
#endif

    tMode = tMode >> 7; // gives values 0-7
    tPulseMicros = map(tSpeedOrPosition, 0, 1023, ZERO_DEGREE_VALUE_MICROS - 150, AT_180_DEGREE_VALUE_MICROS + 200);

    /*
     * Avoid print for mode 0 if nothing changed
     */
    if (tMode != 0) {
        printVCCAndMode(tVoltageMillivolts, tMode);
    } else if (abs(sLastPulseMicros - tPulseMicros) > 3) {
        printVCCAndMode(tVoltageMillivolts, tMode);
    }

    switch (tMode) {
    case 0:
        // direct position from 0 to 180 degree. We choose bigger range just to be sure both ends are reached.
#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
        writeMicroseconds9(tPulseMicros);
#else
        ServoUnderTest.writeMicroseconds(tPulseMicros);
#endif
        /*
         * Avoid print if nothing changed
         */
        if (abs(sLastPulseMicros - tPulseMicros) > 3) {
            sLastPulseMicros = tPulseMicros;

            Serial.print(F("direct position: micros="));
            Serial.print(tPulseMicros);
            Serial.print(F(" degree="));
#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
            Serial.print(MicrosecondsToDegreeLightweightServo(tPulseMicros));
            Serial.print(F(" refresh period="));
            Serial.print(ICR1 / 2000);
            Serial.print('.');
            Serial.print((ICR1 / 200) % 10);
            Serial.print(F("ms"));
#else
            Serial.print(ServoUnderTest.read());
#endif
            Serial.println();
        }
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
        tSpeedOrPosition = (((1023 - tSpeedOrPosition) >> 7) * 20) + 20; // gives values 20, 40 to 160
        Serial.print(F("fast reaction: 85 -> 90 -> 95 -> 90 delay="));
        Serial.println(tSpeedOrPosition);

#ifndef USE_LEIGHTWEIGHT_SERVO_LIB
        ServoUnderTest.write(85);
        delay(tSpeedOrPosition);
        ServoUnderTest.write(90);
        delay(tSpeedOrPosition);
        ServoUnderTest.write(95);
        delay(tSpeedOrPosition);
        ServoUnderTest.write(90);
        delay(tSpeedOrPosition);
#endif
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
    Serial.print(F(", delay="));
    Serial.print(tDelayMillis);
    Serial.print(F(" ms"));
#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
    Serial.print(F(", refresh period="));
    Serial.print(ICR1 / 2000);
    Serial.print('.');
    Serial.print((ICR1 / 200) % 10);
    Serial.print(F(" ms"));
#endif
    Serial.println();

    uint8_t tDegree = 0;
    while (tDegree < 180) {
#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
        write9(tDegree);
#else
        ServoUnderTest.write(tDegree); // starts with 0
#endif
        // read again to enable fast reaction to changed value
        delay(readDelay(aDegreePerStep));
        tDegree += aDegreePerStep;
    }
    while (tDegree > 0) {
#ifdef USE_LEIGHTWEIGHT_SERVO_LIB
        write9(tDegree);
#else
        ServoUnderTest.write(tDegree);
#endif
        // read again to enable fast reaction to changed value
        delay(readDelay(aDegreePerStep));
        tDegree -= aDegreePerStep;
    }
}
