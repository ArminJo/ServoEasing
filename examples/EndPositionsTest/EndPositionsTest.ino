/*
 * EndPositionsTest.cpp
 *
 *  Allows to determine the end position of the servo under test.
 *  It is one of the 8 test modes from the SpeedTest example.
 *
 *  Turn the potentiometer until the servo begins to move and see the serial output for the value at this position.
 *
 *  Some of my SG90 servos have 620 and 2400 micro seconds for 0 and 180 degree at 4.8 volt, others have 544 and 2300 micro seconds at 4.8 volt
 *  My Modelcraft RS-2 servo has 535 and 2400 micro seconds for 0 and 180 degree at 4.8 volt
 *  My M-1504 servo has 545 and 2325 micro seconds for 0 and 180 degree at 4.6 volt
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

#if defined(ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

#if defined(AVR)
#include "ADCUtils.h" // for get getVCCVoltageMillivolt
#endif

#define VERSION_EXAMPLE "1.4"


// Attach the sliding contact of the potentiometer here
#if defined(ESP8266)
const int SERVO_UNDER_TEST_PIN = 14; // D5
const int POSITION_ANALOG_INPUT_PIN = 0;

#elif defined(ESP32)
const int SERVO_UNDER_TEST_PIN = 5;
const int POSITION_ANALOG_INPUT_PIN = 36;

#elif defined(__STM32F1__)
const int SERVO_UNDER_TEST_PIN = PB9; // Needs timer 4 for Servo library
const int SPEED_IN_PIN = PA0;

#else
const int SERVO_UNDER_TEST_PIN = 9;
const int POSITION_ANALOG_INPUT_PIN = A1;
#endif

Servo ServoUnderTest;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__)
    while (!Serial); //delay for Leonardo, but this loops forever for Maple Serial
#endif
#if defined(SERIAL_USB)
    delay(2000); // To be able to connect Serial monitor after reset and before first printout
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    Serial.print(F("Value for 0 degree="));
    Serial.print(MIN_PULSE_WIDTH);
    Serial.print(F("us. Value for 180 degree="));
    Serial.print(MAX_PULSE_WIDTH);
    Serial.println(F("us."));

    // attach servo to pin
    ServoUnderTest.attach(SERVO_UNDER_TEST_PIN);
    // set servo to start position.
    ServoUnderTest.write(0);

    delay(500);

}

void loop() {
    static int sLastPulseMicros;

    int tPosition = analogRead(POSITION_ANALOG_INPUT_PIN);

#if defined(__STM32F1__)
    int tPulseMicros = map(tPosition, 0, 4096, 400, 2500); // 12 bit ADC
#else
    int tPulseMicros = map(tPosition, 0, 1023, 400, 2500);
#endif

    if (sLastPulseMicros != tPulseMicros) {
        sLastPulseMicros = tPulseMicros;
        Serial.print("Micros=");
        Serial.print(tPulseMicros);
        Serial.print(" degree=");
        Serial.print(ServoUnderTest.read());

#if defined(AVR)
        int tVoltageMillivolts = getVCCVoltageMillivolt();
        Serial.print(" VCC=");
        // since the values may depend from the supply voltage, print this value too,
        // but be careful, it may not be exact due to the tolerance of the internal bandgap reference
        Serial.print(tVoltageMillivolts);
        Serial.println(F(" mV"));
#endif

        ServoUnderTest.writeMicroseconds(tPulseMicros);
    }
    delay(100); // multiple of 20
}
