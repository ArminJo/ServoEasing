/*
 * EndPositionsTest.cpp
 *
 *  Allows to determine the end position of the servo under test.
 *  The values can be used in the call of attach(int aPin, int aInitialDegree, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree)
 *  It is one of the 8 test modes from the SpeedTest example.
 *
 *  Turn the potentiometer until the servo begins to move and see the serial output for the value at this position.
 *
 *  Some of my SG90 servos have 620 and 2400 micro seconds for 0 and 180 degree at 4.8 volt, others have 544 and 2300 micro seconds at 4.8 volt
 *  My Modelcraft RS-2 servo has 535 and 2400 micro seconds for 0 and 180 degree at 4.8 volt
 *  My M-1504 servo has 545 and 2325 micro seconds for 0 and 180 degree at 4.6 volt
 *
 *  Copyright (C) 2019-2021  Armin Joachimsmeyer
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

#if defined(__AVR__)
#include "ADCUtils.h" // for get getVCCVoltageMillivolt
#endif

#define VERSION_EXAMPLE "1.4"

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

Servo ServoUnderTest;

#define START_DEGREE_VALUE  0 // The degree value written to the servo at time of attach.

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

    Serial.println(F("Value for 0 degree=" STR(MIN_PULSE_WIDTH) "us. Value for 180 degree=" STR(MAX_PULSE_WIDTH) "us."));

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

#if defined(__AVR__)
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
