/*
 *  CatMover.cpp
 *  Demo of using two servos in a pan tilt housing to move a laser pointer
 *
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

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#include "ServoEasing.h"

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
// For ATmega328 pins 9 + 10 are connected to timer 2 and can therefore be used also by the Lightweight Servo library
const int HORIZONTAL_SERVO_PIN = SERVO1_PIN;
const int VERTICAL_SERVO_PIN = SERVO2_PIN;

const int LASER_POWER_PIN = SPEED_IN_PIN;

struct ServoControlStruct {
    uint16_t minDegree;
    uint16_t maxDegree;
};
ServoControlStruct ServoHorizontalControl;
ServoControlStruct ServoVerticalControl;

ServoEasing ServoHorizontal;
ServoEasing ServoVertical;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(2000); // To be able to connect Serial monitor after reset or power up and before first printout
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));

    // initialize the digital pin as an output.
    pinMode(LASER_POWER_PIN, OUTPUT);
    digitalWrite(LASER_POWER_PIN, HIGH);

    /*
     * Set up servos
     */
    Serial.print(F("Attach servo at pin "));
    Serial.println(HORIZONTAL_SERVO_PIN);
    if (ServoHorizontal.attach(HORIZONTAL_SERVO_PIN) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
    }

    /*
     * Check at least the last call to attach()
     */
    Serial.print(F("Attach servo at pin "));
    Serial.println(VERTICAL_SERVO_PIN);
    if (ServoVertical.attach(VERTICAL_SERVO_PIN) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }

    ServoHorizontalControl.minDegree = 45;
    ServoHorizontalControl.maxDegree = 135;
    ServoVerticalControl.minDegree = 0;
    ServoVerticalControl.maxDegree = 45;

    /**************************************************
     * Set servos to start position.
     * This is the position where the movement starts.
     *************************************************/
    // This values helps mounting the pan / tilt housing
    ServoHorizontal.write(90);
    ServoVertical.write(90);

    delay(4000);

    /*
     * show border of area which can be reached by laser
     */
    Serial.println(F("Mark border of area and then do auto move."));
    ServoHorizontal.write(ServoHorizontalControl.minDegree);
    ServoVertical.write(ServoVerticalControl.minDegree);
    delay(500);
    analogWrite(LASER_POWER_PIN, 255);
    ServoHorizontal.easeTo(ServoHorizontalControl.maxDegree, 50);
    ServoVertical.easeTo(ServoVerticalControl.maxDegree, 50);
    ServoHorizontal.easeTo(ServoHorizontalControl.minDegree, 50);
    ServoVertical.easeTo(ServoVerticalControl.minDegree, 50);
}

uint8_t getRandomValue(ServoControlStruct *aServoControlStruct, ServoEasing *aServoEasing) {
    /*
     * get new different value
     */
    uint8_t tNewTargetAngle;
    do {
        tNewTargetAngle = random(aServoControlStruct->minDegree, aServoControlStruct->maxDegree);
    } while (tNewTargetAngle == aServoEasing->MicrosecondsOrUnitsToDegree(aServoEasing->mCurrentMicrosecondsOrUnits)); // do not accept current angle as new value
    return tNewTargetAngle;
}

void loop() {

    if (!ServoHorizontal.isMoving()) {
        /*
         * start new random move in the restricted area
         */
        delay(random(500));
        uint8_t tNewHorizontal = getRandomValue(&ServoHorizontalControl, &ServoHorizontal);
        uint8_t tNewVertical = getRandomValue(&ServoVerticalControl, &ServoVertical);
        int tSpeed = random(10, 90);

        Serial.print(F("Move to horizontal="));
        Serial.print(tNewHorizontal);
        Serial.print(F(" vertical="));
        Serial.print(tNewVertical);
        Serial.print(F(" speed="));
        Serial.println(tSpeed);
        ServoHorizontal.setEaseTo(tNewHorizontal, tSpeed);
        ServoVertical.setEaseTo(tNewVertical, tSpeed);
        synchronizeAllServosAndStartInterrupt();
    } else {
        /*
         * Do whatever you want in his part of the loop
         */

    }
}
