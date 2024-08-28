/*
 *  CatMover.cpp
 *  Demo of using two servos in a pan tilt housing to move a laser pointer
 *
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

// Must specify this before the include of "ServoEasing.hpp"
//#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library.
//#define USE_LEIGHTWEIGHT_SERVO_LIB    // Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
//#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
//#define MAX_EASING_SERVOS 3
//#define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle) parameter. Saves 128 bytes program memory.
//#define DISABLE_MIN_AND_MAX_CONSTRAINTS    // Activating this disables constraints. Saves 4 bytes RAM per servo but strangely enough no program memory.
//#define DISABLE_PAUSE_RESUME               // Activating this disables pause and resume functions. Saves 5 bytes RAM per servo.
//#define DEBUG                              // Activating this enables generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER           // Activating this enables generate the Arduino plotter output from ServoEasing.hpp.
#include "ServoEasing.hpp"
#include "PinDefinitionsAndMore.h"

/*
 * Pin mapping table for different platforms - used by all examples
 *
 * Platform         Servo1      Servo2      Servo3      Analog     Core/Pin schema
 * -------------------------------------------------------------------------------
 * (Mega)AVR + SAMD    9          10          11          A0
 * ATtiny3217         20|PA3       0|PA4       1|PA5       2|PA6   MegaTinyCore
 * ESP8266            14|D5       12|D6       13|D7        0
 * ESP32               5          18          19          A0
 * BluePill          PB7         PB8         PB9         PA0
 * APOLLO3            11          12          13          A3
 * RP2040             6|GPIO18     7|GPIO19    8|GPIO20
 */
// For ATmega328 pins 9 + 10 are connected to timer 2 and can therefore be used also by the Lightweight Servo library
const int HORIZONTAL_SERVO_PIN = SERVO1_PIN;
const int VERTICAL_SERVO_PIN = SERVO2_PIN;

const int LASER_POWER_PIN = SPEED_IN_PIN;

// Holds max and min values for the servo / laser range
struct ServoControlStruct {
    int minDegree;
    int maxDegree;
};
ServoControlStruct ServoHorizontalControl;
ServoControlStruct ServoVerticalControl;

ServoEasing ServoHorizontal;
ServoEasing ServoVertical;

#define START_DEGREE_VALUE 90 // This values helps mounting the pan / tilt housing

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for Serial to become available. Is optimized away for some cores.

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));

    // initialize the digital pin as an output.
    pinMode(LASER_POWER_PIN, OUTPUT);
    digitalWrite(LASER_POWER_PIN, HIGH);

    /************************************************************
     * Attach servo to pin and set servos to start position.
     * This is the position where the movement starts.
     ***********************************************************/
    Serial.print(F("Attach servo at pin "));
    Serial.println(HORIZONTAL_SERVO_PIN);
    if (ServoHorizontal.attach(HORIZONTAL_SERVO_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
    }

    /*
     * Check at least the last call to attach()
     */
    Serial.print(F("Attach servo at pin "));
    Serial.println(VERTICAL_SERVO_PIN);
    if (ServoVertical.attach(VERTICAL_SERVO_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }

    /*
     * Set the border of the servo/ laser range.
     * This of course depends on the orientation of the pan/tilt mounting.
     */
    ServoHorizontalControl.minDegree = 45;
    ServoHorizontalControl.maxDegree = 135;
    ServoVerticalControl.minDegree = 0;
    ServoVerticalControl.maxDegree = 45;

    delay(4000); // This delay helps mounting the pan / tilt housing

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
