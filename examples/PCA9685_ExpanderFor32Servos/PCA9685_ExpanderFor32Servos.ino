/*
 * PCA9685_ExpanderFor32Servos.cpp
 *
 *  Shows smooth linear movement from one servo position to another for 32 servos at two PCA9685 expander boards.
 *  The PCA9685 library was successfully tested with 3 expander boards.
 *
 *  *****************************************************************************************************************************
 *  !!! Comment out line 36 / "#define USE_PCA9685_SERVO_EXPANDER" in ServoEasing.h to make the expander example work !!!
 *  Otherwise you will see errors like: "PCA9685_Expander:44:46: error: 'Wire' was not declared in this scope"
 *
 *  To access the library files from your sketch, you have to first use `Sketch/Show Sketch Folder (Ctrl+K)` in the Arduino IDE.
 *  Then navigate to the parallel `libraries` folder and select the library you want to access.
 *  The library files itself are located in the `src` sub-directory.
 *  If you did not yet store the example as your own sketch, then with Ctrl+K you are instantly in the right library folder.
 *  *****************************************************************************************************************************
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

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!! Comment out line 36 / "#define USE_PCA9685_SERVO_EXPANDER" in ServoEasing.h to make the expander example work !!!
 * Otherwise you will see errors like: "PCA9685_Expander:44:46: error: 'Wire' was not declared in this scope"
 * For this example you must also modify MAX_EASING_SERVOS to 32 at line 67 in ServoEasing.h or commenting out
 * the line #define USE_ONLY_ONE_EXPANDER below
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#include "ServoEasing.h"

#define VERSION_EXAMPLE "1.0"

//#define DEBUG // to see all ServoEasing's object info
#define INFO // to see serial output of loop

//#define USE_ONLY_ONE_EXPANDER
#if !defined(USE_ONLY_ONE_EXPANDER) && (MAX_EASING_SERVOS < 32)
#warning "You use at least 2 expanders but you have no space for 32 servos (which might be ok, since you must not use 16 servos on one expander)"
#endif

// for ESP32 LED_BUILTIN is defined as static const uint8_t LED_BUILTIN = 2;
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif

#define FIRST_PCA9685_EXPANDER_ADDRESS PCA9685_DEFAULT_ADDRESS
#if !defined(USE_ONLY_ONE_EXPANDER)
#define SECOND_PCA9685_EXPANDER_ADDRESS (PCA9685_DEFAULT_ADDRESS + 0x20) // Bridged A5 on the board -> 0x60
#endif

#define NUMBER_OF_SERVOS MAX_EASING_SERVOS

uint16_t getFreeRam(void);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__)
    while (!Serial); //delay for Leonardo, but this loops forever for Maple Serial
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    Serial.print(F("Example for "));
    Serial.print(NUMBER_OF_SERVOS);
    Serial.println(F(" servos"));
    Serial.println();

    ServoEasing * tServoEasingObjectPtr;

    /*
     * Check if I2C communication is possible. If not, we will wait forever at endTransmission.
     */
    Serial.println(F("Try to communicate with PCA9685 Expander by TWI / I2C"));
    Serial.flush();
    Wire.beginTransmission(FIRST_PCA9685_EXPANDER_ADDRESS);
    if (Wire.endTransmission(true) == 0) {
        Serial.print(F("Found"));
    } else {
        Serial.print(F("Error: Communication with I2C was successful, but found no"));
    }
    Serial.print(F(" I2C device attached at address: 0x"));
    Serial.println(FIRST_PCA9685_EXPANDER_ADDRESS, HEX);

    /*
     * Get the 16 ServoEasing objects for the first expander
     * The attach() function inserts them in the sServoArray[] array.
     */
    Serial.println(F("Get ServoEasing objects and attach servos to first PCA9685 expander"));
    for (uint8_t i = 0; i < PCA9685_MAX_CHANNELS; ++i) {
#if defined(ARDUINO_SAM_DUE)
        tServoEasingObjectPtr= new ServoEasing(FIRST_PCA9685_EXPANDER_ADDRESS, &Wire1);
#else
        tServoEasingObjectPtr = new ServoEasing(FIRST_PCA9685_EXPANDER_ADDRESS, &Wire);
#endif
        if (tServoEasingObjectPtr->attach(i) == INVALID_SERVO) {
            Serial.print(F("i="));
            Serial.print(i);
            Serial.print(F(" Error attaching servo - maybe MAX_EASING_SERVOS="));
            Serial.print(MAX_EASING_SERVOS);
            Serial.println(F(" is to small to hold all servos"));
        }
    }

#if !defined(USE_ONLY_ONE_EXPANDER)

    Wire.beginTransmission(SECOND_PCA9685_EXPANDER_ADDRESS);
    if (Wire.endTransmission(true) == 0) {
        Serial.print(F("Found"));
    } else {
        Serial.print(F("Error: Found no"));
    }
    Serial.print(F(" I2C device at address: 0x"));
    Serial.println(SECOND_PCA9685_EXPANDER_ADDRESS, HEX);

    /*
     * Get the 16 ServoEasing objects for the second expander
     * The attach() function inserts them in the sServoArray[] array.
     */
    Serial.println(F("Get ServoEasing objects and attach servos to second PCA9685 expander"));
    for (uint8_t i = 0; i < PCA9685_MAX_CHANNELS; ++i) {
#if defined(ARDUINO_SAM_DUE)
        tServoEasingObjectPtr = new ServoEasing(SECOND_PCA9685_EXPANDER_ADDRESS, &Wire1);
#else
        tServoEasingObjectPtr = new ServoEasing(SECOND_PCA9685_EXPANDER_ADDRESS, &Wire);
#endif
        if (tServoEasingObjectPtr->attach(i) == INVALID_SERVO) {
            Serial.print(F("i="));
            Serial.print(i);
            Serial.print(F(" Error attaching servo - maybe MAX_EASING_SERVOS="));
            Serial.print(MAX_EASING_SERVOS);
            Serial.println(F(" is to small to hold all servos"));
        }
    }
#endif

    /**************************************************
     * Set servos to start position.
     * This is the position where the movement starts.
     *************************************************/
    for (uint8_t i = 0; i < NUMBER_OF_SERVOS; ++i) {
        sServoArray[i]->write(0);
#ifdef DEBUG
        sServoArray[i]->print(&Serial);
#endif
    }

#if defined (SP)
    Serial.print(F("Free Ram/Stack[bytes]="));
    Serial.println(getFreeRam());
#endif
    // Wait for servos to reach start position.
    delay(500);
}

void loop() {
#ifdef INFO
    Serial.print(F("Move all to 180 degree with 20 degree per second with "));
    Serial.print((180 * (1000L / 20)) / NUMBER_OF_SERVOS);
    Serial.println(F(" ms delay"));
#endif
    setSpeedForAllServos(20);  // This speed is taken if no further speed argument is given.
    for (uint8_t i = 0; i < NUMBER_OF_SERVOS; ++i) {
        sServoArray[i]->startEaseTo(180);
        /*
         * Choose delay so that the last servo starts when the first is about to end
         */
        delay((180 * (1000L / 20)) / NUMBER_OF_SERVOS);
    }
    delay(1000);

    // Now move back
#ifdef INFO
    Serial.println(F("Move all back to 0 degree with 20 degree per second"));
#endif
    for (uint8_t i = 0; i < NUMBER_OF_SERVOS; ++i) {
        sServoArray[i]->startEaseTo(0);
#ifdef DEBUG
        Serial.print(F("Start i="));
        Serial.println(i);
#endif
        /*
         * Choose delay so that the last servo starts when the first is about to end
         */
        delay((180 * (1000L / 20)) / NUMBER_OF_SERVOS);
    }

    delay(1000);
}

#if defined (SP)
/*
 * Get amount of free RAM = Stack - Heap
 */
uint16_t getFreeRam(void) {
    extern unsigned int __heap_start;
    extern void *__brkval;

    uint16_t tFreeRamBytes;

    if (__brkval == 0) {
        tFreeRamBytes = SP - (int) &__heap_start;
    } else {
        tFreeRamBytes = SP - (int) __brkval;
    }
    return (tFreeRamBytes);
}

#endif
