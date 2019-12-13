/*
 * PCA9685_ExpanderFor32Servos.cpp
 *
 *  Shows smooth linear movement from one servo position to another for 32 servos at two PCA9685 expander boards.
 *  The PCA9685 library was successfully tested with up to 3 expander boards.
 *
 *  *****************************************************************************************************************************
 *  !!! Comment out line 40 "#define USE_PCA9685_SERVO_EXPANDER" in ServoEasing.h to make the expander example work !!!
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
 * !!! Comment out line 40 "#define USE_PCA9685_SERVO_EXPANDER" in ServoEasing.h to make the expander example work !!!
 * Otherwise you will see errors like: "PCA9685_Expander:44:46: error: 'Wire' was not declared in this scope"
 * For this example you must also modify MAX_EASING_SERVOS to 32 at line 88 in ServoEasing.h or commenting out
 * the line #define USE_ONLY_ONE_EXPANDER below
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#include "ServoEasing.h"

#define VERSION_EXAMPLE "1.0"

//#define DEBUG // to see all ServoEasing's object info
#define INFO // to see serial output of loop

//#define USE_ONLY_ONE_EXPANDER
#if !defined(USE_ONLY_ONE_EXPANDER) && (MAX_EASING_SERVOS < 32)
#warning "You use at least 2 expanders but MAX_EASING_SERVOS is less than 32, so you have no space for 32 servos (which might be ok, since you must not use 16 servos on one expander)"
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

void checkI2CConnection(uint8_t aI2CAddress);
void getAndAttach16ServosToPCA9685Expander(uint8_t aPCA9685I2CAddress);
uint16_t getFreeRam(void);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__)
	while (!Serial); //delay for Leonardo, but this loops forever for Maple Serial
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    Serial.print(F("Example for a maximum of "));
    Serial.print(NUMBER_OF_SERVOS);
    Serial.println(F(" servos"));
    Serial.println();

    // Initialize wire before checkI2CConnection()
    Wire.begin();  // starts with 100000 Hz
    checkI2CConnection(FIRST_PCA9685_EXPANDER_ADDRESS);
    getAndAttach16ServosToPCA9685Expander(FIRST_PCA9685_EXPANDER_ADDRESS);

#if !defined(USE_ONLY_ONE_EXPANDER)
	checkI2CConnection(SECOND_PCA9685_EXPANDER_ADDRESS);
	getAndAttach16ServosToPCA9685Expander(SECOND_PCA9685_EXPANDER_ADDRESS);
#endif

    /**************************************************
     * Set servos to start position.
     * This is the position where the movement starts.
     *************************************************/
    writeAllServos(0);
#ifdef DEBUG
	for (uint8_t i = 0; i <= sServoArrayMaxIndex; ++i) {
		sServoArray[i]->print(&Serial);
	}
#endif

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
    Serial.print((180 * (1000L / 20)) / (sServoArrayMaxIndex + 1));
    Serial.println(F(" ms delay"));
#endif
    setSpeedForAllServos(20);  // This speed is taken if no further speed argument is given.
    for (uint8_t i = 0; i <= sServoArrayMaxIndex; ++i) {
        sServoArray[i]->startEaseTo(180);
        /*
         * Choose delay so that the last servo starts when the first is about to end
         */
        delay((180 * (1000L / 20)) / (sServoArrayMaxIndex + 1));
    }
    delay(1000);

    // Now move back
#ifdef INFO
    Serial.println(F("Move all back to 0 degree with 20 degree per second"));
#endif
    for (uint8_t i = 0; i <= sServoArrayMaxIndex; ++i) {
        sServoArray[i]->startEaseTo(0);
#ifdef DEBUG
		Serial.print(F("Start i="));
		Serial.println(i);
#endif
        /*
         * Choose delay so that the last servo starts when the first is about to end
         */
        delay((180 * (1000L / 20)) / (sServoArrayMaxIndex + 1));
    }

    delay(1000);
}

/*
 * Check if I2C communication is possible. If not, we will wait forever at endTransmission.
 */
void checkI2CConnection(uint8_t aI2CAddress) {
    Serial.print(F("Try to communicate with I2C device at address=0x"));
    Serial.println(aI2CAddress, HEX);
    Serial.flush();

    Wire.beginTransmission(aI2CAddress);
    if (Wire.endTransmission(true) == 0) {
        Serial.print(F("Found"));
    } else {
        Serial.print(F("ERROR: Communication with I2C was successful, but found no"));
    }
    Serial.print(F(" I2C device attached at address: 0x"));
    Serial.println(aI2CAddress, HEX);
}

/*
 * Get the 16 ServoEasing objects for the PCA9685 expander
 * The attach() function inserts them in the sServoArray[] array.
 */
void getAndAttach16ServosToPCA9685Expander(uint8_t aPCA9685I2CAddress) {
    ServoEasing * tServoEasingObjectPtr;

    Serial.print(F("Get ServoEasing objects and attach servos to PCA9685 expander at address=0x"));
    Serial.println(aPCA9685I2CAddress, HEX);
    for (uint8_t i = 0; i < PCA9685_MAX_CHANNELS; ++i) {
#if defined(ARDUINO_SAM_DUE)
		tServoEasingObjectPtr= new ServoEasing(aPCA9685I2CAddress, &Wire1);
#else
        tServoEasingObjectPtr = new ServoEasing(aPCA9685I2CAddress, &Wire);
#endif
        if (tServoEasingObjectPtr->attach(i) == INVALID_SERVO) {
            Serial.print(F("Address=0x"));
            Serial.print(aPCA9685I2CAddress, HEX);
            Serial.print(F(" i="));
            Serial.print(i);
            Serial.print(F(" ERROR attaching servo - maybe MAX_EASING_SERVOS="));
            Serial.print(MAX_EASING_SERVOS);
            Serial.println(F(" is to small to hold all servos"));
        }
    }
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
