/*
 * PCA9685_ExpanderFor32Servos.cpp
 *
 *  Shows smooth linear movement from one servo position to another for 32 servos at two PCA9685 expander boards.
 *  The PCA9685 library was successfully tested with up to 3 expander boards.
 *
 *  *****************************************************************************************************************************
 *  !!! Activate the line "#define USE_PCA9685_SERVO_EXPANDER" in ServoEasing.h to make the expander example work !!!
 *  Otherwise you will see errors like: "PCA9685_Expander.cpp:88:5: error: 'Wire' was not declared in this scope"
 *  or "no matching function for call to 'ServoEasing::ServoEasing(int&, TwoWire*)'"
 *
 *  To access the library files from your sketch, you have to first use `Sketch > Show Sketch Folder (Ctrl+K)` in the Arduino IDE.
 *  Then navigate to the parallel `libraries` folder and select the library you want to access.
 *  The library files itself are located in the `src` sub-directory.
 *  If you did not yet store the example as your own sketch, then with Ctrl+K you are instantly in the right library folder.
 *  *****************************************************************************************************************************
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

// Must specify this before the include of "ServoEasing.hpp"
#define USE_PCA9685_SERVO_EXPANDER // Activate this to enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB // Activate this to force additional using of regular servo library.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT // Activate this to disable all but LINEAR movement. Saves up to 1540 bytes FLASH.
//#define DISABLE_COMPLEX_FUNCTIONS // Activate this to disable the SINE, CIRCULAR, BACK, ELASTIC and BOUNCE easings. Saves up to 1850 bytes FLASH.
#define MAX_EASING_SERVOS 32
//#define ENABLE_MICROS_AS_DEGREE_PARAMETER // Activate this to enable also microsecond values as (target angle) parameter. Requires additional 128 Bytes FLASH.
//#define DEBUG // Activate this to generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER // Activate this to generate the Arduino plotter output
#include "ServoEasing.hpp"

//#define DEBUG // to see all ServoEasing's object info
#define INFO // to see serial output of loop

//#define USE_ONLY_ONE_EXPANDER // Activate this to reuse this example for one expander at PCA9685_DEFAULT_ADDRESS

// for ESP32 LED_BUILTIN is defined as static const uint8_t LED_BUILTIN = 2;
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif
// On the Zero and others we switch explicitly to SerialUSB
#if defined(ARDUINO_ARCH_SAMD)
#define Serial SerialUSB
#endif

#define FIRST_PCA9685_EXPANDER_ADDRESS PCA9685_DEFAULT_ADDRESS
#if !defined(USE_ONLY_ONE_EXPANDER)
#define SECOND_PCA9685_EXPANDER_ADDRESS (PCA9685_DEFAULT_ADDRESS + 0x20) // Bridged A5 on the board -> 0x60
#endif

#define NUMBER_OF_SERVOS MAX_EASING_SERVOS

void getAndAttach16ServosToPCA9685Expander(uint8_t aPCA9685I2CAddress);
#if defined (SP)
uint16_t getFreeRam(void);
#endif

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));

    Serial.println(F("Example for a maximum of " STR(NUMBER_OF_SERVOS) " servos"));
    Serial.println();

    // Initialize wire before checkI2CConnection()
    Wire.begin();  // Starts with 100 kHz. Clock will be increased at first attach() except for ESP32.
#if defined (ARDUINO_ARCH_AVR) // Other platforms do not have this new function
    Wire.setWireTimeout(); // Sets default timeout of 25 ms.
#endif
    checkI2CConnection(FIRST_PCA9685_EXPANDER_ADDRESS, &Serial);
    getAndAttach16ServosToPCA9685Expander(FIRST_PCA9685_EXPANDER_ADDRESS);

#if !defined(USE_ONLY_ONE_EXPANDER)
    if (checkI2CConnection(SECOND_PCA9685_EXPANDER_ADDRESS, &Serial)) {
        Serial.println(F("Second PCA9685 expander not connected -> only 16 servos initialized"));
    } else {
        getAndAttach16ServosToPCA9685Expander(SECOND_PCA9685_EXPANDER_ADDRESS);
    }
#endif

    /**************************************************
     * Set servos to start position.
     * This is the position where the movement starts.
     *************************************************/
    writeAllServos(0);

#ifdef DEBUG
    for (uint8_t i = 0; i <= ServoEasing::sServoArrayMaxIndex; ++i) {
        ServoEasing::ServoEasingArray[i]->print(&Serial);
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
    Serial.print((180 * (1000L / 20)) / (ServoEasing::sServoArrayMaxIndex + 1));
    Serial.println(F(" ms delay"));
#endif
    setSpeedForAllServos(20);  // This speed is taken if no further speed argument is given.
    for (uint8_t i = 0; i <= ServoEasing::sServoArrayMaxIndex; ++i) {
        ServoEasing::ServoEasingArray[i]->startEaseTo(180);
        /*
         * Choose delay so that the last servo starts when the first is about to end
         */
        delay((180 * (1000L / 20)) / (ServoEasing::sServoArrayMaxIndex + 1));
    }
    delay(1000);

    // Now move back
#ifdef INFO
    Serial.println(F("Move all back to 0 degree with 20 degree per second"));
#endif
    for (uint8_t i = 0; i <= ServoEasing::sServoArrayMaxIndex; ++i) {
        ServoEasing::ServoEasingArray[i]->startEaseTo(0);
#ifdef DEBUG
        Serial.print(F("Start i="));
        Serial.println(i);
#endif
        /*
         * Choose delay so that the last servo starts when the first is about to end
         */
        delay((180 * (1000L / 20)) / (ServoEasing::sServoArrayMaxIndex + 1));
    }

    delay(1000);
}

/*
 * Get the 16 ServoEasing objects for the PCA9685 expander
 * The attach() function inserts them in the ServoEasing::ServoEasingArray[] array.
 */
void getAndAttach16ServosToPCA9685Expander(uint8_t aPCA9685I2CAddress) {
    ServoEasing *tServoEasingObjectPtr;

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
            Serial.println(
                    F(
                            " Error attaching servo - maybe MAX_EASING_SERVOS=" STR(MAX_EASING_SERVOS) " is to small to hold all servos"));

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
