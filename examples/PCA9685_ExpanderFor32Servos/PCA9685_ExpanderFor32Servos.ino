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
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

// Must specify this before the include of "ServoEasing.hpp"
#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board.
//#define PCA9685_ACTUAL_CLOCK_FREQUENCY 26000000L // Change it, if your PCA9685 has another than the default 25 MHz internal clock.
//#define USE_SOFT_I2C_MASTER           // Saves 1756 bytes program memory and 218 bytes RAM compared with Arduino Wire
//#define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library.
//#define USE_USER_PROVIDED_SERVO_LIB   // Use of your own servo library.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
//#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
#define MAX_EASING_SERVOS 32
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
//#define LOCAL_DEBUG

//#define USE_ONLY_ONE_EXPANDER // Activating this enables reuse this example for one expander at PCA9685_DEFAULT_ADDRESS

// for ESP32 LED_BUILTIN is defined as static const uint8_t LED_BUILTIN = 2;
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif

#define FIRST_PCA9685_EXPANDER_ADDRESS  PCA9685_DEFAULT_ADDRESS
#if !defined(USE_ONLY_ONE_EXPANDER)
#define SECOND_PCA9685_EXPANDER_ADDRESS (PCA9685_DEFAULT_ADDRESS + 0x20) // Bridged A5 on the board -> 0x60
#endif

#define NUMBER_OF_SERVOS    MAX_EASING_SERVOS

void getAndAttach16ServosToPCA9685Expander(uint8_t aPCA9685I2CAddress);
#if defined (SP)
uint16_t getCurrentFreeHeapOrStack(void);
#endif

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

#if defined(LOCAL_DEBUG)
    for (uint_fast8_t i = 0; i <= ServoEasing::sServoArrayMaxIndex; ++i) {
        ServoEasing::ServoEasingArray[i]->print(&Serial);
    }
#endif

#if defined (SP)
    Serial.print(F("Current free Heap / Stack[bytes]="));
    Serial.println(getCurrentFreeHeapOrStack());
#endif
    // Wait for servos to reach start position.
    delay(500);
}

void loop() {
    Serial.print(F("Move all to 180 degree with 20 degree per second with "));
    Serial.print((180 * (1000L / 20)) / (ServoEasing::sServoArrayMaxIndex + 1));
    Serial.println(F(" ms delay"));
    setSpeedForAllServos(20);  // This speed is taken if no further speed argument is given.
    for (uint_fast8_t i = 0; i <= ServoEasing::sServoArrayMaxIndex; ++i) {
        ServoEasing::ServoEasingArray[i]->startEaseTo(180);
#if defined(LOCAL_DEBUG)
        Serial.print(F("Start i="));
        Serial.println(i);
        ServoEasing::ServoEasingArray[i]->print(&Serial);
#endif
        /*
         * Choose delay so that the last servo starts when the first is about to end
         */
        delay((180 * (1000L / 20)) / (ServoEasing::sServoArrayMaxIndex + 1));
    }
    delay(1000);

    // Now move back
    Serial.println(F("Move all back to 0 degree with 20 degree per second"));
    for (uint_fast8_t i = 0; i <= ServoEasing::sServoArrayMaxIndex; ++i) {
        ServoEasing::ServoEasingArray[i]->startEaseTo(0);
#if defined(LOCAL_DEBUG)
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
    for (uint_fast8_t i = 0; i < PCA9685_MAX_CHANNELS; ++i) {
        tServoEasingObjectPtr = new ServoEasing(aPCA9685I2CAddress);
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
extern void *__brkval;

/*
 * Returns actual start of free heap
 * Usage for print:
 Serial.print(F("HeapStart=0x"));
 Serial.println((uintptr_t) getHeapStart(), HEX);
 */
uint8_t* getHeapStart(void) {
    if (__brkval == 0) {
        __brkval = __malloc_heap_start;
    }
    return (uint8_t*) __brkval;
}

/*
 * Get amount of free RAM = current stackpointer - heap end
 */
uint16_t getCurrentFreeHeapOrStack(void) {
    uint16_t tHeapStart = (uint16_t) getHeapStart();
    if (tHeapStart >= SP) {
        return 0;
    }
    return (SP - (uint16_t) getHeapStart());
}

#endif
