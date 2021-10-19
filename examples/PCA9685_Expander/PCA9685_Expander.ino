/*
 * PCA9685_Expander.cpp
 *
 *  Shows smooth linear movement from one servo position to another using PCA9685 expander board.
 *  The PCA9685 library was successfully tested with 3 expander boards.
 *
 *  *****************************************************************************************************************************
 *  !!! Activate the line "#define USE_PCA9685_SERVO_EXPANDER" in ServoEasing.h to make the expander example work !!!
 *  Otherwise you will see errors like: "PCA9685_Expander.cpp:70:46: error: 'Wire' was not declared in this scope"
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
//#define MAX_EASING_SERVOS 3
//#define ENABLE_MICROS_AS_DEGREE_PARAMETER // Activate this to enable also microsecond values as (target angle) parameter. Requires additional 128 Bytes FLASH.
//#define DEBUG // Activate this to generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER // Activate this to generate the Arduino plotter output
#include "ServoEasing.hpp"

#define INFO // to see serial output of loop

/*
 * This example is the OneServo example with only one modification to figure out that there is almost no difference between using the PCA9685 expander or the default Arduino Servo interface.
 * The PCA9685 library was successfully tested with 3 expander boards :-)
 */
const int SERVO1_PIN = 9;

// for ESP32 LED_BUILTIN is defined as static const uint8_t LED_BUILTIN = 2;
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif
// On the Zero and others we switch explicitly to SerialUSB
#if defined(ARDUINO_ARCH_SAMD)
#define Serial SerialUSB
#endif

/*
 * Constructor to specify the expander address (required at least if you use more than one expander board)
 * and to specify the I2C implementation library.
 * This can be done for each servo separately, but you can not (yet) mix the 3 different
 * Servo implementation libraries (Arduino Servo, Lightweight Servo and I2C Expansion Board)
 */
#if defined(ARDUINO_SAM_DUE)
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS, &Wire1); // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS at line 88 in ServoEasing.h
#else
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS, &Wire); // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS at line 88 in ServoEasing.h
#endif

#define START_DEGREE_VALUE  0 // The degree value written to the servo at time of attach.

void blinkLED();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));

    /*
     * Check if I2C communication is possible. If not, we will wait forever at endTransmission.
     */
    Serial.println(F("Try to communicate with PCA9685 Expander by TWI / I2C"));
    Serial.flush();
    // Initialize wire before checkI2CConnection()
    Wire.begin();  // Starts with 100 kHz. Clock will eventually be increased at first attach() except for ESP32.
#if defined (ARDUINO_ARCH_AVR) // Other platforms do not have this new function
    Wire.setWireTimeout(); // Sets default timeout of 25 ms.
#endif
    if (checkI2CConnection(PCA9685_DEFAULT_ADDRESS, &Serial)) {
        Serial.println(F("PCA9685 expander not connected"));
        while (true) {
            blinkLED();
        }
    } else {
        Serial.println(F("Attach servo to port 9 of PCA9685 expander"));
        /************************************************************
         * Attach servo to pin and set servos to start position.
         * This is the position where the movement starts.
         *
         * Check at least the last call to attach()
         ***********************************************************/
        if (Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
            Serial.println(
                    F("Error attaching servo - maybe MAX_EASING_SERVOS=" STR(MAX_EASING_SERVOS) " is to small to hold all servos"));
            while (true) {
                blinkLED();
            }
        }
    }
    // Wait for servos to reach start position.
    delay(500);
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

void loop() {
    // Move slow
#ifdef INFO
    Serial.println(F("Move to 90 degree with 10 degree per second blocking"));
#endif
    Servo1.setSpeed(10);  // This speed is taken if no further speed argument is given.
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_90_DEGREE);
#else
    Servo1.easeTo(90);
#endif

    // Now move faster without any delay between the moves
#ifdef INFO
    Serial.println(F("Move to 180 degree with 30 degree per second using interrupts"));
#endif
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
    Servo1.startEaseTo(DEFAULT_MICROSECONDS_FOR_180_DEGREE, 30);
#else
    Servo1.startEaseTo(180, 30);
#endif
    /*
     * Now you can run your program while the servo is moving.
     * Just let the LED blink for 3 seconds (90 degrees moving by 30 degrees per second).
     */
    for (int i = 0; i < 15; ++i) {
        blinkLED();
    }

    delay(1000);

#ifdef INFO
    Serial.println(F("Move to 45 degree in one second using interrupts"));
#endif
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
    Servo1.startEaseToD((544 + ((2400 - 544) / 4)), 1000);
#else
    Servo1.startEaseToD(45, 1000);
#endif
    // Blink until servo stops
    while (ServoEasing::areInterruptsActive()) {
        blinkLED();
    }

    delay(1000);

#ifdef INFO
    Serial.println(F("Move to 135 degree and back to 45 degree nonlinear in one second each using interrupts"));
#endif
    Servo1.setEasingType(EASE_CUBIC_IN_OUT);

    for (int i = 0; i < 2; ++i) {
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
        Servo1.startEaseToD((544 + (((2400 - 544) / 4) * 3)), 1000);
#else
        Servo1.startEaseToD(135, 1000);
#endif
        // Must call yield here for the ESP boards, since we have no delay called
        while (ServoEasing::areInterruptsActive()) {
            ; // no delays here to avoid break between forth and back movement
        }
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
        Servo1.startEaseToD((544 + ((2400 - 544) / 4)), 1000);
#else
        Servo1.startEaseToD(45, 1000);
#endif
        while (ServoEasing::areInterruptsActive()) {
            ; // no delays here to avoid break between forth and back movement
        }
    }
    Servo1.setEasingType(EASE_LINEAR);

    delay(1000);

    /*
     * The LED goes on if servo reaches 120 degree
     */
#ifdef INFO
    Serial.println(F("Move to 180 degree with 50 degree per second blocking"));
#endif
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
    Servo1.startEaseTo(DEFAULT_MICROSECONDS_FOR_180_DEGREE, 50);
#else
    Servo1.startEaseTo(180, 50);
#endif
    while (Servo1.getCurrentAngle() < 120) {
        delay(20); // just wait until angle is above 120 degree
    }
    digitalWrite(LED_BUILTIN, HIGH);
    while (ServoEasing::areInterruptsActive()) {
        ; // wait for servo to stop
    }

    delay(1000);

    /*
     * Very fast move. The LED goes off when servo theoretical reaches 90 degree
     */
#ifdef INFO
    Serial.println(F("Move from 180 to 0 degree with 360 degree per second using interrupts of Timer1"));
#endif
#ifdef ENABLE_MICROS_AS_DEGREE_PARAMETER
    Servo1.startEaseTo(DEFAULT_MICROSECONDS_FOR_0_DEGREE, 360, true);
#else
    Servo1.startEaseTo(0, 360, true);
#endif
    // Wait for 250 ms. The servo should have moved 90 degree.
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);

    /*
     * Demonstrate stop and continue in the middle of a movement
     */
    Servo1.stop();
#ifdef INFO
    Serial.println(F("Stop for 1 second at 90 degree"));
#endif
    delay(1000);
    // continue movement using interrupts
    Servo1.continueWithInterrupts();

    delay(1000);
}
