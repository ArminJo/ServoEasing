/*
 * PCA9685_ExpanderAndServo.cpp
 *
 *  Shows smooth linear movement from one servo position to another using one regular connected servo
 *  and one servo connected to PCA9685 expander board.
 *  The PCA9685 library was successfully tested with 3 expander boards.
 *
 *  *****************************************************************************************************************************
 *  !!! Activate the line "#define USE_PCA9685_SERVO_EXPANDER" in ServoEasing.h to make the expander example work !!!
 *  Otherwise you will see errors like: "PCA9685_Expander.cpp:72:46: error: 'Wire' was not declared in this scope"
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
#define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library.
//#define USE_USER_PROVIDED_SERVO_LIB   // Use of your own servo library.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
//#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
//#define MAX_EASING_SERVOS 16
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
#define INFO // to see serial output of loop

/*
 * This example is the OneServo example with only few modifications in setup to figure out that there is almost no difference between using the PCA9685 expander or the default Arduino Servo interface.
 * The PCA9685 library was successfully tested with 3 expander boards :-)
 */

// for ESP32 LED_BUILTIN is defined as static const uint8_t LED_BUILTIN = 2;
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif

/*
 * Constructor to specify the expander address (required at least if you use more than one expander board)
 * and to specify the I2C implementation library.
 * This can be done for each servo separately, but you can not (yet) mix the 3 different
 * Servo implementation libraries (Arduino Servo, Lightweight Servo and I2C Expansion Board)
 */
ServoEasing Servo1AtPCA9685(PCA9685_DEFAULT_ADDRESS); // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS

ServoEasing Servo1;

#define START_DEGREE_VALUE  0 // The degree value written to the servo at time of attach.

void blinkLED();

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

    if (Servo1AtPCA9685.InitializeAndCheckI2CConnection(&Serial)) {
        while (true) {
            blinkLED();
        }
    } else {
        Serial.println(F("Attach servo to port 0 of PCA9685 expander"));
        /************************************************************
         * Attach servo to pin and set servos to start position.
         * This is the position where the movement starts.
         *
         * Attach the expander servos first
         ***********************************************************/
        if (Servo1AtPCA9685.attach(0, START_DEGREE_VALUE) == INVALID_SERVO) {
            Serial.println(
                    F("Error attaching servo - maybe MAX_EASING_SERVOS=" STR(MAX_EASING_SERVOS) " is to small to hold all servos"));
            while (true) {
                blinkLED();
            }
        }
    }

    // Attach servo to pin and set servos to start position.
    Serial.println(F("Attach servo at pin " STR(SERVO1_PIN)));
    if (Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
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
#if defined(INFO)
    Serial.println(F("Move to 90 degree with 10 degree per second blocking"));
#endif
    Servo1.setSpeed(10);  // This speed is taken if no further speed argument is given.
    Servo1AtPCA9685.setSpeed(10);  // This speed is taken if no further speed argument is given.
    Servo1.startEaseTo(90);
    Servo1AtPCA9685.startEaseTo(90);
    updateAndWaitForAllServosToStop();  // blocking wait

    // Now move faster without any delay between the moves
#if defined(INFO)
    Serial.println(F("Move to 180 degree with 30 degree per second using interrupts"));
#endif
    Servo1.startEaseTo(180, 30);
    Servo1AtPCA9685.startEaseTo(180, 30);
    /*
     * Now you can run your program while the servo is moving.
     * Just let the LED blink for 3 seconds (90 degrees moving by 30 degrees per second).
     */
    for (int i = 0; i < 15; ++i) {
        blinkLED();
    }

    delay(1000);

#if defined(INFO)
    Serial.println(F("Move to 135/45 degree in one second using interrupts"));
#endif
    Servo1.startEaseToD(135, 1000);
    Servo1AtPCA9685.startEaseToD(45, 1000);
    // Blink until servo stops
    while (ServoEasing::areInterruptsActive()) {
        blinkLED();
    }

    delay(1000);

#if defined(INFO)
    Serial.println(F("Move to 45/135 degree and back to 135/45 degree nonlinear in one second each using interrupts"));
    Serial.println(F("Move both servos in opposite directions"));
#endif
    Servo1.setEasingType(EASE_CUBIC_IN_OUT);
    Servo1AtPCA9685.setEasingType(EASE_CUBIC_IN_OUT);
    /*
     * Move both servos in opposite directions
     */
    for (int i = 0; i < 2; ++i) {
        Servo1.startEaseToD(45, 1000);
        Servo1AtPCA9685.startEaseToD(135, 1000);
        // Call yield for the ESP boards must be handled in areInterruptsActive()
        while (ServoEasing::areInterruptsActive()) {
            ; // no delays here to avoid break between forth and back movement
        }
        Servo1.startEaseToD(135, 1000);
        Servo1AtPCA9685.startEaseToD(45, 1000);
        while (ServoEasing::areInterruptsActive()) {
            ; // no delays here to avoid break between forth and back movement
        }
    }
    Servo1.setEasingType(EASE_LINEAR);
    Servo1AtPCA9685.setEasingType(EASE_LINEAR);

    delay(1000);

    /*
     * The LED goes on if servo reaches 120 degree
     */
#if defined(INFO)
    Serial.println(F("Move to 180 degree with 50 degree per second blocking"));
#endif
    Servo1.startEaseTo(180, 50);
    Servo1AtPCA9685.startEaseTo(180, 50);
    while (Servo1AtPCA9685.getCurrentAngle() < 120) {
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
#if defined(INFO)
    Serial.println(F("Move from 180 to 0 degree with 360 degree per second using interrupts of Timer1"));
#endif
    Servo1.startEaseTo(0, 360, START_UPDATE_BY_INTERRUPT);
    Servo1AtPCA9685.startEaseTo(0, 360, true);
    // Wait for 250 ms. The servo should have moved 90 degree.
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);

    delay(1000);
}
