/*
 * ThreeServos.cpp
 *
 *  Shows smooth movement from one servo position to another for 3 servos synchronously.
 *  Demonstrates the use of ServoEasingArray and ServoEasingNextPositionArray.
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
//#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library.
//#define USE_USER_PROVIDED_SERVO_LIB   // Use of your own servo library.
#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
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

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;

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
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));
#endif

    /************************************************************
     * Attach servo to pin and set servos to start position.
     * This is the position where the movement starts.
     *
     * The order of the attach() determine the position
     * of the Servos in internal ServoEasing::ServoEasingArray[]
     ***********************************************************/
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Attach servo at pin " STR(SERVO1_PIN)));
#endif
    Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE);

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Attach servo at pin " STR(SERVO2_PIN)));
#endif
    Servo2.attach(SERVO2_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);

    /*
     * Check at least the last call to attach()
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Attach servo at pin " STR(SERVO3_PIN)));
#endif
    if (Servo3.attach(SERVO3_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
        Serial.println(
                F("Error attaching servo - maybe MAX_EASING_SERVOS=" STR(MAX_EASING_SERVOS) " is to small to hold all servos"));
        while (true) {
            blinkLED();
        }
    }

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    /*
     * Print internal servo control data
     */
    Servo1.print(&Serial);
    Servo2.print(&Serial);
    ServoEasing::ServoEasingArray[2]->print(&Serial); // "ServoEasing::ServoEasingArray[2]->" can be used instead of "Servo3."
#endif

#if defined(PRINT_FOR_SERIAL_PLOTTER)
    // Legend for Arduino plotter
    Serial.println(); // end of line of attach values
    Serial.println(F("Servo1, Servo2, Servo3"));
#endif

    // Wait for servos to reach start position.
    delay(500);
    Serial.println(F("Start loop"));

}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

void loop() {
    uint16_t tSpeed = 20;

    /*
     * Move three servos synchronously without interrupt handler
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.print(F("Move to 90/135/180 degree with up to "));
    Serial.print(tSpeed);
    Serial.println(F(" degree per second with updates by own do-while loop"));
#endif
    // this speed is changed for the first 2 servos below by synchronizing to the longest duration
    setSpeedForAllServos(tSpeed);

    ServoEasing::ServoEasingArray[0]->setEaseTo(90);    // This servo uses effectively 10 degrees per second, since it is synchronized to Servo3
    ServoEasing::ServoEasingArray[1]->setEaseTo(135);   // "ServoEasing::ServoEasingArray[1]->" can be used instead of "Servo2."
    Servo3.setEaseTo(180);                              // This servo has the longest distance -> it uses 20 degrees per second
    synchronizeAllServosAndStartInterrupt(false);       // Do not start interrupt, because we use updateAllServos() every 20 ms below

    do {
        // Here you can insert your own code
        /*         */
        delay(REFRESH_INTERVAL_MILLIS); // Optional 20ms delay. Can be less.
    } while (!updateAllServos());

    delay(1000);

    /*
     * Move three servos synchronously with interrupt handler
     */
    tSpeed = 30;
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.print(F("Move to 180/180/0 degree with up to "));
    Serial.print(tSpeed);
    Serial.println(F(" degree per second using interrupts"));

#endif
    ServoEasing::ServoEasingNextPositionArray[0] = 180;
    ServoEasing::ServoEasingNextPositionArray[1] = 180;
    ServoEasing::ServoEasingNextPositionArray[2] = 0;
    setEaseToForAllServosSynchronizeAndStartInterrupt(tSpeed); // Set speed and start interrupt here, we check the end with areInterruptsActive()
    /*
     * Now you can run your program while the servos are moving.
     * Just let the LED blink until servos stop.
     * Since all servos stops at the same time I have to check only one
     */
    while (ServoEasing::areInterruptsActive()) {
        // Here you can insert your own code
        blinkLED();
    }

    delay(1000);

    /*
     * Move first and second servo synchronously with interrupt handler
     */
    tSpeed = 80;
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.print(F("Move to 90/90 degree with "));
    Serial.print(tSpeed);
    Serial.println(F(" degree per second using interrupts"));
#endif
    Servo1.setEaseTo(90, tSpeed);
    Servo2.startEaseToD(90, Servo1.mMillisForCompleteMove);
    // No timing synchronization with synchronizeAllServosAndStartInterrupt() required, because we use startEaseToD with Servo1.mMillisForCompleteMove :-)

    // blink until servo stops
    while (ServoEasing::areInterruptsActive()) {
        // Here you can insert your own code
        blinkLED();
    }

    delay(1000);

    // Move only third servo
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move third to 90 degree with 80 degree per second blocking"));
#endif
    Servo3.easeTo(90, 80);

    delay(1000);

    /*
     * Move all 3 servos independently
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move independently to 0/0/0 degree with 80/40/20 degree per second using interrupts"));
#endif
    Servo1.setEaseTo(0, 80);
    Servo2.setEaseTo(0, 40);
    Servo3.startEaseTo(0, 20); // Start interrupt for all servos. No synchronization here since the servos should move independently.

    // Blink until servos stops
    while (ServoEasing::areInterruptsActive()) {
        // Here you can insert your own code
        blinkLED();
    }

    delay(2000);
}
