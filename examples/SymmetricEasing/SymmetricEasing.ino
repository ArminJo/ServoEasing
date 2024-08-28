/*
 * SymmetricEasing.cpp
 *
 *  Shows symmetric (end movement is mirror of start movement) linear, quadratic and cubic movements for 3 servos synchronously.
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
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
//#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
//#define MAX_EASING_SERVOS 3
//#define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle) parameter. Saves 128 bytes program memory.
//#define DISABLE_MIN_AND_MAX_CONSTRAINTS    // Activating this disables constraints. Saves 4 bytes RAM per servo but strangely enough no program memory.
//#define DISABLE_PAUSE_RESUME               // Activating this disables pause and resume functions. Saves 5 bytes RAM per servo.
//#define DEBUG                              // Activating this enables generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER           // Activating this enables generate the Arduino plotter output from ServoEasing.hpp.

/*
 * Specify which easings types should be available.
 * If no easing is defined, all easings are active.
 * This must be done before the #include "ServoEasing.hpp"
 */
#define ENABLE_EASE_QUADRATIC
#define ENABLE_EASE_CUBIC
//#define ENABLE_EASE_QUARTIC
#define ENABLE_EASE_SINE
//#define ENABLE_EASE_CIRCULAR
//#define ENABLE_EASE_BACK
//#define ENABLE_EASE_ELASTIC
//#define ENABLE_EASE_BOUNCE
//#define ENABLE_EASE_PRECISION
//#define ENABLE_EASE_USER

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

#define START_DEGREE_VALUE 90

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

#if defined(ESP32)
    analogReadResolution(10);
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
    if (Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE,
    DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
    }

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Attach servo at pin " STR(SERVO2_PIN)));
#endif
    if (Servo2.attach(SERVO2_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE,
    DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
    }

    /*
     * Check at least the last call to attach()
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Attach servo at pin " STR(SERVO3_PIN)));
#endif
    if (Servo3.attach(SERVO3_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE,
    DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }

    // Wait for servos to reach start position.
    delay(2000);

#if defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(); // end of line of attach values
#else
    Serial.println(F("Move from 90 to 45 degree in 1 second"));
#endif
    Servo1.startEaseToD(45, 1000, START_UPDATE_BY_INTERRUPT);
    Servo2.startEaseToD(45, 1000, START_UPDATE_BY_INTERRUPT);
    Servo3.startEaseToD(45, 1000, START_UPDATE_BY_INTERRUPT);
    delay(1000);

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Set easing type to LINEAR, QUADRATIC_IN_OUT and CUBIC_IN_OUT for Servos 1 to 3"));
#else
    Serial.println("Linear Quadratic Cubic"); // Print legend
#endif
    Servo1.setEasingType(EASE_LINEAR);
    Servo2.setEasingType(EASE_QUADRATIC_IN_OUT);
    Servo3.setEasingType(EASE_CUBIC_IN_OUT);

//#if !defined(PRINT_FOR_SERIAL_PLOTTER)
//    Serial.println(F("Set easing type to QUADRATIC_IN_OUT, CUBIC_IN_OUT and SINE_IN_OUT for Servos 1 to 3"));
//#else
//    Serial.println("Quadratic, Cubic, Sine"); // Print legend
//#endif
//    Servo1.setEasingType(EASE_QUADRATIC_IN_OUT);
//    Servo2.setEasingType(EASE_CUBIC_IN_OUT);
//    Servo3.setEasingType(EASE_SINE_IN_OUT);

    delay(500);
}

void loop() {

    uint16_t tSpeed = analogRead(SPEED_IN_PIN);
#if defined(__STM32F1__)
    tSpeed = map(tSpeed, 0, 4096, 5, 150); // 12 bit ADC
#else
    tSpeed = map(tSpeed, 0, 1023, 5, 150);
#endif
    setSpeedForAllServos(tSpeed);

    /*
     * Move three servos synchronously without interrupt handler
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.print(F("Move to 135 degree with "));
    Serial.print(Servo1.getSpeed());
    Serial.println(F(" degree per second with with updates by own do-while loop"));
#endif
    /*
     * Here we use the allServos functions
     */
    setIntegerDegreeForAllServos(3, 135, 135, 135);
    setEaseToForAllServos();
    synchronizeAllServosAndStartInterrupt(false); // false, since we call updateAllServos() manually below

    do {
        // here you can call your own program
        delay(REFRESH_INTERVAL_MILLIS); // optional 20ms delay
    } while (!updateAllServos());

    /*
     * Move three servos synchronously with interrupt handler
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.print(F("Move to 45 degree with "));
    Serial.print(Servo1.getSpeed());
    Serial.println(F(" degree per second using interrupts"));
#endif
    Servo1.setEaseTo(45);
    Servo2.setEaseToD(45, Servo1.mMillisForCompleteMove);
    Servo3.startEaseToD(45, Servo1.mMillisForCompleteMove);
    /*
     * No need to call synchronizeAllServosAndStartInterrupt(), since I know that all durations are the same
     * Since all servos stops at the same time I have to check only one
     * Call yield for the ESP boards must be handled in areInterruptsActive()
     */
    while (ServoEasing::areInterruptsActive()) {
        ; // no delays here to avoid break between forth and back movement
    }

}
