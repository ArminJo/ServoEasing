/*
 * TwoServos.cpp
 *
 *  Shows smooth movement from one servo position to another for 2 servos synchronously.
 *  It operates the first servo from -90 to +90 degree using setTrim(90).
 *  This example uses the LightweightServo library. This saves 640 bytes program memory compared to using Arduino Servo library.
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

// Must specify this before the include of "ServoEasing.hpp"
//#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library.
#define USE_LEIGHTWEIGHT_SERVO_LIB    // Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
//#define MAX_EASING_SERVOS 2
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

#define START_DEGREE_VALUE  0 // The degree value written to the servo at time of attach.

void blinkLED();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));

    /************************************************************
     * Attach servo to pin and set servos to start position.
     * This is the position where the movement starts.
     *
     * The order of the attach() determine the position
     * of the Servos in internal ServoEasing::ServoEasingArray[]
     * and ServoEasing::ServoEasingNextPositionArray[]
     ***********************************************************/
    Serial.println(F("Attach servo at pin " STR(SERVO1_PIN)));
#endif
    /*
     * Operate Servo1 from -90 to +90 degree
     * Instead of specifying a trim you can use:
     *    Servo1.attach(SERVO1_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, -90, 90);
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Operate servo 1 from -90 to + 90 degree by using attachWithTrim()"));
#endif
    Servo1.attachWithTrim(SERVO1_PIN, 90, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    /*
     * Check at least the last call to attach()
     */
    Serial.println(F("Attach servo at pin " STR(SERVO2_PIN)));
#endif
    if (Servo2.attach(SERVO2_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
        while (true) {
            blinkLED();
        }
    }

#if defined(PRINT_FOR_SERIAL_PLOTTER)
    // Print legend for Plotter
    Serial.println("Servo1[us], Servo2[us]");
#endif
    setSpeedForAllServos(30);

    // Just wait for servos to reach position.
    delay(500);
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

void loop() {

    /*
     * Move both servos blocking
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move to 0/90 degree with 30 degree per second blocking"));
#endif
    setSpeedForAllServos(30);
    Servo1.setEaseTo(0.0f); // Use x.y with trailing f (to specify a floating point constant) to avoid compiler errors.
    Servo2.setEaseTo(90.0f);
    synchronizeAllServosStartAndWaitForAllServosToStop();

    /*
     * Now continue faster.
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move to 90/10 degree with up to 60 degree per second using interrupts"));
#endif
    Servo1.setEaseTo(90.0f, 60);
    /*
     * An alternative method to synchronize and start
     * Synchronize by simply using the same duration
     */
    Servo2.startEaseToD(10, Servo1.mMillisForCompleteMove); // This starts interrupt for all servos
    /*
     * Now you can run your program while the servos are moving.
     * Just let the LED blink until servos stop.
     */
    while (ServoEasing::areInterruptsActive()) {
        blinkLED();
    }

    /*
     * Move servo1 using cubic easing. Use interrupts for update.
     *  The first servo moves with the specified speed.
     *  The second will be synchronized to slower speed (longer duration, than specified) because it has to move only 80 degree.
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move to 0/90 degree with up to 90 degree per second using interrupts. Use cubic easing for first servo."));
#endif
    Servo1.setEasingType(EASE_CUBIC_IN_OUT);
    /*
     * Another method to specify moves
     * Use the ServoEasingNextPositionArray and then call the appropriate function
     */
    ServoEasing::ServoEasingNextPositionArray[0] = 0.0f;
    ServoEasing::ServoEasingNextPositionArray[1] = 90.0f;
    setEaseToForAllServosSynchronizeAndStartInterrupt(90);

    // Must call yield here for the ESP boards, since we have no delay called
    while (ServoEasing::areInterruptsActive()) {
        ;
    }
    Servo1.setEasingType(EASE_LINEAR);

    delay(300);

    /*
     * Move both servos independently
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move independently to -90/0 degree with 60/80 degree per second using interrupts"));
#endif
    Servo1.setEaseTo(-90.0f, 60);
    Servo2.startEaseTo(0.0f, 80); // This start interrupt for all servos
    // blink until both servos stop
    while (ServoEasing::areInterruptsActive()) {
        blinkLED();
    }

    delay(500);

}
