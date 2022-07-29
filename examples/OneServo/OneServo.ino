/*
 * OneServo.cpp
 *
 *  Shows smooth linear movement from one servo position to another.
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

// Must specify this before the include of "ServoEasing.hpp"
//#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SOFT_I2C_MASTER           // Saves 1756 bytes program memory and 218 bytes RAM compared with Arduino Wire
//#define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library.
//#define USE_LEIGHTWEIGHT_SERVO_LIB    // Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
#define MAX_EASING_SERVOS 1
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
//#define ENABLE_EASE_QUADRATIC
#define ENABLE_EASE_CUBIC
//#define ENABLE_EASE_QUARTIC
//#define ENABLE_EASE_SINE
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

#if defined(USE_PCA9685_SERVO_EXPANDER)
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS); // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS
#else
ServoEasing Servo1;
#endif

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
#endif

    /********************************************************
     * Attach servo to pin and set servos to start position.
     * This is the position where the movement starts.
     *******************************************************/
#if defined(USE_PCA9685_SERVO_EXPANDER)
    if (Servo1.InitializeAndCheckI2CConnection(&Serial)) {
        while (true) {
            blinkLED();
        }
    }
#endif
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
#  if defined(USE_PCA9685_SERVO_EXPANDER)
#undef SERVO1_PIN
#define SERVO1_PIN  0 // we use first port of expander
    Serial.println(F("Attach servo to port 0 of PCA9685 expander"));
#  else
    Serial.println(F("Attach servo at pin " STR(SERVO1_PIN)));
#  endif
#endif
    if (Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
        while (true) {
            blinkLED();
        }
    }

    // Wait for servo to reach start position.
    delay(500);
#if defined(PRINT_FOR_SERIAL_PLOTTER)
    // Legend for Arduino Serial plotter
    Serial.println(); // end of line of attach values
    Serial.println("OneServo[us]_Linear->Cubic->Linear");
#endif
}

void loop() {
    // Move slow
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move to 90 degree with 10 degree per second blocking"));
#endif
    Servo1.setSpeed(10);  // This speed is taken if no further speed argument is given.
    Servo1.easeTo(90);
//    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_90_DEGREE); // Alternatively you can specify the target as microsecond value

    // Now move faster without any delay between the moves
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move to 180 degree with 30 degree per second blocking with own loop"));
#endif
    Servo1.startEaseTo(180, 30, DO_NOT_START_UPDATE_BY_INTERRUPT); // no interrupts here
    do {
        // First do the delay, then check for update, since we are likely called directly after start and there is nothing to move yet
        delay(REFRESH_INTERVAL_MILLIS); // 20 ms

#if defined(PRINT_FOR_SERIAL_PLOTTER)
    } while (!updateAllServos()); // this outputs a value plus a newline, whilst Servo1.update() would not output the newline
#else
    } while (!Servo1.update());
#endif

    delay(1000);

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move to 45 degree in one second using interrupts"));
    Serial.flush(); // Just in case interrupts do not work
#endif
    Servo1.startEaseToD(45, 1000);
//    Servo1.startEaseToD((544 + ((2400 - 544) / 4)), 1000); // Alternatively you can specify the target as microsecond value
    // Blink until servo stops
    while (Servo1.isMoving()) {
        /*
         * Put your own code here
         */
        blinkLED();
    }

    delay(2000); // wait one second after servo has arrived at target position

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move to 135 degree and back nonlinear in one second each using interrupts"));
#endif
    Servo1.setEasingType(EASE_CUBIC_IN_OUT); // EASE_LINEAR is default

    for (int i = 0; i < 2; ++i) {
        Servo1.startEaseToD(135, 1000);
//        Servo1.startEaseToD((544 + (((2400 - 544) / 4) * 3)), 1000); // Alternatively you can specify the target as microsecond value
        // isMoving() calls yield for the ESP8266 boards
        while (Servo1.isMoving()) {
            /*
             * Put your own code here
             */
            ; // no delays here to avoid break between forth and back movement
        }
        Servo1.startEaseToD(45, 1000);
//        Servo1.startEaseToD((544 + ((2400 - 544) / 4)), 1000); // Alternatively you can specify the target as microsecond value
        while (Servo1.isMoving()) {
            ; // no delays here to avoid break between forth and back movement
        }
    }
    Servo1.setEasingType(EASE_LINEAR);

    delay(2000); // wait one second after servo has arrived at target position

    /*
     * The LED goes on if servo reaches 120 degree
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move to 180 degree with 15 degree per second using interrupts and switch LED on at 120 degree"));
#endif
    Servo1.startEaseTo(180, 15, START_UPDATE_BY_INTERRUPT);
//    Servo1.startEaseTo(DEFAULT_MICROSECONDS_FOR_180_DEGREE, 50); // Alternatively you can specify the target as microsecond value
    while (Servo1.getCurrentAngle() < 120) {
        delay(20); // just wait until angle is above 120 degree
    }
    digitalWrite(LED_BUILTIN, HIGH);
    while (Servo1.isMoving()) {
        ; // wait for servo to stop
    }

    delay(1000);

    /*
     * The LED goes off when servo theoretical reaches 90 degree
     */
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move from 180 to 0 degree with 90 degree per second using interrupts"));
#endif
    Servo1.startEaseTo(0, 90, START_UPDATE_BY_INTERRUPT);
//    Servo1.startEaseTo(DEFAULT_MICROSECONDS_FOR_0_DEGREE, 360, true); // Alternatively you can specify the target as microsecond value
    // Wait. The servo should have moved 90 degree.
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

#if !defined(DISABLE_PAUSE_RESUME)
#  if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Interrupt movement with pause() for 1 second at 90 degree"));
  #endif
    /*
     * Demonstrate pause and resume in the middle of a movement
     */
    Servo1.pause();
    delay(1000);
    // resume movement using interrupts
    Servo1.resumeWithInterrupts();
#endif
    while (Servo1.isMoving()); // wait for servo to stop

    Servo1.detach();
    /*
     * After detach the servo is "not powered" for 5 seconds, i.e. no servo signal is generated.
     * This allows you to easily move the servo manually.
     */
    delay(5000); // wait 5 seconds
    Servo1.attach(SERVO1_PIN, 0);
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}
