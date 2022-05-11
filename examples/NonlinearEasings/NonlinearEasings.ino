/*
 * NonlinearEasings.cpp
 *
 *  Shows smooth non-linear movement from one servo position to another.
 *  Linear->Quadratic->Cubic->Quartic->Sine-Circular->Back->Elastic
 *  Note, that Back and Elastic are not totally visible at your servo, since they use angels above 180 and below 0 degree in this example.
 *
 *  Copyright (C) 2022  Armin Joachimsmeyer
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
//#define USE_PCA9685_SERVO_EXPANDER    // Activate this to enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB                 // Activate this to force additional using of regular servo library.
//#define USE_LEIGHTWEIGHT_SERVO_LIB    // Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activate this to disable all but LINEAR movement. Saves up to 1540 bytes program memory.
//#define DISABLE_COMPLEX_FUNCTIONS     // Activate this to disable the SINE, CIRCULAR, BACK, ELASTIC and BOUNCE easings. Saves up to 1850 bytes program memory.
#define MAX_EASING_SERVOS 1
//#define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle) parameter. Saves 128 bytes program memory.
//#define DEBUG                         // Activate this to generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER      // Activate this to generate the Arduino plotter output.
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
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS, &Wire); // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS
#else
ServoEasing Servo1;
#endif

#define START_DEGREE_VALUE  0 // The degree value written to the servo at time of attach.
//#define USE_MICROSECONDS      // Use microseconds instead degrees as parameter

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
    Serial.println(F("Attach servo at pin " STR(SERVO1_PIN)));
#endif
    if (Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
    }

    // Wait for servo to reach start position.
    delay(500);
#if defined(PRINT_FOR_SERIAL_PLOTTER)
    // Legend for Arduino Serial plotter
    Serial.println(); // end of line of attach values
    Serial.println("OneServo[us]_Linear->Quadratic->Cubic->Quartic ->Sine-Circular->Back->Elastic");
#endif
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

void loop() {
    Servo1.setSpeed(90);  // This speed is taken if no further speed argument is given.

    // Move linear
    Servo1.setEasingType(EASE_LINEAR);
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move linear to 90 degree"));
#endif
#if defined(USE_MICROSECONDS)
    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_90_DEGREE);
#else
    Servo1.easeTo(90);
#endif
    delay(500);

    Servo1.setEasingType(EASE_QUADRATIC_IN_OUT);
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move quadratic to 180 degree"));
#endif
#if defined(USE_MICROSECONDS)
    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_180_DEGREE);
#else
    Servo1.easeTo(180);
#endif
    delay(500);

    Servo1.setEasingType(EASE_CUBIC_IN_OUT);
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move cubic to 90 degree"));
#endif
#if defined(USE_MICROSECONDS)
    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_90_DEGREE);
#else
    Servo1.easeTo(90);
#endif

    delay(500);

    Servo1.setEasingType(EASE_QUARTIC_IN_OUT);
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move quartic to 0 degree"));
#endif
#if defined(USE_MICROSECONDS)
    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_0_DEGREE);
#else
    Servo1.easeTo(0);
#endif
    delay(2000);

    Servo1.setEasingType(EASE_SINE_IN_OUT);
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move sine to 90 degree"));
#endif
#if defined(USE_MICROSECONDS)
    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_90_DEGREE);
#else
    Servo1.easeTo(90);
#endif
    delay(500);

    Servo1.setEasingType(EASE_CIRCULAR_IN_OUT);
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move circular to 180 degree"));
#endif
#if defined(USE_MICROSECONDS)
    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_180_DEGREE);
#else
    Servo1.easeTo(180);
#endif
    delay(500);

    Servo1.setEasingType(EASE_BACK_IN_OUT);
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move back to 90 degree"));
#endif
#if defined(USE_MICROSECONDS)
    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_90_DEGREE);
#else
    Servo1.easeTo(90);
#endif
    delay(500);

    Servo1.setEasingType(EASE_ELASTIC_IN_OUT);
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move elastic to 0 degree"));
#endif
#if defined(USE_MICROSECONDS)
    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_0_DEGREE);
#else
    Servo1.easeTo(0);
#endif
    delay(500);

    delay(5000);
}
