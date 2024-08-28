/*
 * PrecisionEasing.cpp
 *
 *  Shows PRECISION easing type.
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
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
//#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
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
//#define ENABLE_EASE_CUBIC
//#define ENABLE_EASE_QUARTIC
//#define ENABLE_EASE_SINE
//#define ENABLE_EASE_CIRCULAR
//#define ENABLE_EASE_BACK
//#define ENABLE_EASE_ELASTIC
//#define ENABLE_EASE_BOUNCE
#define ENABLE_EASE_PRECISION
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

#define START_DEGREE_VALUE  45 // The degree value written to the servo at time of attach.
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
    Serial.println("PrecisionEasing[us]_Precision_out->Precision_in");
#endif

    Servo1.setSpeed(90);  // This speed is taken if no further speed argument is given.
}

void loop() {
    /*
     * Show EASE_PRECISION_OUT
     */
    Servo1.setEasingType(EASE_PRECISION_OUT); // EASE_LINEAR is default
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move up/out to 135 degree with EASE_PRECISION_OUT doing a bounce"));
#endif
    Servo1.easeTo(135);
    delay(500);

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move back/in to 45 degree with EASE_PRECISION_OUT with no bounce"));
#endif
    Servo1.easeTo(45);
    delay(1000);

    /*
     * Show EASE_PRECISION_IN
     */
    Servo1.setEasingType(EASE_PRECISION_IN); // EASE_LINEAR is default
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move up/out to 135 degree with EASE_PRECISION_IN with no bounce"));
#endif
    Servo1.easeTo(135);
    delay(500);

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    Serial.println(F("Move back/in to 45 degree with EASE_PRECISION_IN doing a bounce"));
#endif
    Servo1.easeTo(45);
    delay(4000);
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}
