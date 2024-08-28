/*
 * ConsecutiveEasingsWithCallback.cpp
 *
 *  This example shows a trajectory consisting of 1 linear and 7 non-linear easings in flavor IN_OUT for 1 servo, followed with flavors of IN, OUT and BOUNCING.
 *  Linear->Quadratic->Cubic->Quartic->Sine-Circular->Back->Elastic->Quadratic_in->Cubic_out->Cubic_bounce->Dummy.
 *  Note, that Back and Elastic are not totally visible at your servo, since they use angels above 180 and below 0 degree in this example.
 *  It uses a callback handler and specification arrays to generate the movement cycle.
 *
 *  Copyright (C) 2022-2023  Armin Joachimsmeyer
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
//#define DEBUG                         // Activating this enables generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER      // Activating this enables generate the Arduino plotter output from ServoEasing.hpp.
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
//#define USE_CONSTRAINTS       // Use constraints to limit the servo movements

/*
 * Arrays for the parameter of movements controlled by callback
 */
#define NUMBER_OF_MOVEMENTS_IN_A_TRAJECTORY  14
uint_fast8_t EasingTypesArray[NUMBER_OF_MOVEMENTS_IN_A_TRAJECTORY] = { EASE_LINEAR, EASE_QUADRATIC_IN_OUT, EASE_CUBIC_IN_OUT,
EASE_QUARTIC_IN_OUT, EASE_SINE_IN_OUT, EASE_CIRCULAR_IN_OUT, EASE_BACK_IN_OUT, EASE_ELASTIC_IN_OUT, EASE_QUADRATIC_IN,
EASE_CUBIC_OUT, EASE_CUBIC_BOUNCING, EASE_DUMMY_MOVE, EASE_PRECISION_IN, EASE_BOUNCE_OUT };
#if defined(USE_MICROSECONDS)
int TargetDegreesArray[NUMBER_OF_MOVEMENTS_IN_A_TRAJECTORY] = { DEFAULT_MICROSECONDS_FOR_90_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE,
DEFAULT_MICROSECONDS_FOR_90_DEGREE, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_90_DEGREE,
DEFAULT_MICROSECONDS_FOR_180_DEGREE, DEFAULT_MICROSECONDS_FOR_90_DEGREE, DEFAULT_MICROSECONDS_FOR_0_DEGREE,
DEFAULT_MICROSECONDS_FOR_90_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE,
DEFAULT_MICROSECONDS_FOR_90_DEGREE, DEFAULT_MICROSECONDS_FOR_90_DEGREE, DEFAULT_MICROSECONDS_FOR_0_DEGREE };
#else
int TargetDegreesArray[NUMBER_OF_MOVEMENTS_IN_A_TRAJECTORY] = { 90, 180, 90, 0, 90, 180, 90, 0, 90, 180, 180, 90, 90, 0 };
#endif

void blinkLED();
void ServoTargetPositionReachedHandler(ServoEasing *aServoEasingInstance);

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
#if defined(USE_CONSTRAINTS)
    Serial.println(
            F(
                    "OneServo_Constraints_at_5_175[us]_Linear->Quadratic->Cubic->Quartic ->Sine-Circular->Back->Elastic ->QuadraticIn->CubicOut->CubicBounce->Dummy->Precision->Bounce"));
#else
    Serial.println(
            F(
                    "OneServo[us]_Linear->Quadratic->Cubic->Quartic ->Sine-Circular->Back->Elastic ->QuadraticIn->CubicOut->CubicBounce->Dummy->Precision->Bounce"));
#endif
    Servo1.setSpeed(125);  // This speed is taken if no further speed argument is given.
#else
    Servo1.setSpeed(90);  // This speed is taken if no further speed argument is given.
#endif

#if defined(USE_CONSTRAINTS)
    Servo1.setMinMaxConstraint(5, 175);
#endif

    /*
     * Initialize and start multiple movements controlled by callback handler
     */
    Servo1.setTargetPositionReachedHandler(ServoTargetPositionReachedHandler);
    ServoTargetPositionReachedHandler(&Servo1); // start by calling handler which in turn calls Servo1.startEaseTo(tTargetDegree)
}

void loop() {

    if (Servo1.mCurrentMicrosecondsOrUnits != Servo1.mEndMicrosecondsOrUnits) {
        // real moving here
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        // delay / noMovement here
        digitalWrite(LED_BUILTIN, LOW);
    }

    delay(50);
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

/*
 * !!!This function is called in ISR context!!!
 * This means, that interrupts are disabled and delay() is not working, only delayMicroseconds() work.
 * All variables, that are set here and read by the main loop, must be declared as "volatile" otherwise race condition may appear.
 * All variables read here and set by the main loop, while this call can happen / is enabled,
 * must we written with a noInterrupts(), interrupts() guard.
 */
void ServoTargetPositionReachedHandler(ServoEasing *aServoEasingInstance) {
    static uint_fast8_t sStep = 0;

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    static bool sDoDelay = false;
    if (sDoDelay) {
        sDoDelay = false;
        aServoEasingInstance->noMovement(500); // Servo1
    } else {
        sDoDelay = true;
#endif
        int tTargetDegree = TargetDegreesArray[sStep];
        uint_fast8_t tEasingType = EasingTypesArray[sStep];

        aServoEasingInstance->setEasingType(tEasingType); // Servo1
        aServoEasingInstance->startEaseTo(tTargetDegree); // easeTo() uses delay() and will not work here.
        sStep++;
        if (sStep >= NUMBER_OF_MOVEMENTS_IN_A_TRAJECTORY) {
            sStep = 0; // do it forever
        }

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
        Serial.print(F("Move "));
        ServoEasing::printEasingType(&Serial, tEasingType);
        Serial.print(F(" to "));
        Serial.print(tTargetDegree);
#  if defined(USE_MICROSECONDS)
        Serial.println(F(" microseconds"));
#  else
        Serial.println(F(" degree"));
#  endif
    }
#endif
//    aServoEasingInstance->print(&Serial, false);
}

