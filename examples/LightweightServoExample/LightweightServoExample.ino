/*
 * LightweightServoExample.cpp
 *
 *  Moves 2 servos attached at pin 9 and 10 using the LightweightServo library for ATmega328*.
 *
 *  Copyright (C) 2020-2024  Armin Joachimsmeyer
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
 *
 */

#include <Arduino.h>

//#define DEBUG

//#define DISABLE_SERVO_TIMER_AUTO_INITIALIZE // Activating this saves 40 bytes program space. You must then use the init functions initLightweightServoPin*() manually.
#include "LightweightServo.hpp"

void setup() {
    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__));
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined (__AVR_ATmega328PB__)
    Serial.println(F("Using library version " VERSION_LIGHTWEIGHT_SERVO));
#endif
    // no initialization required for LightweightServo :-)
    // or use manual initialization (and compiler macro "DISABLE_SERVO_TIMER_AUTO_INITIALIZE") to save additional 60 bytes program memory
#if defined(DISABLE_SERVO_TIMER_AUTO_INITIALIZE)
     initLightweightServoPins();
#endif
    delay(3000);
}

void loop() {
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined (__AVR_ATmega328PB__)
    /*
     * Let the servo at pin 9 + 10 swipe from 180 to 0 and back to 90 degree
     */
    Serial.println(F("Move to 180 degree"));
    write9(180);
    write10(180);
    delay(2000);

    Serial.println(F("Move to 90 degree"));
    write9(90);
    write10(90);
    delay(2000);

    Serial.println(F("Move to 0 degree"));
    write9(0);
    write10(0);
    delay(2000);

    Serial.println(F("Move to 90 degree"));
    write9(90);
    write10(90);
    delay(2000);

    Serial.println(F("Move back to 180 degree"));
    write9(180);
    write10(180);
    delay(2000);
    Serial.println();

    /*
     * Move both servos to 135 and 45 degree using microseconds as parameter
     */
    Serial.print(F("Move to 135 degree = "));
    Serial.print(DegreeToMicrosecondsLightweightServo(135));
    Serial.println(F(" micro seconds"));
    writeMicroseconds9(DegreeToMicrosecondsLightweightServo(135));
    writeMicroseconds10(DegreeToMicrosecondsLightweightServo(135));
    Serial.println();
    delay(2000);

    Serial.print(F("Move to 45 degree = "));
    Serial.print(DegreeToMicrosecondsLightweightServo(45));
    Serial.println(F(" micro seconds"));
    writeMicroseconds9(DegreeToMicrosecondsLightweightServo(45));
    writeMicroseconds10(DegreeToMicrosecondsLightweightServo(45));

    delay(5000);

#elif defined(__AVR_ATmega2560__)
    /*
     * Let the servo at pin 9 + 10 swipe from 180 to 0 and back to 90 degree
     */
    Serial.println(F("Move to 180 degree"));
    writeLightweightServoPin(180, 44);
    writeLightweightServoPin(180, 45);
    writeLightweightServoPin(180, 46);
    delay(2000);

    Serial.println(F("Move to 90 degree"));
    writeLightweightServoPin(90, 44);
    writeLightweightServoPin(90, 45);
    writeLightweightServoPin(90, 46);
    delay(2000);

    Serial.println(F("Move to 0 degree"));
    writeLightweightServoPin(0, 44);
    writeLightweightServoPin(0, 45);
    writeLightweightServoPin(0, 46);
    delay(2000);

    Serial.println(F("Move to 90 degree"));
    writeLightweightServoPin(90, 44);
    writeLightweightServoPin(90, 45);
    writeLightweightServoPin(90, 46);
    delay(2000);

    Serial.println(F("Move back to 180 degree"));
    writeLightweightServoPin(180, 44);
    writeLightweightServoPin(180, 45);
    writeLightweightServoPin(180, 46);
    delay(2000);
    Serial.println();

    /*
     * Move both servos to 135 and 45 degree using microseconds as parameter
     */
    Serial.print(F("Move to 135 degree = "));
    Serial.print(DegreeToMicrosecondsLightweightServo(135));
    Serial.println(F(" micro seconds"));
    writeMicrosecondsLightweightServoPin(DegreeToMicrosecondsLightweightServo(135),44);
    writeMicrosecondsLightweightServoPin(DegreeToMicrosecondsLightweightServo(135),45);
    writeMicrosecondsLightweightServoPin(DegreeToMicrosecondsLightweightServo(135),46);
    Serial.println();
    delay(2000);

    Serial.print(F("Move to 45 degree = "));
    Serial.print(DegreeToMicrosecondsLightweightServo(45));
    Serial.println(F(" micro seconds"));
    writeMicrosecondsLightweightServoPin(DegreeToMicrosecondsLightweightServo(45),44);
    writeMicrosecondsLightweightServoPin(DegreeToMicrosecondsLightweightServo(45),45);
    writeMicrosecondsLightweightServoPin(DegreeToMicrosecondsLightweightServo(45),46);

    delay(5000);

#else
    Serial.println(F("LightweightServoExample works only for ATmega328* and ATmega2560"));
#endif
}
