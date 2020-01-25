/*
 * LightweightServoExample.cpp
 *
 *  Moves 2 servos attached at pin 9 and 10 using the LightweightServo library for ATmega328*.
 *
 *  Copyright (C) 2020  Armin Joachimsmeyer
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
 *
 */

#include <Arduino.h>

#include "LightweightServo.h"

#define VERSION_EXAMPLE "1.0"

void setup() {
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__)
    while (!Serial); //delay for Leonardo, but this loops forever for Maple Serial
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
    // no initialization needed for LightweightServo :-)
}

void loop() {
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined (__AVR_ATmega328PB__)
    /*
     * Let the servo at pin 9 swipe from 0 to 180 and back to 90 degree
     */
    write9(0);
    delay(1500);
    write9(180);
    delay(1000);
    write9(90);
    delay(1000);

    /*
     * Now move servo at pin 10
     */
    write10(0);
    delay(1500);
    write10(180);
    delay(1000);
    write10(90);
    delay(1000);

    /*
     * Move both servos to 45 degree using microseconds as parameter
     */
    Serial.print(F("45 degree = "));
    Serial.print(MicrosecondsToDegreeLightweightServo(45));
    Serial.println(F(" micro seconds"));
    writeMicroseconds9(1008);
    writeMicroseconds10(1008);
#else
    Serial.println(F("LightweightServoExample works only for ATmega328*"));
#endif
    delay(5000);
}
