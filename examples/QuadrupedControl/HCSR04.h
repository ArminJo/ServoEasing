/*
 * HCSR04.h
 *
 *  Supports 1 Pin mode as you get on the HY-SRF05 if you connect OUT to ground.
 *  You can modify the HC-SR04 modules to 1 Pin mode by:
 *  Old module with 3 16 pin chips: Connect Trigger and Echo direct or use a resistor < 4.7 kOhm.
 *        If you remove both 10 kOhm pullup resistor you can use a connecting resistor < 47 kOhm, but I suggest to use 10 kOhm which is more reliable.
 *  Old module with 3 16 pin chips but with no pullup resistors near the connector row: Connect Trigger and Echo with a resistor > 200 Ohm. Use 10 kOhm.
 *  New module with 1 16 pin and 2 8 pin chips: Connect Trigger and Echo by a resistor > 200 Ohm and < 22 kOhm.
 *  All modules: Connect Trigger and Echo by a resistor of 4.7 kOhm.
 *
 *  Copyright (C) 2018-2020  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Utils https://github.com/ArminJo/Arduino-Utils.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
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
 *
 */

#ifndef _HCSR04_H
#define _HCSR04_H

#include <stdint.h>

#define DISTANCE_TIMEOUT_RESULT                0
#define US_DISTANCE_DEFAULT_TIMEOUT_MICROS    20000
#define US_DISTANCE_DEFAULT_TIMEOUT_CENTIMETER  343   // Timeout of 20000L is 3.43 meter

#define US_DISTANCE_TIMEOUT_MICROS_FOR_1_METER  5825  // Timeout of 5825 is 1 meter
#define US_DISTANCE_TIMEOUT_MICROS_FOR_2_METER 11650 // Timeout of 11650 is 2 meter
#define US_DISTANCE_TIMEOUT_MICROS_FOR_3_METER 17475 // Timeout of 17475 is 3 meter

void initUSDistancePins(uint8_t aTriggerOutPin, uint8_t aEchoInPin = 0);
void initUSDistancePin(uint8_t aTriggerOutEchoInPin); // Using this determines one pin mode
unsigned int getUSDistance(unsigned int aTimeoutMicros = US_DISTANCE_DEFAULT_TIMEOUT_MICROS);
unsigned int getCentimeterFromUSMicroSeconds(unsigned int aDistanceMicros);
unsigned int getUSDistanceAsCentimeter(unsigned int aTimeoutMicros = US_DISTANCE_DEFAULT_TIMEOUT_MICROS);
unsigned int getUSDistanceAsCentimeterWithCentimeterTimeout(unsigned int aTimeoutCentimeter);
void testUSSensor(uint16_t aSecondsToTest);

#if (defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7) | defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13) | defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5))
/*
 * Non blocking version
 */
void startUSDistanceAsCentimeterWithCentimeterTimeoutNonBlocking(unsigned int aTimeoutCentimeter);
bool isUSDistanceMeasureFinished();
extern unsigned int sUSDistanceCentimeter;
extern volatile unsigned long sUSPulseMicros;
#endif

#define HCSR04_MODE_UNITITIALIZED   0
#define HCSR04_MODE_USE_1_PIN       1
#define HCSR04_MODE_USE_2_PINS      2
extern uint8_t sHCSR04Mode;

#endif // _HCSR04_H
