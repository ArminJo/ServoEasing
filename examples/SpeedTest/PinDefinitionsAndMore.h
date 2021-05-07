/*
 *  PinDefinitionsAndMore.h
 *
 *  Contains SERVOX_PIN definitions for ServoEasing examples for various platforms
 *  as well as includes and definitions for LED_BUILTIN
 *
 *  Copyright (C) 2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  IRMP is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

/*
 * Pin mapping table for different platforms
 *
 * Platform         Servo1      Servo2      Servo3      Analog
 * -----------------------------------------------------------
 * (Mega)AVR + SAMD   9           10          11          A0
 * ESP8266            14 // D5    12 // D6    13 // D7    0
 * ESP32              5           18          19          A0
 * BluePill           PB7         PB8         PB9         PA0
 * APOLLO3            11          12          13          A3
 */

#if defined(ESP8266)
#define SERVO1_PIN 14 // D5
#define SERVO2_PIN 12 // D6
#define SERVO3_PIN 13 // D7
#define SPEED_IN_PIN 0

#elif defined(ESP32)
#define SERVO1_PIN 5
#define SERVO2_PIN 18
#define SERVO3_PIN 19
#define SPEED_IN_PIN A0 // 36/VP
#define MODE_ANALOG_INPUT_PIN A3 // 39

#elif defined(STM32F1xx) || defined(__STM32F1__)
// BluePill in 2 flavors
// STM32F1xx is for "Generic STM32F1 series / STM32:stm32" from STM32 Boards from STM32 cores of Arduino Board manager
// __STM32F1__is for "Generic STM32F103C series / stm32duino:STM32F1" from STM32F1 Boards (STM32duino.com) of Arduino Board manager
#define SERVO1_PIN PB7
#define SERVO2_PIN PB8
#define SERVO3_PIN PB9 // Needs timer 4 for Servo library
#define SPEED_IN_PIN PA0
#define MODE_ANALOG_INPUT_PIN PA1

#elif defined(ARDUINO_ARCH_APOLLO3)
#define SERVO1_PIN 11
#define SERVO2_PIN 12
#define SERVO3_PIN 13
#define SPEED_IN_PIN A2
#define MODE_ANALOG_INPUT_PIN A3

#elif defined(ARDUINO_ARCH_MBED) // Arduino Nano 33 BLE
#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define SERVO3_PIN 11
#define SPEED_IN_PIN A0
#define MODE_ANALOG_INPUT_PIN A1

#elif defined(__AVR__)
#define SERVO1_PIN 9 // For ATmega328 pins 9 + 10 are connected to timer 2 and can therefore be used also by the Lightweight Servo library
#define SERVO2_PIN 10
#define SERVO3_PIN 11
#define SPEED_IN_PIN A0
#define MODE_ANALOG_INPUT_PIN A1

#else
#warning Board / CPU is not detected using pre-processor symbols -> using default values, which may not fit. Please extend PinDefinitionsAndMore.h.
// Default valued for unidentified boards
#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define SERVO3_PIN 11
#define SPEED_IN_PIN A0
#define MODE_ANALOG_INPUT_PIN A1

#endif

#define SERVO_UNDER_TEST_PIN SERVO1_PIN

#define SPEED_OR_POSITION_ANALOG_INPUT_PIN SPEED_IN_PIN
#define POSITION_ANALOG_INPUT_PIN SPEED_IN_PIN

// for ESP32 LED_BUILTIN is defined as: static const uint8_t LED_BUILTIN 2
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif
// On the Zero and others we switch explicitly to SerialUSB
#if defined(ARDUINO_ARCH_SAMD)
#define Serial SerialUSB
// The Chinese SAMD21 M0-Mini clone has no led connected, if you connect it, it is on pin 24 like on the original board.
// Attention! D2 and D4 are reversed on these boards
//#undef LED_BUILTIN
//#define LED_BUILTIN 25 // Or choose pin 25, it is the RX pin, but active low.
#endif
