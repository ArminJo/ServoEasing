/*
 * UnitTests.cpp
 *
 *  Internal tests for different functions.
 *
 *  Copyright (C) 2019-2024  Armin Joachimsmeyer
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
//#define TRACE                              // Activating this enables generate lots of trace output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER           // Activating this enables generate the Arduino plotter output from ServoEasing.hpp.
#include "ServoEasing.hpp"
#include "digitalWriteFast.h"
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

/*
 * On AVR you can use the same pin for monitoring, on ESP32, this stops program.
 */
#if defined(ESP32)
#define DETACH_MONITORING_IN_PIN    SERVO3_PIN
#else
#define DETACH_MONITORING_IN_PIN    SERVO1_PIN
#endif

// for ESP32 LED_BUILTIN is defined as: static const uint8_t LED_BUILTIN = 2;
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif

ServoEasing Servo1;
ServoEasing Servo2;

#define TEST_FIXED_PULSE_NUMBERS
#define TEST_DETACH_TIMING
#define TEST_GET_CURRENT_ANGLE
#define TEST_ATTACH_PARAMETER
#define TEST_DETACH_AND_ATTACH_AGAIN

void generatePulsesManuallyAndWait(uint8_t aPin, uint8_t aDegree, uint16_t aMicroseconds, uint8_t aNumberOfPulses,
        uint16_t aMillisToWait);
void testDetachTiming();
void moveToBothEndsAnd90Degree();
void testFixedPulseNumbers();
void testGetCurrentAngle();
void testAttachParameters();
void testDetachAndReatttach();
void testSetDegreeForAllServos();

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
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));

    // Attach servo to pin
    Serial.println(F("Attach servo at pin " STR(SERVO1_PIN) " and wait 3 seconds"));

    Servo1.attach(SERVO1_PIN); // First attach moves servo to DEFAULT_PULSE_WIDTH (90 degree | 1500 us) by underlying Servo library.
    delay(2000);

    // Move to both ends and end at 90 degree
    moveToBothEndsAnd90Degree();
    delay(4000);

#if defined(TEST_DETACH_TIMING)
    testDetachTiming();
    delay(2000);
#endif

#if defined(TEST_FIXED_PULSE_NUMBERS)
    /*
     * Test of 1 to 7 discrete servo pulses
     */
    testFixedPulseNumbers();
    delay(10000);
#endif

#if defined(TEST_DETACH_AND_ATTACH_AGAIN)
    testDetachAndReatttach();
    delay(2000);
#endif

#if defined(TEST_GET_CURRENT_ANGLE)
    testGetCurrentAngle();
    delay(2000);
#endif

#if defined(TEST_ATTACH_PARAMETER)
    /*
     * Miscellaneous attach parameter set tests
     */
    testAttachParameters();
    delay(3000);
#endif

#if defined(TEST_ATTACH_PARAMETER)
    /*
     * setDegreeForAllServos() tests
     */
    testSetDegreeForAllServos();
    delay(5000);
#endif

    /*******************************************************
     * Set servos to start position.
     * This is the position where the loop movement starts.
     ******************************************************/
    Serial.println(F("--- Move to 0 degree and wait 5 seconds before loop starts"));
    Servo1.write(0);
    Servo2.write(0);
    delay(5000);
    Serial.print(F("Start loop"));

}

void loop() {
    // Move to one end
    Serial.print(F("--- Move to 135 degree with 90 degree | "));
    Serial.print(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(135));
    Serial.println(F(" us with 90 degree per second blocking -> lasting 1 second"));
    Servo1.setSpeed(90); // This speed is taken if no speed argument is given.
    Servo1.easeTo(135);

    delay(1000);

    // Now move to the other end
    Serial.print(F("--- Move to 45 degree | "));
    Serial.print(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(45));
    Serial.println(F(" us with 30 degree per second -> lasting 3 seconds"));
    Servo1.easeTo(45, 30);

    delay(1000);
}

/*
 * Move to both ends and end at 90 degree
 */
void moveToBothEndsAnd90Degree() {
    Serial.println(F("Move to 0 -> 180 -> 90 degree"));
    Servo1.write(0); // start at 0 degree
    delay(1000);
    Servo1.write(180); // move to 180 degree
    delay(1000);
    Servo1.write(90); // move to 90 degree
    delay(200);
}

/*
 * Connect servo out pin to monitoring in pin
 */
#define MICROS_BETWEEN_PRINTS 50
void waitAndPrintPinLevel(uint32_t aMicrosOfPrint) {
    // Wait for micros
    while (true) {
        if (aMicrosOfPrint < micros()) {
            break;
        }
    }
    if (digitalRead(DETACH_MONITORING_IN_PIN) == HIGH) {
        Serial.print('-');
    } else {
        Serial.print('_');
    }
}
/*
 * Attach and move to 180 degree
 * 1. loop: detach while pulse is generated
 * 2. loop: detach after pulse is generated
 *
 * Sample output for Uno / Nano / BluePill / ESP32:
 * Each character is 50 microseconds
 * Move servo to 180 degree | 2400 us, wait 400 ms and then detach 600 microseconds after start of pulse
 * _-------------------------------------------------------
 * Move servo to 180 degree | 2400 us, wait 400 ms and then detach 2500 microseconds after start of pulse
 * _-----------------------------------------------________
 *  showing a 2.35 ms pulse
 */
void testDetachTiming() {
    Serial.println();
#if SERVO1_PIN != DETACH_MONITORING_IN_PIN
    Serial.println(F("Connect servo out pin " STR(SERVO1_PIN) " to monitoring in pin " STR(DETACH_MONITORING_IN_PIN)));
#endif
    Serial.println(F("Each character is 50 microseconds"));

    uint8_t tInitialWait50Micros = (DEFAULT_MICROSECONDS_FOR_0_DEGREE / MICROS_BETWEEN_PRINTS) + 2; // 600 us
    for (int j = 0; j < 2; ++j) {
        Servo1.reattach(); // First attach moves servo to DEFAULT_PULSE_WIDTH (90 degree | 1500 us) by underlying Servo library.
        Servo1.write(180); // move to 180 degree
        Serial.print(F("Move servo to 180 degree | "));
        Serial.print(DEFAULT_MICROSECONDS_FOR_180_DEGREE);
        Serial.print(F(" us, wait 1000 ms and then detach "));
        Serial.print(tInitialWait50Micros * MICROS_BETWEEN_PRINTS);
        Serial.println(F(" microseconds after start of pulse"));

        delay(1000);
        // wait until signal is LOW
        while (true) {
            if (digitalRead(DETACH_MONITORING_IN_PIN) == LOW) {
                break;
            }
        }
        Serial.print('_');
        // wait for start of pulse
        while (true) {
            if (digitalRead(DETACH_MONITORING_IN_PIN) == HIGH) {
                break; // Start of pulse detected
            }
        }

        uint32_t tMicrosOfPrint = micros();
        int i = 0;
        // wait for the length of a minimal servo pulse (around 600 us) and detach
        for (; i < tInitialWait50Micros; ++i) {
            tMicrosOfPrint += MICROS_BETWEEN_PRINTS;
            waitAndPrintPinLevel(tMicrosOfPrint);
        }
        Servo1.detach();

        /*
         * Print resulting signal as ASCII art
         */
        for (; i < (DEFAULT_MICROSECONDS_FOR_180_DEGREE / MICROS_BETWEEN_PRINTS) + 7; ++i) {
            tMicrosOfPrint += MICROS_BETWEEN_PRINTS;
            waitAndPrintPinLevel(tMicrosOfPrint);
        }
        Serial.println();
        tInitialWait50Micros = (DEFAULT_MICROSECONDS_FOR_180_DEGREE / MICROS_BETWEEN_PRINTS) + 2; // 2400 DEFAULT_MICROSECONDS_FOR_180_DEGREE is 2400
        delay(2000);
    }
    Servo1.reattach();
    Serial.println();
}

/*
 * Test of one to 7 discrete servo pulses
 *
 * After disconnected, my MG90 servo requires 1 pulse for a 110 degree turn. the second pulse (after 20 ms) adds around 10 degree to it,
 * so it takes around 6 to 7 pulses (120 ms to 140 ms) for a complete 180 degree turn.
 * The values seems to be independent of the turn direction.
 *
 * After disconnected, my SG90 servo requires 4 pulses for a 180 degree turn. It may be less, if the turn is smaller.
 * 1 pulse for 180 or 0 degree moves 20 to 50 degree dependent of the last position - the bigger the difference, the bigger the movement.
 */
void testFixedPulseNumbers() {
    Servo1.detach();

    Serial.println();
    Serial.println(F("Generate 1 to 7 180 degree pulses at pin " STR(SERVO1_PIN)));
    pinMode(SERVO1_PIN, OUTPUT);
    digitalWriteFast(SERVO1_PIN, LOW);
    delay(100);
    for (uint_fast8_t i = 0; i < 7; ++i) {
        // position at 0, then move to 180
        generatePulsesManuallyAndWait(SERVO1_PIN, 0, DEFAULT_MICROSECONDS_FOR_0_DEGREE, 10, 2000); // wait longer in order that the servo capacitor is unloaded
        generatePulsesManuallyAndWait(SERVO1_PIN, 180, DEFAULT_MICROSECONDS_FOR_180_DEGREE, i + 1, 2000);
        // position at 90, then move to 180
        generatePulsesManuallyAndWait(SERVO1_PIN, 90, DEFAULT_MICROSECONDS_FOR_90_DEGREE, 10, 2000);
        generatePulsesManuallyAndWait(SERVO1_PIN, 180, DEFAULT_MICROSECONDS_FOR_180_DEGREE, i + 1, 2000);
    }
    delay(4000);
    Serial.println(F("Generate 1 to 7 0 degree pulses manually at pin " STR(SERVO1_PIN)));
    for (uint_fast8_t i = 0; i < 7; ++i) {
        // position at 180, then move to 0
        generatePulsesManuallyAndWait(SERVO1_PIN, 180, DEFAULT_MICROSECONDS_FOR_180_DEGREE, 10, 2000);
        generatePulsesManuallyAndWait(SERVO1_PIN, 0, DEFAULT_MICROSECONDS_FOR_0_DEGREE, i + 1, 2000);
        // position at 90, then move to 0
        generatePulsesManuallyAndWait(SERVO1_PIN, 90, DEFAULT_MICROSECONDS_FOR_90_DEGREE, 10, 2000);
        generatePulsesManuallyAndWait(SERVO1_PIN, 0, DEFAULT_MICROSECONDS_FOR_0_DEGREE, i + 1, 2000);
    }
    Serial.println();
    Servo1.reattach();
}

void testGetCurrentAngle() {
    Serial.println();
    Serial.println(F("getCurrentAngle() tests while moving from 0 to 180 degree and back."));
    Servo1.startEaseToD(180, 1000);
    while (ServoEasing::areInterruptsActive()) {
        Serial.print("> ");
        Serial.println(Servo1.getCurrentAngle());
        delay(100);
    }
    Servo1.startEaseToD(0, 1000);
    while (ServoEasing::areInterruptsActive()) {
        Serial.print("< ");
        Serial.println(Servo1.getCurrentAngle());
        delay(100);
    }
    Serial.println();
}

/*
 * Miscellaneous attach parameter set tests
 */
void testAttachParameters() {
    Serial.println();
    Serial.println(F("\r\nMiscellaneous attach parameter set tests."));
    Serial.println(F("First value must be 544 second 2400, independent of degree"));
    Serial.println();
    Serial.println();
    Serial.print(F("Full virtual scale is now 0 to 135. Attach with initial move to virtual 135 degree -> "));
    Servo1.attach(SERVO1_PIN, 135, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, 0, 135);
    Serial.print(F("0="));
    Serial.print(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(0));
    Serial.print(F(" | 135="));
    Serial.println(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(135));
    Serial.println(F("--- Move to physical 180 degree | virtual 135 degree"));
    delay(2000);
    Serial.print(F("Full virtual scale is now 0 to 90 -> "));
    Servo1.detach();
    Servo1.attach(SERVO1_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, 0, 90);
    Serial.print(F("0="));
    Serial.print(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(0));
    Serial.print(F(" | 90="));
    Serial.println(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(90));
    Servo1.write(45);
    Serial.println(F("--- Move to physical 90 degree | virtual 45 degree"));
    delay(3000);
    Serial.print(F("Trim of 90, full virtual scale is now -90 to +90 degree -> "));
    Servo1.detach();
    Servo1.attach(SERVO1_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);
    Servo1.setTrim(90, true);
    Serial.print(F("-90="));
    Serial.print(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(-90));
    Serial.print(F(" | 90="));
    Serial.println(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(90));
    Servo1.write(90);
    Serial.println(F("--- Move to physical 180 degree | virtual 90 degree"));
    delay(3000);
    Serial.print(F("Trim of -45 and full virtual scale is now 45 to +225 -> "));
    Servo1.detach();
    Servo1.attachWithTrim(SERVO1_PIN, -45, 10, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);
    Serial.print(F("45="));
    Serial.print(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(-45));
    Serial.print(F(" | 225="));
    Serial.println(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(135));
    Servo1.write(135);
    Serial.println(F("--- Move to physical 90 degree | virtual 135 degree"));
    delay(3000);
    Serial.print(F("Full virtual scale is now (reverse) 60 to -120 -> "));
    Servo1.detach();
    Servo1.attach(SERVO1_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, 60, -120);
    Serial.print(F("60="));
    Serial.print(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(60));
    Serial.print(F(" | -120="));
    Serial.println(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(-120));
    Servo1.write(-120);
    Serial.println(F("--- Move to physical 180 degree | virtual -120 degree"));
    delay(3000);
    Serial.print(F("Full virtual scale is now -45 to 90 -> "));
    Servo1.detach();
    Servo1.attach(SERVO1_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, -45, 90);
    Serial.print(F("-45="));
    Serial.print(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(-45));
    Serial.print(F(" | 90="));
    Serial.println(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(90));
    Servo1.write(22);
    Serial.println(F("--- Move to physical 90 degree | virtual 22.5 degree"));
    delay(3000);
    Serial.print(F("Full scale is now (reverse) 90 to -45 -> "));
    Servo1.detach();
    Servo1.attach(SERVO1_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, 90, -45);
    Serial.print(F("90="));
    Serial.print(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(90));
    Serial.print(F(" -45="));
    Serial.println(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(-45));
    Servo1.write(-45);
    Serial.println(F("--- Move to physical 180 degree | virtual -45 degree"));
    Servo1.write(90);
    Serial.println(F("--- Move to physical 0 degree | virtual 90 degree"));
    Serial.println();
}

/*
 * setDegreeForAllServos() tests
 */
void testSetDegreeForAllServos() {
    Serial.println();
    Serial.print(F("sServoArrayMaxIndex="));
    Serial.println(ServoEasing::sServoArrayMaxIndex);
    Servo1.detach();
    delay(500);
    Servo1.attach(SERVO1_PIN); // attach with default values
    Servo1.setSpeed(90);
    Servo1.write(0); // Set right start position for easing. Otherwise the virtual position of 90 from above is taken.
    Serial.println();
    Serial.println(F("Attach 1. servo with default values and 2. servo at pin " STR(SERVO2_PIN)));
    Serial.println(F("3 setDegreeForAllServos() tests"));
    Servo2.attach(SERVO2_PIN); // attach with default values
    Servo2.setSpeed(45);
    Serial.print(F("Lowest speed is 45 degree/s of Servo2"));
    Serial.print(F("sServoArrayMaxIndex="));
    Serial.println(ServoEasing::sServoArrayMaxIndex);
    ServoEasing::ServoEasingNextPositionArray[0] = 180;
    ServoEasing::ServoEasingNextPositionArray[1] = 90;
    printArrayPositions(&Serial);
    setEaseToForAllServosSynchronizeAndWaitForAllServosToStop(); // Take the longer duration in order to move all servos synchronously
    Serial.print(F("--- Moved blocking "));
    Serial.print(ServoEasing::ServoEasingArray[0]->mMillisForCompleteMove);
    Serial.println(F(" ms to array position and wait 2 seconds"));
    delay(2000);
    setIntegerDegreeForAllServos(2, 90, 180);
    printArrayPositions(&Serial);
    setEaseToForAllServos();
    Serial.print(F("--- Move blocking max("));
    Serial.print(ServoEasing::ServoEasingArray[0]->mMillisForCompleteMove);
    Serial.print(F(", "));
    Serial.print(ServoEasing::ServoEasingArray[1]->mMillisForCompleteMove);
    Serial.println(F(") ms to array position and wait 2 seconds"));
    synchronizeAllServosStartAndWaitForAllServosToStop();
    delay(2000);
    setFloatDegreeForAllServos(2, 180.8, 90.5);
    printArrayPositions(&Serial);
    Servo2.setSpeed(90);
    setEaseToForAllServos();
    Serial.print(F("--- Move blocking max("));
    Serial.print(ServoEasing::ServoEasingArray[0]->mMillisForCompleteMove);
    Serial.print(F(", "));
    Serial.print(ServoEasing::ServoEasingArray[1]->mMillisForCompleteMove);
    Serial.println(F(") ms to array position and wait 2 seconds"));
    synchronizeAllServosStartAndWaitForAllServosToStop();
    Serial.println();
}

void testDetachAndReatttach() {
    Serial.println();
    Serial.println(F("--- Move to 135 degree and wait 1 second."));
    Servo1.write(135);
    delay(1000);
    Serial.println(F("Detach for 5 seconds."));
    Serial.println(F("--- Servo should not move, but could be moved manually now"));
    Servo1.detach();
    delay(5000); // Test https://github.com/ArminJo/ServoEasing/discussions/91#discussioncomment-7592860
    Serial.print(F("Reattach, servo moves to last position before detach (135 degree) -> "));
    Servo1.reattach();
    Serial.print(F("0 (default)="));
    Serial.print(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(0));
    Serial.print(F(" | 180 (default)="));
    Serial.println(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(180));
    delay(5000);
    Serial.print(F("The same with reverse -> "));
    Servo1.detach();
    Servo1.reattach(); // Servo should not move back to former position (135 degree) on attach
    Servo1.setReverseOperation(true);
    Serial.print(F("0="));
    Serial.print(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(0));
    Serial.print(F(" | 180="));
    Serial.println(Servo1.DegreeOrMicrosecondToMicrosecondsOrUnits(180));
    Servo1.write(135);
    Serial.println(F("--- Move to physical 45 degree"));
    delay(5000);
    Serial.println(F("--- Wait 2 seconds, detach, wait 2 second, servo should not move"));
    delay(2000);
    Servo1.detach();
    delay(2000);
    Serial.println();
}

void generatePulsesManuallyAndWait(uint8_t aPin, uint8_t aDegree, uint16_t aMicroseconds, uint8_t aNumberOfPulses,
        uint16_t aMillisToWait) {
    Serial.print(aPin);
    Serial.print(F(" - "));
    Serial.print(aNumberOfPulses);
    Serial.print(F(" pulses for "));
    Serial.print(aDegree);
    Serial.print(F(" degree | "));
    Serial.print(aMicroseconds);
    Serial.println(F(" us"));
    Serial.flush();

    for (uint_fast8_t i = 0; i < aNumberOfPulses; ++i) {
        noInterrupts();
        digitalWriteFast(aPin, HIGH);
        delayMicroseconds(aMicroseconds);
        digitalWriteFast(aPin, LOW);
        interrupts();

        delay(19);
    }
    delay(aMillisToWait);
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}
