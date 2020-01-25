/*
 * QuadrupedControl.cpp
 *
 * Program for controlling a mePed Robot V2 with 8 servos
 * Full version also controls IR remote receiver, 3 NeoPixel bars and a HCSR04 US distance module.
 * The IR remote receiver is attached at pin A0. Supported IR remote controls are:
 *          KEYES (the original mePed remote)
 *          KEYES_CLONE (the one with numberpad and direction control swapped, which you get when you buy a KEYES at aliexpress).
 *          WM10
 * If you use another than the KEYES_CLONE, you have to select the one you use at line 20ff. in IRCommandMapping.h
 *
 * To run this example you need to install the "ServoEasing"
 * For full control install also "IRLremote", "PinChangeInterrupt", "NeoPatterns" and "Adafruit_NeoPixel" libraries.
 * These libraries can be installed under "Tools -> Manage Libraries..." or "Ctrl+Shift+I".
 *
 *  Copyright (C) 2019  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of QuadrupedControl https://github.com/ArminJo/QuadrupedControl.
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
 */

#include <Arduino.h>

#include "QuadrupedControl.h"

#if defined(QUADRUPED_HAS_IR_CONTROL)
#include "IRCommandDispatcher.h"
#include "IRCommandMapping.h" // for IR_REMOTE_NAME
#endif

#include "QuadrupedServoControl.h"
#include "QuadrupedMovements.h"
#include "Commands.h"
#include "ADCUtils.h"

#if defined(QUADRUPED_PLAYS_RTTTL)
#include <PlayRtttl.h>
#endif

#if defined(QUADRUPED_HAS_NEOPIXEL)
#include "QuadrupedNeoPixel.h"
#endif

#if defined(QUADRUPED_HAS_US_DISTANCE)
#include "HCSR04.h"
ServoEasing USServo;    // Servo for US sensor
#endif

//#define INFO // comment this out to see serial info output

#if defined(QUADRUPED_HAS_NEOPIXEL)
color32_t sBarBackgroundColorArray[PIXELS_ON_ONE_BAR] = { COLOR32_RED_QUARTER, COLOR32_RED_QUARTER, COLOR32_RED_QUARTER,
COLOR32_YELLOW, COLOR32_YELLOW, COLOR32_GREEN_QUARTER, COLOR32_GREEN_QUARTER, COLOR32_GREEN_QUARTER };
#endif

#define VERSION_EXAMPLE "3.0"
// 3.0 NeoPixel and distance sensor added
// 2.1 auto move
// 2.0 major refactoring
// 1.1 mirror computation at transformAndSetPivotServos and transformOneServoIndex

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__)
    while (!Serial); //delay for Leonardo, but this loops forever for Maple Serial
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    setupQuadrupedServos();
    setSpeedForAllServos(sServoSpeed);

    // Just for setting channel and reference
    printVCCVoltageMillivolt(&Serial);

#if defined(QUADRUPED_HAS_US_DISTANCE)
    Serial.println(F("Init US distance sensor"));
    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);
    USServo.attach(PIN_US_SERVO);
    USServo.write(90); // start value
#endif

    /*
     * set servo to 90 degree WITHOUT trim and wait 2 seconds
     */
    resetServosTo90Degree();
    delay(2000);

    /*
     * Read trim values and set servo to 90 degree with trim and wait
     */
    eepromReadAndSetServoTrim();
//    resetServosTo90Degree();

#if defined(QUADRUPED_PLAYS_RTTTL)
    playRtttlBlockingPGM(PIN_SPEAKER, Short);
#else
    tone(PIN_SPEAKER, 2000, 300);
#endif

    delay(1000);

    /*
     * Set to initial height
     */
    centerServos();
    convertBodyHeightAngleToHeight();

#if defined(QUADRUPED_HAS_IR_CONTROL)
    setupIRDispatcher();
    Serial.print(F("Listening to IR remote of type "));
    Serial.println(IR_REMOTE_NAME);
#endif

#if defined(QUADRUPED_HAS_NEOPIXEL)
    Serial.println(F("Init NeoPixel"));
    initNeoPatterns();
    enableServoEasingInterrupt(); // This enables the interrupt, which synchronizes the NeoPixel update with the servo pulse generation.
#endif

}

void loop() {
    /*
     * Reset moving characteristic
     */
    setEasingTypeToLinear();

#if defined(QUADRUPED_HAS_US_DISTANCE)
    handleUSSensor();
#endif

#if defined(QUADRUPED_HAS_IR_CONTROL)
    /*
     * Check for IR commands and execute them.
     * Returns only AFTER finishing of requested movement
     */
    loopIRDispatcher();
#if defined(QUADRUPED_HAS_NEOPIXEL)
    if (sJustCalledExclusiveIRCommand) {
        sJustCalledExclusiveIRCommand = false;
        sStartOrChangeNeoPatterns = true;
    }
#endif

    /*
     * Do auto move if timeout after boot was reached and no IR command was received
     */
    if (!sAtLeastOneValidIRCodeReceived && (millis() > MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE)) {
#if defined(QUADRUPED_HAS_NEOPIXEL)
        wipeOutPatternsBlocking();
#endif
#if !defined(USE_USER_DEFINED_MOVEMENTS)
        doQuadrupedAutoMove();
#endif
        sAtLeastOneValidIRCodeReceived = true; // do auto move only once
    }

    /*
     * Get attention that no command was received since 2 minutes and quadruped may be switched off
     */
    if (millis() - sLastTimeOfIRCodeReceived > MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE) {
        doAttention();
        printVCCVoltageMillivolt(&Serial);
        // next attention in 1 minute
        sLastTimeOfIRCodeReceived += MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE;
    }
#else
    delayAndCheck(5000);
    doQuadrupedAutoMove();
    delayAndCheck(25000);
#endif

    if (checkForLowVoltage()) {
        shutdownServos();
        tone(PIN_SPEAKER, 2000, 200);
        delay(400);
        tone(PIN_SPEAKER, 1400, 300);
        delay(600);
        tone(PIN_SPEAKER, 1000, 400);
        delay(800);
        tone(PIN_SPEAKER, 700, 500);
#if defined(QUADRUPED_HAS_NEOPIXEL)
        wipeOutPatternsBlocking();
#endif
        delay(10000);  // blocking wait for next check
    }
}

/*
 * Stop servos if voltage gets low
 * @return  true - if voltage too low
 */
bool checkForLowVoltage() {
    uint16_t tVCC = getVCCVoltageMillivoltSimple();
    if (tVCC > VCC_STOP_THRESHOLD_MILLIVOLT) {
        return false; // signal OK
    }
    /*
     * Low voltage here
     */
    Serial.print(F("VCC "));
    Serial.print(tVCC);
    Serial.print(F(" below 3600 Millivolt -> "));
    return true;
}

#if defined(QUADRUPED_HAS_US_DISTANCE)
/*
 * Get front distance
 */
void handleUSSensor() {
    static uint32_t sLastMeasurementMillis;
    static uint16_t sLastDistance;
    if (millis() - sLastMeasurementMillis > MILLIS_BETWEEN_MEASUREMENTS) {
        sLastMeasurementMillis = millis();
        uint16_t tDistance = getUSDistanceAsCentiMeter();
        if (sLastDistance != tDistance) {
            sLastDistance = tDistance;
#ifdef INFO
//            Serial.print(F("Distance="));
//            Serial.print(tDistance);
//            Serial.println(F("cm"));
#endif
#if defined(QUADRUPED_HAS_NEOPIXEL)
            // Show bar if no other pattern is active
            if (FrontNeoPixelBar.ActivePattern == PATTERN_NONE) {
                /*
                 * The first 6 pixel represent a distance of each 5 cm
                 * The 7. pixel is active if distance is > 50 centimeter
                 * The 8. pixel is active if distance is > 1 meter
                 */
                uint8_t tBarLength;
                if (tDistance > 100) {
                    tBarLength = 8;
                } else if (tDistance > 50) {
                    tBarLength = 7;
                } else {
                    tBarLength = tDistance / 5;
                }
                FrontNeoPixelBar.drawBarFromColorArray(tBarLength, sBarBackgroundColorArray);
                showPatternSynchronized();
            }
#endif
        }
    }
}

void doUSRight() {
    if (!sCurrentCommandIsRepeat && USServo.getCurrentAngle() > 15) {
        USServo.write(USServo.getCurrentAngle() - 15);
    }
}
void doUSLeft() {
    if (!sCurrentCommandIsRepeat && USServo.getCurrentAngle() < 165) {
        USServo.write(USServo.getCurrentAngle() + 15);
    }
}
void doUSScan() {
    if (!sCurrentCommandIsRepeat) {
        USServo.write(90);
    }
}
#endif

/*
 * Special delay function for the quadruped control.
 * It checks for low voltage, IR input and US distance sensor
 * @return  true - if exit condition occurred like stop received
 */
bool delayAndCheck(uint16_t aDelayMillis) {
    uint32_t tStartMillis = millis();

// check only once per delay
    if (!checkForLowVoltage()) {
        do {
#if defined(QUADRUPED_HAS_IR_CONTROL)
            if (checkIRInputForNonExclusiveCommand()) {
                Serial.println(F("Invalid or exclusive command received -> set stop"));
                sActionType = ACTION_TYPE_STOP;
                return true;
            }
#endif
            yield();
        } while (millis() - tStartMillis < aDelayMillis);
        return false;
    }
#if defined(QUADRUPED_HAS_IR_CONTROL)
    sActionType = ACTION_TYPE_STOP;
#endif
    return true;
}
