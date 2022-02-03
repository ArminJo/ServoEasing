/*
 * QuadrupedHelper.cpp
 *
 *  Contains miscellaneous function for the quadruped
 *
 *  Copyright (C) 2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of QuadrupedControl https://github.com/ArminJo/QuadrupedControl.
 *
 *  QuadrupedControl is free software: you can redistribute it and/or modify
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

#include "QuadrupedConfiguration.h"
#include "ADCUtils.h" // for getVCCVoltageMillivoltSimple()
#include "QuadrupedControlCommands.h" // for sCurrentlyRunningAction

#if defined(QUADRUPED_HAS_NEOPIXEL)
#include "QuadrupedNeoPixel.h"
#endif

#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

void playShutdownMelody() {
#if defined(QUADRUPED_ENABLE_RTTTL)
    playRtttlBlockingPGM(PIN_BUZZER, Down);
#else
    tone(PIN_BUZZER, 2000, 200);
    delay(400);
    tone(PIN_BUZZER, 1400, 300);
    delay(600);
    tone(PIN_BUZZER, 1000, 400);
    delay(800);
    tone(PIN_BUZZER, 700, 600);
#endif
}

/*
 * Stop servos if voltage gets low
 * @return  true - if voltage too low
 */
bool checkForLowVoltage() {
    uint16_t tVCC = getVCCVoltageMillivoltSimple();
    if (tVCC > VCC_STOP_THRESHOLD_MILLIVOLT) {
        return false; // signal voltage is OK
    }
    /*
     * Low voltage here
     */
    Serial.print(F("VCC "));
    Serial.print(tVCC);
    Serial.println(F(" below " STR(VCC_STOP_THRESHOLD_MILLIVOLT) " Millivolt"));
    return true;
}

#if defined(QUADRUPED_HAS_US_DISTANCE)
/*
 * Get front distance and display it on front bar
 */
void handleUSSensor() {
    static uint32_t sLastMeasurementMillis;
    static uint16_t sLastDistance;
    if (millis() - sLastMeasurementMillis > MILLIS_BETWEEN_MEASUREMENTS) {
        sLastMeasurementMillis = millis();
        uint16_t tDistance = getUSDistanceAsCentimeter();
        if (tDistance != 0 && sLastDistance != tDistance) {
            sLastDistance = tDistance;
#  if defined(INFO)
//            Serial.print(F("Distance="));
//            Serial.print(tDistance);
//            Serial.println(F("cm"));
#  endif
#  if defined(QUADRUPED_HAS_NEOPIXEL)
            // Show distance bar if no other pattern is active
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
                FrontNeoPixelBar.drawBarFromColorArray(tBarLength, sBarBackgroundColorArrayForDistance);
                showPatternSynchronizedWithServos();
            }
#  endif
        }
    }
}
#endif // #if defined(QUADRUPED_HAS_US_DISTANCE)

#if defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
#define SERVO_INCREMENT     10
void doUSRight() {
    if (!IRDispatcher.IRReceivedData.isRepeat && USServo.read() >= SERVO_INCREMENT) {
        USServo.write(USServo.read() - SERVO_INCREMENT);
    }
}
void doUSLeft() {
    if (!IRDispatcher.IRReceivedData.isRepeat && USServo.read() <= (180 - SERVO_INCREMENT)) {
        USServo.write(USServo.read() + SERVO_INCREMENT);
    }
}
void doUSScan() {
    if (!IRDispatcher.IRReceivedData.isRepeat) {
        USServo.write(90);
    }
}
#endif

/*
 * Special delay function for the quadruped control.
 * It checks for low voltage and returns prematurely if requestToStopReceived is set
 * @return  true - if stop received
 */
bool delayAndCheckForLowVoltageAndStop(uint16_t aDelayMillis) {
// check voltage only once per delay
    if (checkForLowVoltage()) {
#if defined(QUADRUPED_HAS_IR_CONTROL)
        sCurrentlyRunningAction = ACTION_TYPE_STOP;
#endif
        return true;
#if defined(QUADRUPED_HAS_IR_CONTROL)
    } else {
        // Voltage is OK here :-)
        if (IRDispatcher.delayAndCheckForStop(aDelayMillis)) {
            Serial.println(F("Stop requested"));
            sCurrentlyRunningAction = ACTION_TYPE_STOP;
            return true;
        }
#endif
    }
    return false;
}

void doBeep() {
    tone(PIN_BUZZER, 2000, 200);
    delay(400);
    tone(PIN_BUZZER, 2000, 200);
}
