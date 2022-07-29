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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

#include "QuadrupedConfiguration.h"
#include "QuadrupedHelper.h"
#include "ADCUtils.h" // for getVCCVoltageMillivoltSimple()
#include "QuadrupedBasicMovements.h" // for sCurrentlyRunningAction

#if defined(QUADRUPED_HAS_NEOPIXEL)
#include "QuadrupedNeoPixel.h"
#endif

#if defined(QUADRUPED_ENABLE_RTTTL)
#define SUPPRESS_HPP_WARNING
#include <PlayRtttl.h>
#endif

#if defined(QUADRUPED_HAS_IR_CONTROL)
#include "IRCommandDispatcher.h"
#endif
#include "QuadrupedServoControl.h"

#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

bool isShutDown = false;
bool doShutDown = false;

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
 * Called exclusively by main loop
 */
void checkForLowVoltageAndShutdown() {
    // Reset shutdown and enable new check, if a new command is running
    if (sCurrentlyRunningAction != ACTION_TYPE_STOP) {
        isShutDown = false;
    }
    if (doShutDown || (!isShutDown && checkForLowVoltage())) {
#if defined(QUADRUPED_HAS_NEOPIXEL)
        doWipeOutPatterns(); // to save power
#endif
#if defined(QUADRUPED_HAS_IR_CONTROL)
        IRDispatcher.setRequestToStopReceived(false); // This enables shutdown move and new moves
#endif
        shutdownServos();
        playShutdownMelody();
        doShutDown = false; // shutdown done
        isShutDown = true; // do it only once until next command arrives
    }
}

/*
 * Special delay function for the quadruped control.
 * It checks for low voltage and returns prematurely if requestToStopReceived is set
 * @return  true - if stop received
 */
bool delayAndCheckForLowVoltageAndStop(uint16_t aDelayMillis) {
    isShutDown = false; // here we are in a (new) command and can enable a shutdown

// check voltage only once per delay
    if (!doShutDown && checkForLowVoltage()) { // !doShutDown enables shutdownServos() to run
        // Voltage is low here
#if defined(QUADRUPED_HAS_IR_CONTROL)
        IRDispatcher.setRequestToStopReceived(); // This in turn stops any movement -> do not forget to reset it before next movement!
#endif
        doShutDown = true;
        sCurrentlyRunningAction = ACTION_TYPE_STOP; // required for QUADRUPED_RETURN_IF_STOP
        return true;
#if defined(QUADRUPED_HAS_IR_CONTROL)
    } else {
        // Voltage is OK here :-)
        if (IRDispatcher.delayAndCheckForStop(aDelayMillis)) {
            Serial.println(F("Stop requested"));
            sCurrentlyRunningAction = ACTION_TYPE_STOP;
            return true;
        }
#else
        delay(aDelayMillis);
#endif
    }
    return false;
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
    static uint8_t sLastDistance;
    if (millis() - sLastMeasurementMillis > MILLIS_BETWEEN_MEASUREMENTS) {
        sLastMeasurementMillis = millis();
        uint8_t tDistance = getUSDistanceAsCentimeter(10000);
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
                 * The first 3 pixel represent a distance of each 5 cm
                 * The 4. pixel is active if distance is >= 20 cm and < 30 cm
                 * The 7. pixel is active if distance is >= 80 cm and < 120 cm
                 * The 8. pixel is active if distance is >= 1.2 meter
                 */
                uint8_t tBarLength;
                if (tDistance < 20) {
                    tBarLength = tDistance / 5;
                } else if (tDistance < 30) {
                    tBarLength = 4;
                } else if (tDistance < 50) {
                    tBarLength = 5;
                } else if (tDistance < 80) {
                    tBarLength = 6;
                } else if (tDistance < 120) {
                    tBarLength = 7;
                } else {
                    tBarLength = 8;
                }
                FrontNeoPixelBar.drawBarFromColorArray(tBarLength, sBarBackgroundColorArrayForDistance);
                showPatternSynchronizedWithServos();
            }
#  endif
        }
    }
}
#endif // #if defined(QUADRUPED_HAS_US_DISTANCE)

#if defined(QUADRUPED_HAS_IR_CONTROL)
#  if defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
#define SERVO_INCREMENT     5
void doUSRight() {
    int tServoPosition = ServoEasing::ServoEasingNextPositionArray[INDEX_OF_US_DISTANCE_SERVO];
    if (tServoPosition >= SERVO_INCREMENT) {
        USServo.write(tServoPosition - SERVO_INCREMENT);
    }
}
void doUSLeft() {
    int tServoPosition = ServoEasing::ServoEasingNextPositionArray[INDEX_OF_US_DISTANCE_SERVO];
    if (tServoPosition <= (180 - SERVO_INCREMENT)) {
        USServo.write(tServoPosition + SERVO_INCREMENT);
    }
}
void doUSScan() {
    USServo.write(90);
}
#  endif

#if defined(QUADRUPED_ENABLE_RTTTL)
void doRandomMelody() {
    sCurrentlyRunningAction = ACTION_TYPE_MELODY;
    startPlayRandomRtttlFromArrayPGMAndPrintName(PIN_BUZZER, RTTTLMelodiesSmall,
    ARRAY_SIZE_MELODIES_SMALL, &Serial, NULL);
    sAtLeastOnePatternsIsActive = false; // disable any pattern, which disturbs the melody
}
#endif

#  if !defined(USE_USER_DEFINED_MOVEMENTS)
// Disable doCalibration() also for user defined movements, since just testing the remote may lead to accidental wrong calibration

#include "IRCommandMapping.h" // for COMMAND_* definitions in doCalibration()

/*
 * Signals which leg is to be calibrated
 */
void signalLeg(uint8_t aPivotServoIndex) {
    ServoEasing::ServoEasingArray[aPivotServoIndex + LIFT_SERVO_OFFSET]->easeTo(LIFT_HIGHEST_ANGLE, 60);
    ServoEasing::ServoEasingArray[aPivotServoIndex]->easeTo(90, 60);
    ServoEasing::ServoEasingArray[aPivotServoIndex + LIFT_SERVO_OFFSET]->easeTo(90, 60);
}

/*
 * Changes the servo calibration values in EEPROM.
 * Starts with front left i.e. ServoEasing::ServoEasingArray[0,1] and switches to the next leg with the COMMAND_ENTER
 * Is only available if IR control is attached
 */
void doCalibration() {
    uint8_t tPivotServoIndex = 0; // start with front left i.e. ServoEasing::ServoEasingArray[0]
    bool tGotExitCommand = false;
    resetServosTo90Degree();
    delay(500);
    signalLeg(tPivotServoIndex);
#  if defined(INFO)
    Serial.println(F("Entered calibration. Use the forward/backward right/left buttons to set the servo position to 90 degree."));
    Serial.println(F("Use enter/OK button to go to next leg. Values are stored at receiving a different button or after 4th leg."));
#  endif

    IRDispatcher.doNotUseDispatcher = true; // disable dispatcher by mapping table
    while (!tGotExitCommand) {
        // wait until next command received
        while (!IRDispatcher.IRReceivedData.isAvailable) {
        }
        IRDispatcher.IRReceivedData.isAvailable = false;

        unsigned long tIRCode = IRDispatcher.IRReceivedData.command;
#  if defined(INFO)
        IRDispatcher.printIRCommandString(&Serial);
#  endif
        switch (tIRCode) {
        case COMMAND_RIGHT:
            sServoTrimAngles[tPivotServoIndex]++;
            ServoEasing::ServoEasingArray[tPivotServoIndex]->setTrim(sServoTrimAngles[tPivotServoIndex], true);
            break;
        case COMMAND_LEFT:
            sServoTrimAngles[tPivotServoIndex]--;
            ServoEasing::ServoEasingArray[tPivotServoIndex]->setTrim(sServoTrimAngles[tPivotServoIndex], true);
            break;
        case COMMAND_FORWARD:
            sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]++;
            ServoEasing::ServoEasingArray[tPivotServoIndex + LIFT_SERVO_OFFSET]->setTrim(
                    sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET], true);
            break;
        case COMMAND_BACKWARD:
            sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]--;
            ServoEasing::ServoEasingArray[tPivotServoIndex + LIFT_SERVO_OFFSET]->setTrim(
                    sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET], true);
            break;
        case COMMAND_ENTER:
            // show 135 and 45 degree positions
            ServoEasing::ServoEasingArray[tPivotServoIndex]->easeTo(135, 100);
            delay(2000);
            ServoEasing::ServoEasingArray[tPivotServoIndex]->easeTo(45, 100);
            delay(2000);
            ServoEasing::ServoEasingArray[tPivotServoIndex]->easeTo(90, 100);
            tPivotServoIndex += SERVOS_PER_LEG;
            eepromWriteServoTrim();
            if (tPivotServoIndex >= NUMBER_OF_LEG_SERVOS) {
                tGotExitCommand = true;
            } else {
                signalLeg(tPivotServoIndex);
            }
            break;
        case COMMAND_CALIBRATE:
            // repeated command here
            break;
        default:
            eepromWriteServoTrim();
            tGotExitCommand = true;
            break;
        }
#  if defined(INFO)
        Serial.print(F("ServoTrimAngles["));
        Serial.print(tPivotServoIndex);
        Serial.print(F("]="));
        Serial.print(sServoTrimAngles[tPivotServoIndex]);
        Serial.print(F(" ["));
        Serial.print(tPivotServoIndex + LIFT_SERVO_OFFSET);
        Serial.print(F("]="));
        Serial.println(sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]);
#  endif
        ServoEasing::ServoEasingArray[tPivotServoIndex]->print(&Serial);
        ServoEasing::ServoEasingArray[tPivotServoIndex + LIFT_SERVO_OFFSET]->print(&Serial);
        delay(200);
    }
    IRDispatcher.doNotUseDispatcher = false; // re enable dispatcher by mapping table

}
#  endif // !defined(USE_USER_DEFINED_MOVEMENTS)
#endif // defined(QUADRUPED_HAS_IR_CONTROL)

void doBeep() {
    tone(PIN_BUZZER, 2000, 200);
    delay(400);
    tone(PIN_BUZZER, 2000, 200);
}
