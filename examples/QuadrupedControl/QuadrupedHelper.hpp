/*
 * QuadrupedHelper.hpp
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef _QUADRUPED_HELPER_HPP
#define _QUADRUPED_HELPER_HPP

#include <Arduino.h>

#include "digitalWriteFast.h"

#include "QuadrupedHelper.h"
#include "ADCUtils.h" // for getVCCVoltageMillivoltSimple()
#include "QuadrupedBasicMovements.h" // for sCurrentlyRunningAction
#include "QuadrupedControlCommands.h"

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

//#if defined(QUADRUPED_HAS_US_DISTANCE)
//uint32_t sLastDistanceMeasurementMillis;
//uint8_t sCurrentDistance;
//uint8_t sLastDistance;
//#endif

uint32_t sLastVolageMeasurementMillis;

#include "QuadrupedServoControl.h"

#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

bool isShutDown = false;

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
bool checkForVCCUnderVoltage() {
    sVCCVoltageMillivolt = getVCCVoltageMillivoltSimple();
    return (sVCCVoltageMillivolt <= VCC_STOP_THRESHOLD_MILLIVOLT);
}

/*
 * Called exclusively by main loop
 */
void checkForVCCUnderVoltageAndShutdown() {
    if ((millis() - sLastVolageMeasurementMillis) <= MILLIS_BETWEEN_VOLTAGE_MEASUREMENTS) {
        return;
    }

    // Reset shutdown and enable new check, if a new command is running
    if (sCurrentlyRunningAction != ACTION_TYPE_STOP) {
        isShutDown = false;
        sShutdownCount = 0;
    }

    if (!checkForVCCUnderVoltage()) {
        isShutDown = false;
        sShutdownCount--;
    } else if (!isShutDown) {
        sShutdownCount++;
        if (sShutdownCount == NUMBER_OF_VOLTAGE_LOW_FOR_SHUTDOWN) {
            /*
             * Transition to low voltage here
             */
            // print message
            Serial.print(F("VCC "));
            Serial.print(sVCCVoltageMillivolt);
            Serial.println(F(" below " STR(VCC_STOP_THRESHOLD_MILLIVOLT) " Millivolt"));

#if defined(QUADRUPED_HAS_NEOPIXEL)
        doWipeOutPatterns(); // to save power
#endif

#if defined(QUADRUPED_HAS_IR_CONTROL)
        IRDispatcher.setRequestToStopReceived(false); // This enables shutdown move and new moves
#endif

            shutdownServos();
            playShutdownMelody();
            isShutDown = true; // Do it only once. Next (IR) command resets this flag
        }
    }
}

/*
 * Special delay function for the quadruped control.
 * It returns prematurely if requestToStopReceived is set
 * @return  true - if stop received
 */
bool delayAndCheckForStopByIR(uint16_t aDelayMillis) {
#if defined(QUADRUPED_HAS_US_DISTANCE)
    // adjust delay value
    // we know, that delay is <= 10 and aDelayMillis is > =19
     handleUSSensor();
     aDelayMillis -= sUsedMillisForMeasurement;
#endif
#if defined(QUADRUPED_HAS_IR_CONTROL)
    if (IRDispatcher.delayAndCheckForStop(aDelayMillis)) {
        Serial.println(F("Stop requested"));
        sCurrentlyRunningAction = ACTION_TYPE_STOP;
        return true;
    }

#else
    delay(aDelayMillis);
#endif
    return false;
}

#if defined(QUADRUPED_HAS_US_DISTANCE)
/**
 * Get US distance and display it on front bar
 * Can introduce a delay up to 10 ms
 * @return ms of delay, that was introduced
 */
void handleUSSensor() {
#  if defined(QUADRUPED_ENABLE_RTTTL)
    // 2 measurements per second even when melody is playing
    if (isPlayRtttlRunning() && (millis() - sLastUSDistanceMeasurementMillis) <= MILLIS_BETWEEN_DISTANCE_MEASUREMENTS_FOR_MELODY) {
        return; // showPatternSynchronizedWithServos() disturbs the melody
    }
#  endif

    if (isShutDown) {
        return; // no measurement required
    }

    if(getUSDistanceAsCentimeterWithCentimeterTimeoutPeriodicallyAndPrintIfChanged( MAX_DISTANCE_TIMEOUT_CM,
            MILLIS_BETWEEN_DISTANCE_MEASUREMENTS, &Serial)){
#  if defined(QUADRUPED_HAS_NEOPIXEL)
        // Show distance bar if no other pattern is active
        if (FrontNeoPixelBar.ActivePattern == PATTERN_NONE && QuadrupedNeoPixelBar.ActivePattern == PATTERN_NONE) {
            /*
             * The first 3 pixel represent a distance of each 5 cm
             * The 4. pixel is active if distance is >= 20 cm and < 30 cm
             * The 7. pixel is active if distance is >= 80 cm and < 120 cm
             * The 8. pixel is active if distance is >= 1.2 meter
             */
            uint8_t tBarLength;
            if (sUSDistanceCentimeter < 20) {
                tBarLength = sUSDistanceCentimeter / 5;
            } else if (sUSDistanceCentimeter < 30) {
                tBarLength = 4;
            } else if (sUSDistanceCentimeter < 50) {
                tBarLength = 5;
            } else if (sUSDistanceCentimeter < 80) {
                tBarLength = 6;
            } else if (sUSDistanceCentimeter < 120) {
                tBarLength = 7;
            } else {
                tBarLength = 8;
            }
            FrontNeoPixelBar.drawBarFromColorArray(tBarLength, sBarBackgroundColorArrayForDistance);
            showPatternSynchronizedWithServos();
        }
#  endif
        if (sCurrentlyRunningAction == ACTION_TYPE_STOP && sCurrentlyRunningCombinedAction == ACTION_TYPE_STOP){
#  if defined(QUADRUPED_ENABLE_RTTTL)
            /*
             * Play melody if distance is below 10 cm and above 3 cm
             */
            if (sCurrentlyRunningAction == ACTION_TYPE_STOP && sUSDistanceCentimeter >= 3 && sUSDistanceCentimeter <= MIN_DISTANCE_FOR_MELODY_CM && !isPlayRtttlRunning()) {
                randomSeed(millis());
                doRandomMelody();
                return; // do not check for wave!
            }
#  endif
            /*
             * Do wave if distance is between 35 cm and 25 cm
             * Fist wave can be done 10 seconds after boot
             */
            if (sCurrentlyRunningAction == ACTION_TYPE_STOP && sUSDistanceCentimeter >= MIN_DISTANCE_FOR_WAVE_CM && sUSDistanceCentimeter <= MAX_DISTANCE_FOR_WAVE_CM
#if defined(QUADRUPED_HAS_IR_CONTROL)
                && (millis() - IRDispatcher.IRReceivedData.MillisOfLastCode) > MILLIS_BETWEEN_WAVES
#else
                && (millis() - sMillisOfLastSpecialAction) > MILLIS_BETWEEN_WAVES
#endif
            ) {
                doWave();
#if defined(QUADRUPED_HAS_IR_CONTROL)
                IRDispatcher.IRReceivedData.MillisOfLastCode = millis(); // disable next auto move, next attention in 2 minutes
#else
                sMillisOfLastSpecialAction = millis(); // disable next auto move, next attention in 2 minutes
#endif
            }
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
    sCurrentlyRunningAction = ACTION_TYPE_MELODY; // to make melody stoppable with stop command of IR
    startPlayRandomRtttlFromArrayPGMAndPrintName(PIN_BUZZER, RTTTLMelodiesSmall,
    ARRAY_SIZE_MELODIES_SMALL, &Serial, NULL);
    sAtLeastOnePatternsIsActive = false; // disable any pattern, which disturbs the melody
}
#endif

#  if !defined(USE_USER_DEFINED_MOVEMENTS)
// Disable doCalibration() also for user defined movements, since just testing the remote may lead to accidental wrong calibration

#include "QuadrupedIRCommandMapping.h" // for COMMAND_* definitions in doCalibration()

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
#if    E2END

    uint8_t tPivotServoIndex = 0; // start with front left i.e. ServoEasing::ServoEasingArray[0]
    bool tGotExitCommand = false;
    resetServosTo90Degree();
    delay(500);
    signalLeg(tPivotServoIndex);
#      if defined(INFO)
    Serial.println(F("Entered calibration. Use the forward/backward right/left buttons to set the servo position to 90 degree."));
    Serial.println(F("Use enter/OK button to go to next leg. Values are stored at receiving a different button or after 4th leg."));
#      endif

    IRDispatcher.doNotUseDispatcher = true; // disable dispatcher by mapping table
    while (!tGotExitCommand) {
        // wait until next command received
        while (!IRDispatcher.IRReceivedData.isAvailable) {
        }
        IRDispatcher.IRReceivedData.isAvailable = false;

        unsigned long tIRCode = IRDispatcher.IRReceivedData.command;
#      if defined(INFO)
        Serial.print(F("IRCommand="));
        IRDispatcher.printIRCommandString(&Serial);
#      endif
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
#      if defined(INFO)
        Serial.print(F("ServoTrimAngles["));
        Serial.print(tPivotServoIndex);
        Serial.print(F("]="));
        Serial.print(sServoTrimAngles[tPivotServoIndex]);
        Serial.print(F(" ["));
        Serial.print(tPivotServoIndex + LIFT_SERVO_OFFSET);
        Serial.print(F("]="));
        Serial.println(sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]);
#      endif
        ServoEasing::ServoEasingArray[tPivotServoIndex]->print(&Serial);
        ServoEasing::ServoEasingArray[tPivotServoIndex + LIFT_SERVO_OFFSET]->print(&Serial);
        delay(200);
    }
    IRDispatcher.doNotUseDispatcher = false; // re enable dispatcher by mapping table
#    endif
}
#  endif // !defined(USE_USER_DEFINED_MOVEMENTS)
#endif // defined(QUADRUPED_HAS_IR_CONTROL)

void doBeep() {
    tone(PIN_BUZZER, 2000, 200);
    delay(400);
    tone(PIN_BUZZER, 2000, 200);
}

#endif // _QUADRUPED_HELPER_HPP
