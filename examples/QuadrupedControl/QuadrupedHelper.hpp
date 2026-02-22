/*
 * QuadrupedHelper.hpp
 *
 *  Contains miscellaneous function for the quadruped
 *
 *  Copyright (C) 2022-2026  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of QuadrupedControl https://github.com/ArminJo/QuadrupedControl.
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  QuadrupedControl and ServoEasing are free software: you can redistribute it and/or modify
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

// This block must be located after the includes of other *.hpp files
//#define LOCAL_INFO  // This enables info output only for this file
#include "LocalDebugLevelStart.h"

#include "QuadrupedConfiguration.h" // is mainly included here for the eclipse auto formatter

#include "digitalWriteFast.h"

#include "QuadrupedHelper.h"
#include "ADCUtils.h" // for getVCCVoltageMillivoltSimple()
#include "QuadrupedBasicMovements.h" // for sCurrentlyRunningAction
#include "QuadrupedControlCommands.h"

#if defined(QUADRUPED_HAS_NEOPIXEL)
#include "QuadrupedNeoPixel.h"
#endif

#if defined(QUADRUPED_HAS_IR_CONTROL)
#include "IRCommandDispatcher.h"
#endif

uint32_t sLastVolageMeasurementMillis;

#include "QuadrupedServoControl.h"

bool isShutDownByUndervoltage = false;

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

#if defined(QUADRUPED_ENABLE_RTTTL)
void doRandomMelody() {
    randomSeed(millis());
    sCurrentlyRunningAction = ACTION_TYPE_MELODY; // to make melody stoppable with stop command of IR
    startPlayRandomRtttlFromArrayPGMAndPrintName(PIN_BUZZER, RTTTLMelodiesSmall, ARRAY_SIZE_MELODIES_SMALL, &Serial, nullptr);
    sAtLeastOnePatternsIsActive = false; // disable any pattern, which disturbs the melody
}
#endif

#if defined(ADC_UTILS_ARE_AVAILABLE)
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
    if (isShutDownByUndervoltage && sCurrentlyRunningAction != ACTION_TYPE_STOP) {
        isShutDownByUndervoltage = false;
        sShutdownCount = 0;
        Serial.println(F("Command is running, reset shutdown flag"));

    }

    if (!checkForVCCUnderVoltage()) {
        isShutDownByUndervoltage = false;
        sShutdownCount--;
    } else if (!isShutDownByUndervoltage) {
        sShutdownCount++;
        if (sShutdownCount == NUMBER_OF_VOLTAGE_LOW_FOR_SHUTDOWN) {
            /*
             * Transition to low voltage here
             */
            // print message
            Serial.print(F("VCC "));
            Serial.print(sVCCVoltageMillivolt);
            Serial.println(F(" is below " STR(VCC_STOP_THRESHOLD_MILLIVOLT) " Millivolt"));

#if defined(QUADRUPED_HAS_NEOPIXEL)
            doWipeOutPatterns(); // to save power
#endif

#if defined(QUADRUPED_HAS_IR_CONTROL)
            IRDispatcher.setRequestToStopReceived(false); // This enables shutdown move and new moves
#endif

            shutdownServos();
            playShutdownMelody();
            isShutDownByUndervoltage = true; // Do it only once. Next (IR) command resets this flag
        }
    }
}
#endif

/*
 * Special delay function for the quadruped control.
 * It returns prematurely if requestToStopReceived is set
 * @return  true - if stop received
 */
#define APPLICATION_DELAY_AND_CHECK_AVAILABLE
bool delayAndCheckByApplication(uint16_t aDelayMillis) {
#if defined(QUADRUPED_HAS_US_DISTANCE)
    handleUSDistanceSensor();
    // adjust delay value. we know, that delay is <= 10 and aDelayMillis is > =19, so we have no underflow
    aDelayMillis -= sUsedMillisForUSDistanceMeasurement;
#endif
#if defined(QUADRUPED_HAS_IR_CONTROL)
    if (IRDispatcher.delayAndCheckForStop(aDelayMillis)) {
        Serial.println(F("Stop requested"));
        return true;
    }
#else
    delay(aDelayMillis);
#endif
    return false;
}

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
#  endif // defined(QUADRUPED_HAS_US_DISTANCE_SERVO)

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
#    if E2END

    uint8_t tPivotServoIndex = 0; // start with front left i.e. ServoEasing::ServoEasingArray[0]
    bool tGotExitCommand = false;
    resetServosTo90Degree();
    delay(500);
    signalLeg(tPivotServoIndex);
    INFO_PRINTLN(F("Entered calibration. Use the forward/backward right/left buttons to set the servo position to 90 degree."));
    INFO_PRINTLN(F("Use enter/OK button to go to next leg. Values are stored at receiving a different button or after 4th leg."));

    IRDispatcher.doNotUseDispatcher = true; // disable dispatcher by mapping table
    while (!tGotExitCommand) {
        // wait until next command received
        while (!IRDispatcher.IRReceivedData.isAvailable) {
        }
        IRDispatcher.IRReceivedData.isAvailable = false;

        unsigned long tIRCode = IRDispatcher.IRReceivedData.command;
#      if defined(LOCAL_INFO)
        Serial.print(F("IRCommand="));
        IRDispatcher.printIRCommandString(&Serial, IRDispatcher.IRReceivedData.command);
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
        INFO_PRINT(F("ServoTrimAngles["));
        INFO_PRINT(tPivotServoIndex);
        INFO_PRINT(F("]="));
        INFO_PRINT(sServoTrimAngles[tPivotServoIndex]);
        INFO_PRINT(F(" ["));
        INFO_PRINT(tPivotServoIndex + LIFT_SERVO_OFFSET);
        INFO_PRINT(F("]="));
        INFO_PRINTLN(sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]);
        ServoEasing::ServoEasingArray[tPivotServoIndex]->print(&Serial);
        ServoEasing::ServoEasingArray[tPivotServoIndex + LIFT_SERVO_OFFSET]->print(&Serial);
        delay(200);
    }
    IRDispatcher.doNotUseDispatcher = false; // re enable dispatcher by mapping table
#    endif // E2END
}
#  endif // !defined(USE_USER_DEFINED_MOVEMENTS)
#endif // defined(QUADRUPED_HAS_IR_CONTROL)

void doBeep() {
    tone(PIN_BUZZER, 2000, 200);
    delay(400);
    tone(PIN_BUZZER, 2000, 200);
}

#include "LocalDebugLevelEnd.h"

#endif // _QUADRUPED_HELPER_HPP
