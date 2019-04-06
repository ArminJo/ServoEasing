/*
 * RobotArmControl.cpp
 *
 * Program for controlling a RobotArm with 4 servos using 4 potentiometers and/or an IR Remote
 *
 * To run this example need to install the "ServoEasing", "IRLremote" and "PinChangeInterrupt" libraries under Sketch -> Include Library -> Manage Librarys...
 * Use "ServoEasing", "IRLremote" and "PinChangeInterrupt" as filter string.
 *
 *  Copyright (C) 2019  Armin Joachimsmeyer
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
 */

#include <Arduino.h>

#include "RobotArmIRConfiguration.h" // must be before IRLremote.h if we use not pin 2 or 3
#include <IRLremote.h>      // include IR Remote library

#include "ServoEasing.h"    // include servo library

#include "ADCUtils.h" // for get getVCCVoltageMillivolt

#define VERSION_EXAMPLE "1.0"

#define ZERO_DEGREE_VALUE_MICROS    544
#define AT_180_DEGREE_VALUE_MICROS 2400

#define PIVOT_INPUT_PIN         A3
#define HORIZONTAL_INPUT_PIN    A1
#define LIFT_INPUT_PIN          A4
#define CLAW_INPUT_PIN          A2

#define LIFT_MAX_ANGLE          150
#define CLAW_MAX_ANGLE          65

#define PIVOT_START_ANGLE       90
#define HORIZONTAL_START_ANGLE  60
#define LIFT_START_ANGLE       100
#define CLAW_START_ANGLE        50
#define CLAW_CLOSE_ANGLE        CLAW_MAX_ANGLE

// Index into (external) servo array. Order must be the same as of definitions in main.
#define SERVO_BASE_PIVOT 0
#define SERVO_HORIZONTAL 1
#define SERVO_LIFT 2
#define SERVO_CLAW 3
#define NUMBER_OF_SERVOS 4

// Define 8 servos in exact this order!
ServoEasing BasePivotServo;    // 0 - Front Left Pivot Servo
ServoEasing HorizontalServo;     // 1 - Front Left Lift Servo
ServoEasing LiftServo;     // 2 - Back Left Pivot Servo
ServoEasing ClawServo;      // 3 - Back Left Lift Servo

/*
 * Global control parameters
 */
uint16_t sServoSpeed = 40;      // in degree/second

uint8_t sBodyPivotAngle = PIVOT_START_ANGLE;
uint8_t sHorizontalServoAngle = HORIZONTAL_START_ANGLE;
uint8_t sLiftServoAngle = LIFT_START_ANGLE;
uint8_t sClawServoAngle = CLAW_START_ANGLE;

uint8_t sCurrentCommand; // to decide if we must change movement
bool sJustExecutingCommand;
uint8_t sNextCommand = COMMAND_EMPTY;    // if != 0 do not wait for IR just take this command as next

#define MODE_MANUAL 0
#define MODE_IR 1
#define MODE_AUTO 2
uint8_t sMode = MODE_MANUAL;

CNec IRLremote;

/*
 * Function declarations
 */
uint8_t getIRCommand(bool doWait = true);

bool synchronizeMoveAllServosAndCheckInputAndWait();
bool moveOneServoAndCheckInput(uint8_t aServoIndex, uint8_t aDegree, uint16_t aDegreesPerSecond);
bool updateCheckInputAndWaitForAllServosToStop();
bool delayAndCheckInput(uint16_t aDelayMillis);

void setEasingTypeToLinear();

bool setAllServos(uint8_t aNumberOfValues, ...);

void doDelay();
/*
 * Code starts here
 */
void setup() {
    // initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial)
        ; //delay for Leonardo
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // Attach servos to Arduino Pins
    BasePivotServo.attach(4);
    HorizontalServo.attach(5);
    LiftServo.attach(7);
    ClawServo.attach(8);

    setSpeedForAllServos(sServoSpeed);

    // Set to start position. Must be done with write()
    BasePivotServo.write(PIVOT_START_ANGLE);
    HorizontalServo.write(HORIZONTAL_START_ANGLE);
    LiftServo.write(LIFT_START_ANGLE);
    ClawServo.write(CLAW_START_ANGLE);
    delay(2000);

    // Start reading the remote. PinInterrupt or PinChangeInterrupt* will automatically be selected
    if (!IRLremote.begin(IR_RECEIVER_PIN)) {
        Serial.println(F("You did not choose a valid pin"));
    }

    Serial.print(F("Value for 0 degree="));
    Serial.print(ZERO_DEGREE_VALUE_MICROS);
    Serial.print(F("us. Value for 180 degree="));
    Serial.print(AT_180_DEGREE_VALUE_MICROS);
    Serial.println(F("us."));
} //setup

#define IR_LOCK_TIME_MILLIS 10000 // 10 seconds lock of manual input
void loop() {

    static bool sManualActionHappened;
    static bool isInitialized = false;
    static int sLastPivot;
    static int sLastHorizontal;
    static int sLastLift;
    static int sLastClaw;

    uint8_t tIRCode;
    if (sNextCommand == COMMAND_EMPTY) {
        tIRCode = getIRCommand(false);
    } else {
        tIRCode = sNextCommand;
        sNextCommand = COMMAND_EMPTY;
    }

    setEasingTypeToLinear();

// search IR code and call associated function
    if (tIRCode != COMMAND_EMPTY) {
        bool tIRCodeFound = false;
        for (uint8_t i = 0; i < sizeof(IRMW10Mapping) / sizeof(struct IRToCommandMapping); ++i) {
            if (tIRCode == IRMW10Mapping[i].IRCode) {
                sCurrentCommand = tIRCode;
                Serial.print(F("Calling "));
                Serial.println(reinterpret_cast<const __FlashStringHelper *>(IRMW10Mapping[i].CommandString));
                /*
                 * Call the Quadruped function specified in IR mapping
                 */
                sJustExecutingCommand = true;
                IRMW10Mapping[i].CommandToCall();
                sJustExecutingCommand = false;
                tIRCodeFound = true;
                break;
            }
        }
        if (!tIRCodeFound) {
            // must be after IRMW10Mapping search
            checkAndCallInstantCommands(tIRCode);
        }
        if (tIRCodeFound) {
            sMode = MODE_IR;
            Serial.println("Switch to IR mode");
        }
        delay(20); // Pause before executing next movement
    } else if (sMode == MODE_AUTO) {
        setAllServos(4, 90, 120, 20, 50);
        doDelay();

        ClawServo.easeTo(CLAW_CLOSE_ANGLE);
        doDelay();

        setAllServos(3, 40, 40, 120);
        doDelay();

        ClawServo.easeTo(CLAW_CLOSE_ANGLE - 30);
        doDelay();

        setAllServos(3, 140, 40, 165);
        doDelay();

        ClawServo.easeTo(CLAW_CLOSE_ANGLE);
        doDelay();

    } else if (sMode == MODE_MANUAL) {
//        int tVoltageMillivolts = getVCCVoltageMillivolt();

//        Serial.print(F("VCC="));
//        Serial.print(tVoltageMillivolts);
//        Serial.println(" mV");

        int tTargetAngle;
#define ANALOG_HYSTERESIS 6
        int tPivot = analogRead(PIVOT_INPUT_PIN);
        if (abs(sLastPivot - tPivot) > ANALOG_HYSTERESIS) {
            sLastPivot = tPivot;
            tTargetAngle = map(tPivot, 0, 1023, 0, 180);
            BasePivotServo.easeTo(tTargetAngle);
            Serial.print("BasePivotServo: micros=");
            Serial.print(BasePivotServo.getEndMicrosecondsOrUnits());
            Serial.print(" degree=");
            Serial.println(tTargetAngle);
            sManualActionHappened = true;
        }
        int tHorizontal = analogRead(HORIZONTAL_INPUT_PIN);
        if (abs(sLastHorizontal - tHorizontal) > ANALOG_HYSTERESIS) {
            sLastHorizontal = tHorizontal;
            tTargetAngle = map(tHorizontal, 0, 1023, 0, 180);
            HorizontalServo.easeTo(tTargetAngle);
            Serial.print("HorizontalServo: micros=");
            Serial.print(HorizontalServo.getEndMicrosecondsOrUnits());
            Serial.print(" degree=");
            Serial.println(tTargetAngle);
            sManualActionHappened = true;
        }
        int tLift = analogRead(LIFT_INPUT_PIN);
        if (abs(sLastLift - tLift) > ANALOG_HYSTERESIS) {
            sLastLift = tLift;
            tTargetAngle = map(tLift, 0, 1023, 0, LIFT_MAX_ANGLE);
            LiftServo.easeTo(tTargetAngle);
            Serial.print("LiftServo: micros=");
            Serial.print(LiftServo.getEndMicrosecondsOrUnits());
            Serial.print(" degree=");
            Serial.println(tTargetAngle);
            sManualActionHappened = true;
        }
        int tClaw = analogRead(CLAW_INPUT_PIN);
        if (abs(sLastClaw - tClaw) > ANALOG_HYSTERESIS) {
            sLastClaw = tClaw;
            tTargetAngle = map(tClaw, 0, 1023, 0, CLAW_MAX_ANGLE);
            ClawServo.easeTo(tTargetAngle);
            Serial.print("ClawServo: micros=");
            Serial.print(ClawServo.getEndMicrosecondsOrUnits());
            Serial.print(" degree=");
            Serial.println(tTargetAngle);
            sManualActionHappened = true;
        }
        // reset manual action after the first move to manual start position
        if (!isInitialized) {
            isInitialized = true;
            sManualActionHappened = false;
        }
        if (!sManualActionHappened && (millis() > 10000)) {
            sMode = MODE_AUTO;
            Serial.println("Switch to auto mode");
        }
    }
}  //loop

void doDelay() {
    int tDelayValue = analogRead(HORIZONTAL_INPUT_PIN);
    delayAndCheckInput(2 * tDelayValue);
}

/*
 * Instant Command are commands that can be executed at each time in movement.
 * return true if instant command found.
 */
bool checkAndCallInstantCommands(uint8_t aIRCode) {
// search IR code and call associated function
    for (uint8_t i = 0; i < sizeof(IRMW10MappingInstantCommands) / sizeof(struct IRToCommandMapping); ++i) {
        if (aIRCode == IRMW10MappingInstantCommands[i].IRCode) {

            Serial.print(F("Calling "));
            Serial.print(reinterpret_cast<const __FlashStringHelper *>(IRMW10MappingInstantCommands[i].CommandString));
            Serial.println(':');
            /*
             * Call the Quadruped instant function specified in IR mapping
             */
            IRMW10MappingInstantCommands[i].CommandToCall();
            /*
             * Print info
             */
            Serial.print(F(" speed="));
            Serial.print(sServoSpeed);
            Serial.println(F(" degree per second"));

            return true;
        }
    }
    return false;
}

/*
 * Wait for next IR command
 */
uint8_t getIRCommand(bool doWait) {
    static uint8_t sLastIRValue = 0;
    static uint16_t sReferenceAddress = 0; // store first received address here for better IR-receive error handling

    uint8_t tIRValue = COMMAND_EMPTY;

    do {
        if (IRLremote.available()) {
            // Get the new data from the remote
            auto tIRData = IRLremote.read();

            Serial.print(F("A=0x"));
            Serial.print(tIRData.address, HEX);
            Serial.print(F(" C=0x"));
            Serial.println(tIRData.command, HEX);
            tIRValue = tIRData.command;
            if (sReferenceAddress == 0) {
                // store reference address for error detection
                sReferenceAddress = tIRData.address;
            }
            if (sReferenceAddress == tIRData.address) {
                // new code for right address
                sLastIRValue = tIRValue;
                break;
            } else if (tIRData.address == IR_NEC_REPEAT_ADDRESS && tIRData.command == IR_NEC_REPEAT_CODE && sLastIRValue != 0) {
                // received repeat code
                tIRValue = sLastIRValue;
                break;
            } else {
                // unknown code - maybe here, because other interrupts interfere with the IR Interrupt
                // Disable repeat in order not to repeat the wrong command
                sLastIRValue = 0;
            }
        }
    } while (doWait);

    return tIRValue;
}

/*
 * Returns true if stop received
 */
bool checkIRInput() {
    uint8_t tIRCode = getIRCommand(false);
    if (tIRCode == COMMAND_EMPTY) {
        return false;
    } else if (tIRCode == sCurrentCommand) {
        return false;
    } else if (checkAndCallInstantCommands(tIRCode)) {
        return false;
    } else if (tIRCode == COMMAND_STOP) {
        return true;
    } else {
        sNextCommand = tIRCode;
        return true; // return to loop
    }
    return false;
}

void printCommandString(uint8_t aIRCode) {
    Serial.print(F("IRCommand="));
    for (uint8_t i = 0; i < sizeof(IRMW10Mapping) / sizeof(struct IRToCommandMapping); ++i) {
        if (aIRCode == IRMW10Mapping[i].IRCode) {
            Serial.println(reinterpret_cast<const __FlashStringHelper *>(IRMW10Mapping[i].CommandString));
            return;
        }
    }
    Serial.println(reinterpret_cast<const __FlashStringHelper *>(unknown));
}

/*
 * Returns true if stop received
 */
bool delayAndCheckInput(uint16_t aDelayMillis) {
    uint32_t tStartMillis = millis();
    do {
        if (checkIRInput()) {
            return true;
        }
    } while (millis() - tStartMillis > aDelayMillis);
    return false;
}

bool moveOneServoAndCheckInput(uint8_t aServoIndex, uint8_t aDegree, uint16_t aDegreesPerSecond) {
    sServoArray[aServoIndex]->startEaseTo(aDegree, aDegreesPerSecond, false);
    do {
        if (checkIRInput()) {
            return true;
        }
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!sServoArray[aServoIndex]->update());
    return false;
}

bool updateCheckInputAndWaitForAllServosToStop() {
    do {
        if (checkIRInput()) {
            return true;
        }
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!updateAllServos());
    return false;
}

bool synchronizeMoveAllServosAndCheckInputAndWait() {
    setEaseToForAllServos();
    synchronizeAllServosAndStartInterrupt(false);
    return updateCheckInputAndWaitForAllServosToStop();
}

/*************************
 * Instant Commands
 *************************/
/*
 * Decrease moving speed by 25%
 */
bool doIncreaseSpeed() {
    sServoSpeed += sServoSpeed / 4;
    if (sServoSpeed > 0xBF) {
        sServoSpeed = 0xBF;
    }
    setSpeedForAllServos(sServoSpeed);
    return false;
}

/*
 * Increase moving speed by 25%
 */
bool doDecreaseSpeed() {
    if (sServoSpeed > 2) {
        sServoSpeed -= sServoSpeed / 4;
        if (sServoSpeed < 4) {
            sServoSpeed = 4;
        }
    }
    setSpeedForAllServos(sServoSpeed);
    return false;
}

/******************************************
 * The Commands to execute
 ******************************************/

bool doFolded() {
    return setAllServos(4, 90, 0, 100, 70);
}

bool doCenter() {
    return setAllServos(4, PIVOT_START_ANGLE, HORIZONTAL_START_ANGLE, LIFT_START_ANGLE, CLAW_START_ANGLE);
}

bool doGoBack() {
    if (sHorizontalServoAngle > 2) {
        sHorizontalServoAngle -= 2;
        HorizontalServo.easeTo(sHorizontalServoAngle);
    }
    return false;
}

bool doGoForward() {
    if (sHorizontalServoAngle < 178) {
        sHorizontalServoAngle += 2;
        HorizontalServo.easeTo(sHorizontalServoAngle);
    }
    return false;
}

bool doTurnRight() {
    if (sBodyPivotAngle > 2) {
        sBodyPivotAngle -= 2;
        BasePivotServo.easeTo(sBodyPivotAngle);
    }
    return false;
}

bool doTurnLeft() {
    if (sBodyPivotAngle <= 178) {
        sBodyPivotAngle += 2;
        BasePivotServo.easeTo(sBodyPivotAngle);
    }
    return false;
}

bool doLiftUp() {
    if (sLiftServoAngle <= LIFT_MAX_ANGLE - 2) {
        sLiftServoAngle += 2;
        LiftServo.easeTo(sLiftServoAngle);
    }
    return false;
}

bool doLiftDown() {
    if (sLiftServoAngle > 2) {
        sLiftServoAngle -= 2;
        LiftServo.easeTo(sLiftServoAngle);
    }
    return false;
}

bool doOpenClaw() {
    if (sClawServoAngle > 2) {
        sClawServoAngle -= 2;
        ClawServo.easeTo(sClawServoAngle);
    }
    return false;
}

bool doCloseClaw() {
    if (sClawServoAngle <= (CLAW_MAX_ANGLE - 2)) {
        sClawServoAngle += 2;
        ClawServo.easeTo(sClawServoAngle);
    }
    return false;
}

bool doSwitchToManual() {
    sMode = MODE_MANUAL;
    return false;
}

void setEasingTypeToLinear() {
    for (uint8_t tServoIndex = 0; tServoIndex < NUMBER_OF_SERVOS; ++tServoIndex) {
        sServoArray[tServoIndex]->setEasingType(EASE_LINEAR);
    }
}

/*
 *  aBasePivot,  aHorizontal,  aLift,  aClaw
 */
bool setAllServos(uint8_t aNumberOfValues, ...) {
    va_list aDegreeValues;
    va_start(aDegreeValues, aNumberOfValues);
    setDegreeForAllServos(aNumberOfValues, &aDegreeValues);
    va_end(aDegreeValues);
    return synchronizeMoveAllServosAndCheckInputAndWait();
}

/*******************************************
 * Trimming stuff
 ******************************************/
