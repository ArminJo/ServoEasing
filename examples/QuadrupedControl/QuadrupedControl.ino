/*
 * QuadrupedControl.cpp
 *
 * Program for controlling a mePed Robot V2 with 8 servos using an IR Remote
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

#include "QuadrupedIRConfiguration.h" // must be before IRLremote.h if we use not pin 2 or 3
#include <IRLremote.h>      // include IR Remote library

#include "QuadrupedServoConfiguration.h"
#include "ServoEasing.h"    // include servo library

#define VERSION_EXAMPLE "1.1"
// 1.1 mirror computation at transformAndSetPivotServos and transformOneServoIndex

// Define 8 servos in exact this order!
ServoEasing frontLeftPivotServo;    // 0 - Front Left Pivot Servo
ServoEasing frontLeftLiftServo;     // 1 - Front Left Lift Servo
ServoEasing backLeftPivotServo;     // 2 - Back Left Pivot Servo
ServoEasing backLeftLiftServo;      // 3 - Back Left Lift Servo
ServoEasing backRightPivotServo;    // 4 - Back Right Pivot Servo
ServoEasing backRightLiftServo;     // 5 - Back Right Lift Servo
ServoEasing frontRightPivotServo;   // 6 - Front Right Pivot Servo
ServoEasing frontRightLiftServo;    // 7 - Front Right Lift Servo

// Arrays of trim angles stored in EEPROM
EEMEM int8_t sServoTrimAnglesEEPROM[NUMBER_OF_SERVOS]; // The one which resides in EEPROM and IR read out at startup - filled by eepromWriteServoTrim
int8_t sServoTrimAngles[NUMBER_OF_SERVOS]; // RAM copy for easy setting trim angles by remote, filled by eepromReadServoTrim

/*
 * Global control parameters
 */
uint8_t sMovingDirection = MOVE_DIRECTION_FORWARD;
uint16_t sServoSpeed = 60;      // in degree/second
uint8_t sBodyHeightAngle = 60;  // From LIFT_MIN_ANGLE to LIFT_MAX_ANGLE !!! The bigger the angle, the lower the body !!!

uint8_t sCurrentCommand; // to decide if we must change movement
bool sJustExecutingCommand;
uint8_t sNextCommand = COMMAND_EMPTY;    // if != 0 do not wait for IR just take this command as next

#if (IR_CONTROL_CODING == 'P')
CPanasonic IRLremote;
#else
CNec IRLremote;
#endif

/*
 * Function declarations
 */
void printTrimAngles();
void resetServosTo90Degree();
void eepromReadAndSetServoTrim();
void eepromWriteServoTrim();
uint8_t getIRCommand(bool doWait = true);

bool synchronizeMoveAllServosAndCheckInputAndWait();
bool moveOneServoAndCheckInput(uint8_t aServoIndex, uint8_t aDegree, uint16_t aDegreesPerSecond);
bool updateCheckInputAndWaitForAllServosToStop();
bool delayAndCheckInput(uint16_t aDelayMillis);

void setEasingTypeToLinear();
void setEasingTypeForMoving();

bool centerServos();
void setLiftServosToBodyHeight(bool aDoMove);
bool setAllServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aFLL, uint8_t aBLL, uint8_t aBRL, uint8_t aFRL);
bool setPivotServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP);
bool setLiftServos(uint8_t aFLL, uint8_t aBLL, uint8_t aBRL, uint8_t aFRL);

bool transformAndSetAllServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aFLL, uint8_t aBLL, uint8_t aBRL,
        uint8_t aFRL, uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false, bool aDoMove = true);
bool transformAndSetPivotServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aDirection = MOVE_DIRECTION_FORWARD,
        bool doMirror = false, bool aDoMove = true);
uint8_t transformOneServoIndex(uint8_t aServoIndexToTransform, uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false);

bool basicHalfCreep(uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false);
bool basicTwist(uint8_t aTwistAngle, bool aTurnLeft = false);
bool basicQuarterTurn(uint8_t aMoveLegIndex, bool aTurnLeft = false);
bool turn();
bool creep();

void checkIfBodyHeightHasChanged();

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
    frontLeftPivotServo.attach(5);
    frontLeftLiftServo.attach(6);
    backLeftPivotServo.attach(7);
    backLeftLiftServo.attach(8);
    // Invert direction for lift servos.
    backLeftLiftServo.setReverseOperation(true);
    backRightPivotServo.attach(9);
    backRightLiftServo.attach(10);
    frontRightPivotServo.attach(11);
    frontRightLiftServo.attach(12);
    frontRightLiftServo.setReverseOperation(true);

    // set servo to 90 degree without trim and wait
    resetServosTo90Degree();
    delay(2000);

    setSpeedForAllServos(sServoSpeed);

    eepromReadAndSetServoTrim();
    // set servo to 90 degree wit trim and wait
    resetServosTo90Degree();
    delay(2000);

    centerServos();

    // Start reading the remote. PinInterrupt or PinChangeInterrupt* will automatically be selected
    if (!IRLremote.begin(IR_RECEIVER_PIN)) {
        Serial.println(F("You did not choose a valid pin"));
    }
} //setup

void loop() {
    uint8_t tIRCode;
    if (sNextCommand == COMMAND_EMPTY) {
        tIRCode = getIRCommand();
    } else {
        tIRCode = sNextCommand;
        sNextCommand = COMMAND_EMPTY;
    }

    setEasingTypeToLinear();

// search IR code and call associated function

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
    delay(50); // Pause for 50ms before executing next movement
}  //loop

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
            Serial.print(F("Height="));
            Serial.print(sBodyHeightAngle);
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
bool doSetDirectionForward() {
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    return false;
}

bool doSetDirectionBack() {
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
    return false;
}

bool doSetDirectionLeft() {
    sMovingDirection = MOVE_DIRECTION_LEFT;
    return false;
}

bool doSetDirectionRight() {
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    return false;
}

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

/*
 * !!! The angle is inverse to the effective height !!!
 * Take two degrees to move faster
 */
bool doIncreaseHeight() {
    if (sBodyHeightAngle > LIFT_MIN_ANGLE) {
        sBodyHeightAngle -= 2;
        setLiftServosToBodyHeight(!sJustExecutingCommand);
    }
    return false;
}

bool doDecreaseHeight() {
    if (sBodyHeightAngle < LIFT_MAX_ANGLE) {
        sBodyHeightAngle += 2;
        setLiftServosToBodyHeight(!sJustExecutingCommand);
    }
    return false;
}

/******************************************
 * The Commands to execute
 ******************************************/
/*
 * Center, lean left and right lean all 4 directions and twist. Ends with a wave.
 */
bool doDance() {
    Serial.print(F("Dance. Speed="));
    Serial.println(sServoSpeed);

    if (centerServos()) {
        return true;
    }
    /*
     * Move down and up and back to vurrent height
     */
    if (setLiftServos(LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE)) {
        return true;
    }
    if (setLiftServos(LIFT_MIN_ANGLE, LIFT_MIN_ANGLE, LIFT_MIN_ANGLE, LIFT_MIN_ANGLE)) {
        return true;
    }
    if (setLiftServos(sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle)) {
        return true;
    }

    for (int i = 0; i < 1; ++i) {
        if (doLeanLeft()) {
            return true;
        }
        if (doLeanRight()) {
            return true;
        }
    }
    for (int i = 0; i < 3; ++i) {
        if (doLeanLeft()) {
            return true;
        }
        // lean back
        if (setLiftServos(LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MIN_ANGLE)) {
            return true;
        }
        uint8_t tTwistAngle = random(15, 40);
        if (basicTwist(tTwistAngle, true)) {
            return true;
        }
        if (basicTwist(tTwistAngle, false)) {
            return true;
        }

        if (doLeanRight()) {
            return true;
        }
        // lean front
        if (setLiftServos(LIFT_MAX_ANGLE, LIFT_MIN_ANGLE, LIFT_MIN_ANGLE, LIFT_MAX_ANGLE)) {
            return true;
        }
    }

    if (doWave()) {
        return true;
    }
    return centerServos();
}

bool doWave() {
    Serial.print(F("Wave 3 times with right leg. Speed="));
    Serial.println(sServoSpeed);

    if (setAllServos(80, 90, 100, 90, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle)) {
        return true;
    }

    if (setLiftServos(LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE)) {
        return true;
    }

    if (delayAndCheckInput(1000)) {
        return true;
    }
    sServoArray[FRONT_RIGHT_PIVOT]->setEasingType(EASE_QUADRATIC_IN_OUT);

    for (uint8_t i = 0; i < 3; ++i) {
        if (moveOneServoAndCheckInput(FRONT_RIGHT_PIVOT, 135, sServoSpeed * 2)) {
            return true;
        }
        if (moveOneServoAndCheckInput(FRONT_RIGHT_PIVOT, 45, sServoSpeed * 2)) {
            return true;
        }
    }
    if (delayAndCheckInput(1000)) {
        return true;
    }
    return centerServos();

}

bool doCenterServos() {
    return centerServos();
}

bool centerServos() {
    Serial.print(F("Center. Speed="));
    Serial.println(sServoSpeed);
    return setAllServos(90, 90, 90, 90, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle);
}

bool doBow() {
    Serial.print(F("Bow. Speed="));
    Serial.println(sServoSpeed);

    if (centerServos()) {
        return true;
    }
    if (delayAndCheckInput(300)) {
        return true;
    }
    // Lift front legs
    sServoArray[FRONT_LEFT_LIFT]->setEaseTo(LIFT_MIN_ANGLE, sServoSpeed);
    sServoArray[FRONT_RIGHT_LIFT]->startEaseToD(LIFT_MIN_ANGLE, sServoArray[FRONT_LEFT_LIFT]->mMillisForCompleteMove);
    if (updateCheckInputAndWaitForAllServosToStop()) {
        return true;
    }

    if (delayAndCheckInput(300)) {
        return true;
    }

    if (centerServos()) {
        return true;
    }
    return false;
}

bool doLeanLeft() {
    Serial.print(F("Lean left. Speed="));
    Serial.println(sServoSpeed);
    return setLiftServos(LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MIN_ANGLE, LIFT_MIN_ANGLE);
}

bool doLeanRight() {
    Serial.print(F("Lean right. Speed="));
    Serial.println(sServoSpeed);
    return setLiftServos(LIFT_MIN_ANGLE, LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE);
}

void setLiftServosToBodyHeight(bool aDoMove) {
    if (aDoMove) {
        // Set values direct, since we have only a change of 2 degree
        for (uint8_t i = LIFT_SERVO_OFFSET; i < NUMBER_OF_SERVOS; i += SERVOS_PER_LEG) {
            sServoArray[i]->write(sBodyHeightAngle);
        }
    }
}

/*
 * Signals which leg is to be calibrated
 */
void signalLeg(uint8_t aPivotServoIndex) {
    sServoArray[aPivotServoIndex + LIFT_SERVO_OFFSET]->easeTo(LIFT_MAX_ANGLE, 60);
    sServoArray[aPivotServoIndex]->easeTo(90, 60);
    sServoArray[aPivotServoIndex + LIFT_SERVO_OFFSET]->easeTo(90, 60);
}

/*
 * Changes the servo calibration values in EEPROM.
 * Starts with front left i.e. sServoArray[0,1] and switches to the next leg with the COMMAND_ENTER
 */
bool doCalibration() {
    uint8_t tPivotServoIndex = 0; // start with front left i.e. sServoArray[0]
    bool tGotExitCommand = false;
    resetServosTo90Degree();
    delay(500);
    signalLeg(tPivotServoIndex);
    Serial.println(F("Entered calibration. Use the forward/backward right/left buttons to set the servo position to 90 degree."));
    Serial.println(F("Use enter/OK button to go to next leg. Values are stored at receiving a different button or after 4th leg."));

    while (!tGotExitCommand) {
        unsigned long tIRCode = getIRCommand();
        printCommandString(tIRCode);

        switch (tIRCode) {
        case COMMAND_RIGHT:
            sServoTrimAngles[tPivotServoIndex]++;
            sServoArray[tPivotServoIndex]->setTrim(sServoTrimAngles[tPivotServoIndex]);
            break;
        case COMMAND_LEFT:
            sServoTrimAngles[tPivotServoIndex]--;
            sServoArray[tPivotServoIndex]->setTrim(sServoTrimAngles[tPivotServoIndex]);
            break;
        case COMMAND_FORWARD:
            sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]++;
            sServoArray[tPivotServoIndex + LIFT_SERVO_OFFSET]->setTrim(sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]);
            break;
        case COMMAND_BACKWARD:
            sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]--;
            sServoArray[tPivotServoIndex + LIFT_SERVO_OFFSET]->setTrim(sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]);
            break;
        case COMMAND_ENTER:
            // show 135 and 45 degree positions
            sServoArray[tPivotServoIndex]->easeTo(135, 100);
            delay(2000);
            sServoArray[tPivotServoIndex]->easeTo(45, 100);
            delay(2000);
            sServoArray[tPivotServoIndex]->easeTo(90, 100);
            tPivotServoIndex += SERVOS_PER_LEG;
            eepromWriteServoTrim();
            if (tPivotServoIndex >= NUMBER_OF_SERVOS) {
                tGotExitCommand = true;
            } else {
                signalLeg(tPivotServoIndex);
            }
            // remove a repeat command
            getIRCommand(false);
            break;
        case COMMAND_CALIBRATE:
            // repeated command here
            break;
        default:
            eepromWriteServoTrim();
            tGotExitCommand = true;
            break;
        }
        Serial.print(F("ServoTrimAngles["));
        Serial.print(tPivotServoIndex);
        Serial.print(F("]="));
        Serial.print(sServoTrimAngles[tPivotServoIndex]);
        Serial.print(F(" ["));
        Serial.print(tPivotServoIndex + LIFT_SERVO_OFFSET);
        Serial.print(F("]="));
        Serial.println(sServoTrimAngles[tPivotServoIndex + LIFT_SERVO_OFFSET]);
        sServoArray[tPivotServoIndex]->print(&Serial);
        sServoArray[tPivotServoIndex + LIFT_SERVO_OFFSET]->print(&Serial);
        delay(200);
    }
    return false;
}

bool doTwist() {
    Serial.print(F("Twist. Speed="));
    Serial.println(sServoSpeed);
    if (basicTwist(30, true)) {
        return true;
    }
    if (basicTwist(30, false)) {
        return true;
    }
    return false;
}

/*
 * Gait variations
 * 1. Creep: Move one leg forward and down, then move body with all 4 legs down, then move diagonal leg.
 * 2. Trot: Move 2 diagonal legs up and forward
 */
bool doTrot() {
//there is no basicTrot, since it is too simple
    Serial.println(F("Trot. Speed="));
    Serial.println(sServoSpeed);
    setEasingTypeForMoving();
    uint8_t tCurrentDirection = sMovingDirection;
    while (true) {
        uint8_t LiftMaxAngle = sBodyHeightAngle + ((LIFT_MAX_ANGLE - sBodyHeightAngle) / 2);
        /*
         * first move right front and left back leg up and forward
         */
        if (transformAndSetAllServos(TROT_BASE_ANGLE_FL_BR + TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR - TROT_MOVE_ANGLE,
        TROT_BASE_ANGLE_FL_BR - TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR + TROT_MOVE_ANGLE, sBodyHeightAngle, LiftMaxAngle,
                sBodyHeightAngle, LiftMaxAngle, tCurrentDirection)) {
            return true;
        }
        checkIfBodyHeightHasChanged();

        // and the the other legs
        if (transformAndSetAllServos(TROT_BASE_ANGLE_FL_BR - TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR + TROT_MOVE_ANGLE,
        TROT_BASE_ANGLE_FL_BR + TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR - TROT_MOVE_ANGLE, LiftMaxAngle, sBodyHeightAngle,
                LiftMaxAngle, sBodyHeightAngle, tCurrentDirection)) {
            return true;
        }
        checkIfBodyHeightHasChanged();

        if (sMovingDirection != tCurrentDirection) {
            tCurrentDirection = sMovingDirection;
        }
    }
    return false;
}

bool basicTwist(uint8_t aTwistAngle, bool aTurnLeft) {
    Serial.print(F("Twist angle="));
    Serial.print(aTwistAngle);
    Serial.print(F(" turn left="));
    Serial.println(aTurnLeft);
    int8_t tTwistAngle;
    aTurnLeft ? tTwistAngle = -aTwistAngle : tTwistAngle = aTwistAngle;

    if (setPivotServos(90 + tTwistAngle, 90 + tTwistAngle, 90 + tTwistAngle, 90 + tTwistAngle)) {
        return true;
    }
    return false;
}

bool doTurnRight() {
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    return turn();
}

bool doTurnLeft() {
    sMovingDirection = MOVE_DIRECTION_LEFT;
    return turn();
}

/*
 * Must reverse direction of legs to move otherwise the COG is not supported by the legs
 */
bool turn() {
    centerServos();
    setEasingTypeForMoving();
    uint8_t tLegIndex = 0;

    while (true) {
        if (basicQuarterTurn(tLegIndex, sMovingDirection == MOVE_DIRECTION_LEFT)) {
            return true;
        }
        // reverse direction if turn right
        sMovingDirection == MOVE_DIRECTION_LEFT ? tLegIndex++ : tLegIndex--;
        tLegIndex = tLegIndex % NUMBER_OF_LEGS;
    }
    return false;
}

bool basicQuarterTurn(uint8_t aMoveLegIndex, bool aTurnLeft) {
    Serial.print(F("Turn leg="));
    Serial.print(aMoveLegIndex);
    Serial.print(F(" turn left="));
    Serial.println(aTurnLeft);
    int8_t tServoIndex = aMoveLegIndex * SERVOS_PER_LEG;
    int8_t tMoveAngle;
    int8_t tTurnAngle;

// Move one leg forward in turn direction
    if (aTurnLeft) {
        tMoveAngle = TURN_MOVE_ANGLE;
        tTurnAngle = -TURN_BODY_ANGLE;
    } else {
        tMoveAngle = -TURN_MOVE_ANGLE;
        tTurnAngle = TURN_BODY_ANGLE;
    }
    sServoNextPositionArray[tServoIndex] = 90 + tMoveAngle;
    sServoNextPositionArray[tServoIndex + LIFT_SERVO_OFFSET] = LIFT_MAX_ANGLE;
    tServoIndex += SERVOS_PER_LEG;

    for (uint8_t i = 0; i < NUMBER_OF_LEGS - 1; ++i) {
        tServoIndex %= NUMBER_OF_SERVOS;
        sServoNextPositionArray[tServoIndex] = sServoNextPositionArray[tServoIndex] + tTurnAngle;
        sServoNextPositionArray[tServoIndex + LIFT_SERVO_OFFSET] = sBodyHeightAngle;
        tServoIndex += SERVOS_PER_LEG;
    }
//    printArrayPositions(&Serial);
    if (synchronizeMoveAllServosAndCheckInputAndWait()) {
        return true;
    }
    return false;
}

/*
 * Set servo positions and speeds needed to creep forward one step
 * Start with move to Y position with right legs together
 */
bool doCreepForward() {
    Serial.print(F("doCreepForward ServoSpeed="));
    Serial.println(sServoSpeed);
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    return creep();
}

/*
 * Start from same position as forward
 */
bool doCreepBack() {
    Serial.print(F("doCreepBack ServoSpeed="));
    Serial.println(sServoSpeed);
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
    return creep();
}

/*
 * Y position with right legs closed and left legs open
 */
bool goToYPosition(uint8_t aDirection) {
    Serial.print(F("goToYPosition aDirection="));
    Serial.println(aDirection);
    transformAndSetPivotServos(180 - Y_POSITION_OPEN_ANGLE, Y_POSITION_OPEN_ANGLE, (180 - Y_POSITION_CLOSE_ANGLE),
    Y_POSITION_CLOSE_ANGLE, aDirection, false, false);
    return setLiftServos(sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle);
}

bool creep() {
    goToYPosition(sMovingDirection);
    setEasingTypeForMoving();
    uint8_t tCurrentDirection = sMovingDirection;

    while (true) {
        if (basicHalfCreep(tCurrentDirection, false)) {
            return true;
        }
        // now mirror movement
        if (basicHalfCreep(tCurrentDirection, true)) {
            return true;
        }
        if (sMovingDirection != tCurrentDirection) {
            tCurrentDirection = sMovingDirection;
        }
    }
    return false;
}

void checkIfBodyHeightHasChanged() {
    static uint8_t tCurrentBodyHeightAngle = 0;
    // init static variable manually
    if (tCurrentBodyHeightAngle == 0) {
        tCurrentBodyHeightAngle = sBodyHeightAngle;
    }

    if (sBodyHeightAngle != tCurrentBodyHeightAngle) {
        setLiftServosToBodyHeight(true);
        tCurrentBodyHeightAngle = sBodyHeightAngle;
    }
}
/*
 * moves one leg forward and down, then moves body, then moves diagonal leg.
 */
bool basicHalfCreep(uint8_t aDirection, bool doMirror) {

    Serial.print(F("BasicHalfCreep Direction="));
    Serial.print(aDirection);
    Serial.print(F(" doMirror="));
    Serial.println(doMirror);
// 1. Move front right leg up, forward and down
    Serial.println(F("Move front leg"));
    if (transformAndSetAllServos(180 - Y_POSITION_OPEN_ANGLE, Y_POSITION_OPEN_ANGLE, 180 - Y_POSITION_CLOSE_ANGLE,
    Y_POSITION_FRONT_ANGLE, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, LIFT_MAX_ANGLE, aDirection, doMirror)) {
        return true;
    }
    // check if body height has changed
    checkIfBodyHeightHasChanged();
// reset lift value
    sServoNextPositionArray[transformOneServoIndex(FRONT_RIGHT_PIVOT) + LIFT_SERVO_OFFSET] = sBodyHeightAngle;

// 2. Move body forward by CREEP_BODY_MOVE_ANGLE
    Serial.println(F("Move body"));
    if (transformAndSetAllServos(180 - Y_POSITION_CLOSE_ANGLE, Y_POSITION_OPEN_ANGLE + CREEP_BODY_MOVE_ANGLE,
            180 - Y_POSITION_OPEN_ANGLE, Y_POSITION_OPEN_ANGLE, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle,
            sBodyHeightAngle, aDirection, doMirror)) {
        return true;
    }
    checkIfBodyHeightHasChanged();

// 3. Move back right leg up, forward and down
    Serial.println(F("Move back leg to close position"));
// Move to Y position with other side legs together
    if (transformAndSetAllServos(180 - Y_POSITION_CLOSE_ANGLE, Y_POSITION_CLOSE_ANGLE, 180 - Y_POSITION_OPEN_ANGLE,
    Y_POSITION_OPEN_ANGLE, sBodyHeightAngle, LIFT_MAX_ANGLE, sBodyHeightAngle, sBodyHeightAngle, aDirection, doMirror)) {
        return true;
    }
    checkIfBodyHeightHasChanged();

// reset lift value
    sServoNextPositionArray[transformOneServoIndex(BACK_LEFT_PIVOT) + LIFT_SERVO_OFFSET] = sBodyHeightAngle;
    return false;
}

void setEasingTypeToLinear() {
    for (uint8_t tServoIndex = 0; tServoIndex < NUMBER_OF_SERVOS; ++tServoIndex) {
        sServoArray[tServoIndex]->setEasingType(EASE_LINEAR);
    }
}

void setEasingTypeForMoving() {
    for (int tServoIndex = 0; tServoIndex < NUMBER_OF_SERVOS; ++tServoIndex) {
        sServoArray[tServoIndex]->setEasingType(EASE_LINEAR);
        tServoIndex++;
        sServoArray[tServoIndex]->setEasingType(EASE_QUADRATIC_BOUNCING);
    }
}

/*
 * Main transformation routines
 *
 * Direction forward changes nothing.
 * Direction backward swaps forward and backward servos / increases index by NUMBER_OF_LEGS/2
 * Direction left increases index by 1 and right by 3.
 * Mirroring swaps left and right (XOR with 0x06) and invert all angles.
 */

uint8_t getMirrorXorMask(uint8_t aDirection) {
    // XOR the index with this value to get the mirrored index
    if (aDirection & MOVE_DIRECTION_SIDE_MASK) {
        return 0x2;
    } else {
        return 0x6;
    }
}

bool transformAndSetAllServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aFLL, uint8_t aBLL, uint8_t aBRL,
        uint8_t aFRL, uint8_t aDirection, bool doMirror, bool aDoMove) {
    uint8_t tIndexToAdd = aDirection * SERVOS_PER_LEG;
    uint8_t tXorToGetMirroredIndex = 0x0;
    // Invert angles for pivot servos
    bool doInvert = false;
    if (doMirror) {
        // XOR the index with this value to get the mirrored index
        tXorToGetMirroredIndex = getMirrorXorMask(aDirection);
        doInvert = true;
    }

    uint8_t tEffectivePivotServoIndex;
    tEffectivePivotServoIndex = ((FRONT_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFLP = 180 - aFLP;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aFLP;
    sServoNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aFLL;

    tEffectivePivotServoIndex = ((BACK_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBLP = 180 - aBLP;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aBLP;
    sServoNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aBLL;

    tEffectivePivotServoIndex = ((BACK_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBRP = 180 - aBRP;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aBRP;
    sServoNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aBRL;

    tEffectivePivotServoIndex = ((FRONT_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFRP = 180 - aFRP;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aFRP;
    sServoNextPositionArray[tEffectivePivotServoIndex + LIFT_SERVO_OFFSET] = aFRL;

    if (aDoMove) {
        return synchronizeMoveAllServosAndCheckInputAndWait();
    }
    return false;
}

/*
 * A subset of the functionality of transformAndSetAllServos() -> less arguments needed :-)
 */
bool transformAndSetPivotServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aDirection, bool doMirror,
        bool aDoMove) {
    uint8_t tIndexToAdd = aDirection * SERVOS_PER_LEG;
    uint8_t tXorToGetMirroredIndex = 0x0;
    // Invert angles for pivot servos
    bool doInvert = false;
    if (doMirror) {
        // XOR the index with this value to get the mirrored index
        tXorToGetMirroredIndex = getMirrorXorMask(aDirection);
        doInvert = true;
    }

    uint8_t tEffectivePivotServoIndex;
    tEffectivePivotServoIndex = ((FRONT_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFLP = 180 - aFLP;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aFLP;

    tEffectivePivotServoIndex = ((BACK_LEFT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBLP = 180 - aBLP;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aBLP;

    tEffectivePivotServoIndex = ((BACK_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aBRP = 180 - aBRP;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aBRP;

    tEffectivePivotServoIndex = ((FRONT_RIGHT_PIVOT + tIndexToAdd) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
    if (doInvert) {
        aFRP = 180 - aFRP;
    }
    sServoNextPositionArray[tEffectivePivotServoIndex] = aFRP;

    if (aDoMove) {
        return synchronizeMoveAllServosAndCheckInputAndWait();
    }
    return false;
}

uint8_t transformOneServoIndex(uint8_t aServoIndexToTransform, uint8_t aDirection, bool doMirror) {
    uint8_t tXorToGetMirroredIndex = 0x0;
    if (doMirror) {
        // XOR the index with this value to get the mirrored index
        tXorToGetMirroredIndex = getMirrorXorMask(aDirection);
    }
    return ((aServoIndexToTransform + (aDirection * SERVOS_PER_LEG)) % NUMBER_OF_SERVOS) ^ tXorToGetMirroredIndex;
}

void testTransform() {
// left legs are close together, right legs are in straight right direction
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_FORWARD, false, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_FORWARD, true, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_BACKWARD, false, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_BACKWARD, true, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_LEFT, false, false);
    printArrayPositions(&Serial);
    transformAndSetAllServos(180, 1, 135, 30, 111, 0, 0, 0, MOVE_DIRECTION_LEFT, true, false);
    printArrayPositions(&Serial);
}

bool setPivotServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP) {
    sServoNextPositionArray[FRONT_LEFT_PIVOT] = aFLP;
    sServoNextPositionArray[BACK_LEFT_PIVOT] = aBLP;
    sServoNextPositionArray[BACK_RIGHT_PIVOT] = aBRP;
    sServoNextPositionArray[FRONT_RIGHT_PIVOT] = aFRP;
    return synchronizeMoveAllServosAndCheckInputAndWait();
}

bool setLiftServos(uint8_t aFLL, uint8_t aBLL, uint8_t aBRL, uint8_t aFRL) {
    sServoNextPositionArray[FRONT_LEFT_LIFT] = aFLL;
    sServoNextPositionArray[BACK_LEFT_LIFT] = aBLL;
    sServoNextPositionArray[BACK_RIGHT_LIFT] = aBRL;
    sServoNextPositionArray[FRONT_RIGHT_LIFT] = aFRL;
    return synchronizeMoveAllServosAndCheckInputAndWait();
}

bool setAllServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aFLL, uint8_t aBLL, uint8_t aBRL, uint8_t aFRL) {
    sServoNextPositionArray[FRONT_LEFT_PIVOT] = aFLP;
    sServoNextPositionArray[BACK_LEFT_PIVOT] = aBLP;
    sServoNextPositionArray[BACK_RIGHT_PIVOT] = aBRP;
    sServoNextPositionArray[FRONT_RIGHT_PIVOT] = aFRP;

    sServoNextPositionArray[FRONT_LEFT_LIFT] = aFLL;
    sServoNextPositionArray[BACK_LEFT_LIFT] = aBLL;
    sServoNextPositionArray[BACK_RIGHT_LIFT] = aBRL;
    sServoNextPositionArray[FRONT_RIGHT_LIFT] = aFRL;
    return synchronizeMoveAllServosAndCheckInputAndWait();
}

/*******************************************
 * Trimming stuff
 ******************************************/
void printTrimAngles() {
    for (uint8_t i = 0; i < NUMBER_OF_SERVOS; ++i) {
        Serial.print(F("ServoTrimAngle["));
        Serial.print(i);
        Serial.print(F("]="));
        Serial.println(sServoTrimAngles[i]);
        sServoArray[i]->setTrim(sServoTrimAngles[i]);
    }
}

void resetServosTo90Degree() {
    for (uint8_t i = 0; i < NUMBER_OF_SERVOS; ++i) {
        sServoArray[i]->write(90);
    }
}

/*
 * Copy calibration array from EEPROM to RAM and set uninitialized values to 0
 */
void eepromReadAndSetServoTrim() {
    Serial.println(F("eepromReadAndSetServoTrim()"));
    eeprom_read_block((void*) &sServoTrimAngles, &sServoTrimAnglesEEPROM, NUMBER_OF_SERVOS);
    printTrimAngles();
}

void eepromWriteServoTrim() {
    eeprom_write_block((void*) &sServoTrimAngles, &sServoTrimAnglesEEPROM, NUMBER_OF_SERVOS);
    printTrimAngles();
}

/*
 * Just as an unused example to see the principle of movement
 */
bool basicSimpleHalfCreep(uint8_t aLeftLegIndex, bool aMoveMirrored) {
    Serial.print(F("LeftLegIndex="));
    Serial.print(aLeftLegIndex);
    uint8_t tLeftLegPivotServoIndex;

    if (aMoveMirrored) {
        Serial.print(F(" mirrored=true"));
        // get index of pivot servo of mirrored leg
        tLeftLegPivotServoIndex = ((NUMBER_OF_LEGS - 1) - aLeftLegIndex) * SERVOS_PER_LEG; // 0->6
    } else {
        tLeftLegPivotServoIndex = aLeftLegIndex * SERVOS_PER_LEG;
    }
    Serial.println();
//    printArrayPositions(&Serial);
    uint8_t tEffectiveAngle;

// 1. Move front left leg up, forward and down
    Serial.println(F("Move front leg"));
    if (moveOneServoAndCheckInput(tLeftLegPivotServoIndex + LIFT_SERVO_OFFSET, LIFT_MAX_ANGLE, sServoSpeed)) {
        return true;
    }
// go CREEP_BODY_MOVE_ANGLE ahead of Y_POSITION_OPEN_ANGLE
    aMoveMirrored ?
            tEffectiveAngle = 180 - (Y_POSITION_OPEN_ANGLE - CREEP_BODY_MOVE_ANGLE) :
            tEffectiveAngle = Y_POSITION_OPEN_ANGLE - CREEP_BODY_MOVE_ANGLE;
    if (moveOneServoAndCheckInput(tLeftLegPivotServoIndex, tEffectiveAngle, sServoSpeed)) {
        return true;
    }
    if (moveOneServoAndCheckInput(tLeftLegPivotServoIndex + LIFT_SERVO_OFFSET, sBodyHeightAngle, sServoSpeed)) {
        return true;
    }

// 2. Move body forward
    Serial.println(F("Move body"));
    uint8_t tIndex = tLeftLegPivotServoIndex;
    uint8_t tIndexDelta;
// Front left
    if (aMoveMirrored) {
        sServoNextPositionArray[tIndex] = 180 - Y_POSITION_OPEN_ANGLE;
        tIndexDelta = -SERVOS_PER_LEG;
    } else {
        sServoNextPositionArray[tIndex] = Y_POSITION_OPEN_ANGLE;
        tIndexDelta = SERVOS_PER_LEG;
    }
// Back left
    tIndex = (tIndex + tIndexDelta) % NUMBER_OF_SERVOS;
    if (aMoveMirrored) {
        sServoNextPositionArray[tIndex] = Y_POSITION_OPEN_ANGLE;
    } else {
        sServoNextPositionArray[tIndex] = 180 - Y_POSITION_OPEN_ANGLE;
    }

// Back right
    tIndex = (tIndex + tIndexDelta) % NUMBER_OF_SERVOS;
    if (aMoveMirrored) {
        sServoNextPositionArray[tIndex] = 180 - CREEP_BODY_MOVE_ANGLE;
    } else {
        sServoNextPositionArray[tIndex] = CREEP_BODY_MOVE_ANGLE;
    }

// Front right
    tIndex = (tIndex + tIndexDelta) % NUMBER_OF_SERVOS;
    if (aMoveMirrored) {
        sServoNextPositionArray[tIndex] = Y_POSITION_CLOSE_ANGLE;
    } else {
        sServoNextPositionArray[tIndex] = 180 - Y_POSITION_CLOSE_ANGLE;
    }
//    printArrayPositions(&Serial);
    synchronizeMoveAllServosAndCheckInputAndWait();

// 3. Move back right leg up, forward and down
    Serial.println(F("Move back leg to close position"));
// Move to Y position with right legs together / 120, 60, 180, 0
    uint8_t tDiagonalIndex = (tLeftLegPivotServoIndex + DIAGONAL_SERVO_OFFSET) % NUMBER_OF_SERVOS;
    if (moveOneServoAndCheckInput(tDiagonalIndex + LIFT_SERVO_OFFSET, LIFT_MAX_ANGLE, sServoSpeed)) {
        return true;
    }
    aMoveMirrored ? tEffectiveAngle = 180 - Y_POSITION_CLOSE_ANGLE : tEffectiveAngle = Y_POSITION_CLOSE_ANGLE;
    if (moveOneServoAndCheckInput(tDiagonalIndex, tEffectiveAngle, sServoSpeed)) {
        return true;
    }
    if (moveOneServoAndCheckInput(tDiagonalIndex + LIFT_SERVO_OFFSET, sBodyHeightAngle, sServoSpeed)) {
        return true;
    }
    return false;
}

