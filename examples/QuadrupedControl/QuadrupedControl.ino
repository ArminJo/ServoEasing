/*
 * QuadrupedControl.cpp
 *
 * Program for controlling a mePed Robot V2 with 8 servos using an IR Remote at pin A0
 * Supported IR remote are KEYES (the original mePed remote) and WM10
 * Select the one you have at line 23 in QuadrupedIRConfiguration.h
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

#include "QuadrupedIRConfiguration.h" // must be before IRLremote.h. Specifies the IR input pin at A0
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
uint16_t sServoSpeed = 90;      // in degree/second
uint8_t sBodyHeightAngle = 60;  // From LIFT_MIN_ANGLE to LIFT_MAX_ANGLE !!! The bigger the angle, the lower the body !!!

uint8_t sCurrentCommand; // to decide if we must change movement
bool sJustExecutingCommand;
bool sRequestToStop;
#define RETURN_IF_STOP if (sRequestToStop) return
uint8_t sNextCommand = COMMAND_EMPTY;    // if != 0 do not wait for IR just take this command as next

#define MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE 20000
#define MODE_IR 0
#define MODE_AUTO 1
uint8_t sMode = MODE_AUTO;

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
uint8_t getIRCommand(bool doWait);

void synchronizeMoveAllServosAndCheckInputAndWait();
void moveOneServoAndCheckInput(uint8_t aServoIndex, uint8_t aDegree, uint16_t aDegreesPerSecond);
void updateCheckInputAndWaitForAllServosToStop();
void delayAndCheckInput(uint16_t aDelayMillis);

void setEasingTypeToLinear();
void setEasingTypeForMoving();

void centerServos();
void setLiftServosToBodyHeight(bool aDoMove);
void setAllServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aFLL, uint8_t aBLL, uint8_t aBRL, uint8_t aFRL);
void setPivotServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP);
void setLiftServos(uint8_t aFLL, uint8_t aBLL, uint8_t aBRL, uint8_t aFRL);

void transformAndSetAllServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aFLL, uint8_t aBLL, uint8_t aBRL,
        uint8_t aFRL, uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false, bool aDoMove = true);
void transformAndSetPivotServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aDirection = MOVE_DIRECTION_FORWARD,
        bool doMirror = false, bool aDoMove = true);
uint8_t transformOneServoIndex(uint8_t aServoIndexToTransform, uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false);

void basicHalfCreep(uint8_t aDirection = MOVE_DIRECTION_FORWARD, bool doMirror = false);
void basicTwist(uint8_t aTwistAngle, bool aTurnLeft = false);
void basicQuarterTurn(uint8_t aMoveLegIndex, bool aTurnLeft = false);
void moveTurn(uint8_t aNumberOfTurns = 0);          // 0 -> 256 turns
void moveTrot(uint8_t aNumberOfTrots = 0);      // 0 -> 256 trots
void moveCreep(uint8_t aNumberOfCreeps = 0);    // 0 -> 256 creeps

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
        tIRCode = getIRCommand(false);
    } else {
        tIRCode = sNextCommand;
        sNextCommand = COMMAND_EMPTY;
    }

    setEasingTypeToLinear();
    sRequestToStop = false;

// search IR code and call associated function
    if (tIRCode != COMMAND_EMPTY) {
        bool tIRCodeFound = false;
        for (uint8_t i = 0; i < sizeof(IRMapping) / sizeof(struct IRToCommandMapping); ++i) {
            if (tIRCode == IRMapping[i].IRCode) {
                sCurrentCommand = tIRCode;
                Serial.print(F("Calling "));
                Serial.println(reinterpret_cast<const __FlashStringHelper *>(IRMapping[i].CommandString));
                /*
                 * Call the Quadruped function specified in IR mapping
                 */
                sJustExecutingCommand = true;
                IRMapping[i].CommandToCall();
                sJustExecutingCommand = false;
                tIRCodeFound = true;
                break;
            }
        }
        if (!tIRCodeFound) {
            // must be after IRMapping search
            checkAndCallInstantCommands(tIRCode);
        }
        if (sMode != MODE_IR && tIRCodeFound) {
            sMode = MODE_IR;
            Serial.println("Switch to IR mode");
        }
    }
    if (sMode == MODE_AUTO && (millis() > MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE)) {
        doAutoMove();
        if (sRequestToStop) {
            sMode = MODE_IR;
            Serial.println("Switch to IR mode");
        }
    }

    delay(50); // Pause for 50ms before executing next movement
}  //loop

/*
 * Instant Command are commands that can be executed at each time in movement.
 * return true if instant command found.
 */
bool checkAndCallInstantCommands(uint8_t aIRCode) {
// search IR code and call associated function
    for (uint8_t i = 0; i < sizeof(IRMappingInstantCommands) / sizeof(struct IRToCommandMapping); ++i) {
        if (aIRCode == IRMappingInstantCommands[i].IRCode) {

            Serial.print(F("Calling "));
            Serial.print(reinterpret_cast<const __FlashStringHelper *>(IRMappingInstantCommands[i].CommandString));
            Serial.println(':');
            /*
             * Call the Quadruped instant function specified in IR mapping
             */
            IRMappingInstantCommands[i].CommandToCall();
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
//    static uint16_t sReferenceAddress = 0; // store first received address here for better IR-receive error handling

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
//            if (sReferenceAddress == 0) {
//                // store reference address for error detection
//                sReferenceAddress = tIRData.address;
//            }
            if (tIRData.address == IR_ADDRESS) {
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
void checkIRInput() {
    uint8_t tIRCode = getIRCommand(false);
    if (tIRCode == COMMAND_EMPTY) {
        return;
    } else if (tIRCode == sCurrentCommand) {
        // repeated main (non instant) command
        return;
    } else if (checkAndCallInstantCommands(tIRCode)) {
        return;
    } else {
        sNextCommand = tIRCode;
        sRequestToStop = true; // return to loop
    }
}

void printCommandString(uint8_t aIRCode) {
    Serial.print(F("IRCommand="));
    for (uint8_t i = 0; i < sizeof(IRMapping) / sizeof(struct IRToCommandMapping); ++i) {
        if (aIRCode == IRMapping[i].IRCode) {
            Serial.println(reinterpret_cast<const __FlashStringHelper *>(IRMapping[i].CommandString));
            return;
        }
    }
    Serial.println(reinterpret_cast<const __FlashStringHelper *>(unknown));
}

/*
 * Returns true if stop received
 */
void delayAndCheckInput(uint16_t aDelayMillis) {
    uint32_t tStartMillis = millis();
    do {
        checkIRInput();
        RETURN_IF_STOP;
    }while (millis() - tStartMillis > aDelayMillis);
}

void moveOneServoAndCheckInput(uint8_t aServoIndex, uint8_t aDegree, uint16_t aDegreesPerSecond) {
    sServoArray[aServoIndex]->startEaseTo(aDegree, aDegreesPerSecond, false);
    do {
        checkIRInput();
        RETURN_IF_STOP;
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!sServoArray[aServoIndex]->update());
}

void updateCheckInputAndWaitForAllServosToStop() {
    do {
        checkIRInput();
        RETURN_IF_STOP;
        delay(REFRESH_INTERVAL / 1000); // 20ms - REFRESH_INTERVAL is in Microseconds
    } while (!updateAllServos());
}

void synchronizeMoveAllServosAndCheckInputAndWait() {
    setEaseToForAllServos();
    synchronizeAllServosAndStartInterrupt(false);
    updateCheckInputAndWaitForAllServosToStop();
}

/*************************
 * Instant Commands
 *************************/
void doStop() {
    sRequestToStop = true;
}

void doSetDirectionForward() {
    sMovingDirection = MOVE_DIRECTION_FORWARD;
}

void doSetDirectionBack() {
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
}

void doSetDirectionLeft() {
    sMovingDirection = MOVE_DIRECTION_LEFT;
}

void doSetDirectionRight() {
    sMovingDirection = MOVE_DIRECTION_RIGHT;
}

/*
 * Decrease moving speed by 25%
 */
void doIncreaseSpeed() {
    sServoSpeed += sServoSpeed / 4;
    if (sServoSpeed > 0xBF) {
        sServoSpeed = 0xBF;
    }
    setSpeedForAllServos(sServoSpeed);
}

/*
 * Increase moving speed by 25%
 */
void doDecreaseSpeed() {
    if (sServoSpeed > 2) {
        sServoSpeed -= sServoSpeed / 4;
        if (sServoSpeed < 4) {
            sServoSpeed = 4;
        }
    }
    setSpeedForAllServos(sServoSpeed);
}

/*
 * !!! The angle is inverse to the effective height !!!
 * Take two degrees to move faster
 */
void doIncreaseHeight() {
    if (sBodyHeightAngle > LIFT_MIN_ANGLE) {
        sBodyHeightAngle -= 2;
        setLiftServosToBodyHeight(!sJustExecutingCommand);
    }
}

void doDecreaseHeight() {
    if (sBodyHeightAngle < LIFT_MAX_ANGLE) {
        sBodyHeightAngle += 2;
        setLiftServosToBodyHeight(!sJustExecutingCommand);
    }
}

/******************************************
 * The Commands to execute
 ******************************************/
void doAutoMove() {
    centerServos();
    RETURN_IF_STOP;

    RETURN_IF_STOP;
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    moveCreep(2);
    RETURN_IF_STOP;

    setSpeedForAllServos(200);
    moveCreep(2);
    RETURN_IF_STOP;

//    setSpeedForAllServos(160);
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveCreep(2);
    RETURN_IF_STOP;

    setSpeedForAllServos(160);
    centerServos();
    moveTrot(2);
    RETURN_IF_STOP;

    sMovingDirection = MOVE_DIRECTION_BACKWARD;
    moveTrot(4);
    RETURN_IF_STOP;

    centerServos();
    moveTurn(8);
    RETURN_IF_STOP;

    delayAndCheckInput(2000);
}
/*
 * Center, lean left and right lean all 4 directions and twist. Ends with a wave.
 */
void doDance() {
    Serial.print(F("Dance. Speed="));
    Serial.println(sServoSpeed);

    centerServos();
    RETURN_IF_STOP;
    /*
     * Move down and up and back to current height
     */
    setLiftServos(LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE);
    RETURN_IF_STOP;
    setLiftServos(LIFT_MIN_ANGLE, LIFT_MIN_ANGLE, LIFT_MIN_ANGLE, LIFT_MIN_ANGLE);
    RETURN_IF_STOP;
    setLiftServos(sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle);
    RETURN_IF_STOP;

    for (int i = 0; i < 1; ++i) {
        doLeanLeft();
        RETURN_IF_STOP;
        doLeanRight();
        RETURN_IF_STOP;
    }
    for (int i = 0; i < 3; ++i) {
        doLeanLeft();
        RETURN_IF_STOP;
        // lean back
        setLiftServos(LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MIN_ANGLE);
        RETURN_IF_STOP;
        uint8_t tTwistAngle = random(15, 40);
        basicTwist(tTwistAngle, true);
        RETURN_IF_STOP;
        basicTwist(tTwistAngle, false);
        RETURN_IF_STOP;

        doLeanRight();
        RETURN_IF_STOP;
        // lean front
        setLiftServos(LIFT_MAX_ANGLE, LIFT_MIN_ANGLE, LIFT_MIN_ANGLE, LIFT_MAX_ANGLE);
        RETURN_IF_STOP;
    }

    doWave();
    RETURN_IF_STOP;
    centerServos();
}

void doWave() {
    Serial.print(F("Wave 3 times with right leg. Speed="));
    Serial.println(sServoSpeed);

    setAllServos(80, 90, 100, 90, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle);
    RETURN_IF_STOP;

    setLiftServos(LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE);
    RETURN_IF_STOP;

    delayAndCheckInput(1000);
    RETURN_IF_STOP;

    sServoArray[FRONT_RIGHT_PIVOT]->setEasingType(EASE_QUADRATIC_IN_OUT);

    for (uint8_t i = 0; i < 3; ++i) {
        moveOneServoAndCheckInput(FRONT_RIGHT_PIVOT, 135, sServoSpeed * 2);
        RETURN_IF_STOP;

        moveOneServoAndCheckInput(FRONT_RIGHT_PIVOT, 45, sServoSpeed * 2);
        RETURN_IF_STOP;
    }
    delayAndCheckInput(1000);
    RETURN_IF_STOP;

    centerServos();

}

void doCenterServos() {
    centerServos();
}

void centerServos() {
    Serial.print(F("Center. Speed="));
    Serial.println(sServoSpeed);
    return setAllServos(90, 90, 90, 90, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle);
}

void doBow() {
    Serial.print(F("Bow. Speed="));
    Serial.println(sServoSpeed);

    centerServos();
    RETURN_IF_STOP;

    delayAndCheckInput(300);
    RETURN_IF_STOP;

    // Lift front legs
    sServoArray[FRONT_LEFT_LIFT]->setEaseTo(LIFT_MIN_ANGLE, sServoSpeed);
    sServoArray[FRONT_RIGHT_LIFT]->startEaseToD(LIFT_MIN_ANGLE, sServoArray[FRONT_LEFT_LIFT]->mMillisForCompleteMove);
    updateCheckInputAndWaitForAllServosToStop();
    RETURN_IF_STOP;

    delayAndCheckInput(300);
    RETURN_IF_STOP;

    centerServos();
}

void doLeanLeft() {
    Serial.print(F("Lean left. Speed="));
    Serial.println(sServoSpeed);
    setLiftServos(LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MIN_ANGLE, LIFT_MIN_ANGLE);
}

void doLeanRight() {
    Serial.print(F("Lean right. Speed="));
    Serial.println(sServoSpeed);
    setLiftServos(LIFT_MIN_ANGLE, LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE);
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
void doCalibration() {
    uint8_t tPivotServoIndex = 0; // start with front left i.e. sServoArray[0]
    bool tGotExitCommand = false;
    resetServosTo90Degree();
    delay(500);
    signalLeg(tPivotServoIndex);
    Serial.println(F("Entered calibration. Use the forward/backward right/left buttons to set the servo position to 90 degree."));
    Serial.println(F("Use enter/OK button to go to next leg. Values are stored at receiving a different button or after 4th leg."));

    while (!tGotExitCommand) {
        unsigned long tIRCode = getIRCommand(true);
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
}

void doTwist() {
    Serial.print(F("Twist. Speed="));
    Serial.println(sServoSpeed);
    basicTwist(30, true);
    RETURN_IF_STOP;

    basicTwist(30, false);
}

/*
 * Gait variations
 * 1. Creep: Move one leg forward and down, then move body with all 4 legs down, then move diagonal leg.
 * 2. Trot: Move 2 diagonal legs up and forward
 */

void moveTrot(uint8_t aNumberOfTrots) {
    setEasingTypeForMoving();
    uint8_t tCurrentDirection = sMovingDirection;
    do {
        uint8_t LiftMaxAngle = sBodyHeightAngle + ((LIFT_MAX_ANGLE - sBodyHeightAngle) / 2);
        /*
         * first move right front and left back leg up and forward
         */
        transformAndSetAllServos(TROT_BASE_ANGLE_FL_BR + TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR - TROT_MOVE_ANGLE,
        TROT_BASE_ANGLE_FL_BR - TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR + TROT_MOVE_ANGLE, sBodyHeightAngle, LiftMaxAngle,
                sBodyHeightAngle, LiftMaxAngle, tCurrentDirection);
        RETURN_IF_STOP;

        checkIfBodyHeightHasChanged();

        // and the the other legs
        transformAndSetAllServos(TROT_BASE_ANGLE_FL_BR - TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR + TROT_MOVE_ANGLE,
        TROT_BASE_ANGLE_FL_BR + TROT_MOVE_ANGLE, TROT_BASE_ANGLE_BL_FR - TROT_MOVE_ANGLE, LiftMaxAngle, sBodyHeightAngle,
                LiftMaxAngle, sBodyHeightAngle, tCurrentDirection);
        RETURN_IF_STOP;

        checkIfBodyHeightHasChanged();

        if (sMovingDirection != tCurrentDirection) {
            tCurrentDirection = sMovingDirection;
        }
        aNumberOfTrots--;
    } while (aNumberOfTrots != 0);
}

void doTrot() {
    Serial.println(F("Trot. Speed="));
    Serial.println(sServoSpeed);
    moveTrot();
}

void basicTwist(uint8_t aTwistAngle, bool aTurnLeft) {
    Serial.print(F("Twist angle="));
    Serial.print(aTwistAngle);
    Serial.print(F(" turn left="));
    Serial.println(aTurnLeft);
    int8_t tTwistAngle;
    aTurnLeft ? tTwistAngle = -aTwistAngle : tTwistAngle = aTwistAngle;

    setPivotServos(90 + tTwistAngle, 90 + tTwistAngle, 90 + tTwistAngle, 90 + tTwistAngle);
}

void doTurnRight() {
    sMovingDirection = MOVE_DIRECTION_RIGHT;
    moveTurn();
}

void doTurnLeft() {
    sMovingDirection = MOVE_DIRECTION_LEFT;
    moveTurn();
}

/*
 * Must reverse direction of legs to move otherwise the COG is not supported by the legs
 */
void moveTurn(uint8_t aNumberOfTurns) {
    centerServos();
    setEasingTypeForMoving();
    uint8_t tNextLegIndex = FRONT_LEFT_PIVOT;

    /*
     * Move one leg out of center position, otherwise the COG may be not supported at the first move
     */
    if (sMovingDirection == MOVE_DIRECTION_LEFT) {
        moveOneServoAndCheckInput(FRONT_RIGHT_PIVOT, 90 + TURN_MOVE_ANGLE, sServoSpeed);
        RETURN_IF_STOP;
    }
    else {
        moveOneServoAndCheckInput(BACK_LEFT_PIVOT, 90 - TURN_MOVE_ANGLE, sServoSpeed);
        RETURN_IF_STOP;
    }

    do {
        basicQuarterTurn(tNextLegIndex, sMovingDirection == MOVE_DIRECTION_LEFT);
        RETURN_IF_STOP;
        // reverse direction of NextLegIndex if moveTurn right
        sMovingDirection == MOVE_DIRECTION_LEFT ? tNextLegIndex++ : tNextLegIndex--;
        tNextLegIndex = tNextLegIndex % NUMBER_OF_LEGS;
        aNumberOfTurns--;
    } while (aNumberOfTurns != 0);
}

void basicQuarterTurn(uint8_t aMoveLegIndex, bool aTurnLeft) {
    Serial.print(F("Turn leg="));
    Serial.print(aMoveLegIndex);
    Serial.print(F(" turn left="));
    Serial.println(aTurnLeft);
    int8_t tServoIndex = aMoveLegIndex * SERVOS_PER_LEG;
    int8_t tMoveAngle;
    int8_t tTurnAngle;

    /*
     * Move one leg forward in moveTurn direction
     */
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

    /*
     *
     */
    for (uint8_t i = 0; i < NUMBER_OF_LEGS - 1; ++i) {
        tServoIndex %= NUMBER_OF_SERVOS;
        sServoNextPositionArray[tServoIndex] = sServoNextPositionArray[tServoIndex] + tTurnAngle;
// reset lift values for all other legs
        sServoNextPositionArray[tServoIndex + LIFT_SERVO_OFFSET] = sBodyHeightAngle;
        tServoIndex += SERVOS_PER_LEG;
    }
//    printArrayPositions(&Serial);
    synchronizeMoveAllServosAndCheckInputAndWait();
}

/*
 * Set servo positions and speeds needed to moveCreep forward one step
 * Start with move to Y position with right legs together
 */
void doCreepForward() {
    Serial.print(F("doCreepForward ServoSpeed="));
    Serial.println(sServoSpeed);
    sMovingDirection = MOVE_DIRECTION_FORWARD;
    moveCreep();
}

/*
 * Start from same position as forward
 */
void doCreepBack() {
    Serial.print(F("doCreepBack ServoSpeed="));
    Serial.println(sServoSpeed);
    sMovingDirection = MOVE_DIRECTION_BACKWARD;
    moveCreep();
}

/*
 * Y position with right legs closed and left legs open
 */
void goToYPosition(uint8_t aDirection) {
    Serial.print(F("goToYPosition aDirection="));
    Serial.println(aDirection);
    transformAndSetPivotServos(180 - Y_POSITION_OPEN_ANGLE, Y_POSITION_OPEN_ANGLE, (180 - Y_POSITION_CLOSE_ANGLE),
    Y_POSITION_CLOSE_ANGLE, aDirection, false, false);
    setLiftServos(sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle);
}

/*
 * 0 -> 256 creeps
 */
void moveCreep(uint8_t aNumberOfCreeps) {
    goToYPosition(sMovingDirection);
    setEasingTypeForMoving();
    uint8_t tCurrentDirection = sMovingDirection;

    do {
        basicHalfCreep(tCurrentDirection, false);
        RETURN_IF_STOP;
// now mirror movement
        basicHalfCreep(tCurrentDirection, true);
        RETURN_IF_STOP;
        if (sMovingDirection != tCurrentDirection) {
            tCurrentDirection = sMovingDirection;
        }
        aNumberOfCreeps--;
    } while (aNumberOfCreeps != 0);
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
void basicHalfCreep(uint8_t aDirection, bool doMirror) {

    Serial.print(F("BasicHalfCreep Direction="));
    Serial.print(aDirection);
    Serial.print(F(" doMirror="));
    Serial.println(doMirror);
// 1. Move front right leg up, forward and down
    Serial.println(F("Move front leg"));
    transformAndSetAllServos(180 - Y_POSITION_OPEN_ANGLE, Y_POSITION_OPEN_ANGLE, 180 - Y_POSITION_CLOSE_ANGLE,
    Y_POSITION_FRONT_ANGLE, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle, LIFT_MAX_ANGLE, aDirection, doMirror);
    RETURN_IF_STOP;

// check if body height has changed
    checkIfBodyHeightHasChanged();
// reset lift value
    sServoNextPositionArray[transformOneServoIndex(FRONT_RIGHT_PIVOT) + LIFT_SERVO_OFFSET] = sBodyHeightAngle;

// 2. Move body forward by CREEP_BODY_MOVE_ANGLE
    Serial.println(F("Move body"));
    transformAndSetAllServos(180 - Y_POSITION_CLOSE_ANGLE, Y_POSITION_OPEN_ANGLE + CREEP_BODY_MOVE_ANGLE,
            180 - Y_POSITION_OPEN_ANGLE, Y_POSITION_OPEN_ANGLE, sBodyHeightAngle, sBodyHeightAngle, sBodyHeightAngle,
            sBodyHeightAngle, aDirection, doMirror);
    RETURN_IF_STOP;
    checkIfBodyHeightHasChanged();

// 3. Move back right leg up, forward and down
    Serial.println(F("Move back leg to close position"));
// Move to Y position with other side legs together
    transformAndSetAllServos(180 - Y_POSITION_CLOSE_ANGLE, Y_POSITION_CLOSE_ANGLE, 180 - Y_POSITION_OPEN_ANGLE,
    Y_POSITION_OPEN_ANGLE, sBodyHeightAngle, LIFT_MAX_ANGLE, sBodyHeightAngle, sBodyHeightAngle, aDirection, doMirror);
    RETURN_IF_STOP;

    checkIfBodyHeightHasChanged();

// reset lift value
    sServoNextPositionArray[transformOneServoIndex(BACK_LEFT_PIVOT) + LIFT_SERVO_OFFSET] = sBodyHeightAngle;
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

void transformAndSetAllServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aFLL, uint8_t aBLL, uint8_t aBRL,
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
        synchronizeMoveAllServosAndCheckInputAndWait();
    }
}

/*
 * A subset of the functionality of transformAndSetAllServos() -> less arguments needed :-)
 */
void transformAndSetPivotServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aDirection, bool doMirror,
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
        synchronizeMoveAllServosAndCheckInputAndWait();
    }
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

void setPivotServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP) {
    sServoNextPositionArray[FRONT_LEFT_PIVOT] = aFLP;
    sServoNextPositionArray[BACK_LEFT_PIVOT] = aBLP;
    sServoNextPositionArray[BACK_RIGHT_PIVOT] = aBRP;
    sServoNextPositionArray[FRONT_RIGHT_PIVOT] = aFRP;
    synchronizeMoveAllServosAndCheckInputAndWait();
}

void setLiftServos(uint8_t aFLL, uint8_t aBLL, uint8_t aBRL, uint8_t aFRL) {
    sServoNextPositionArray[FRONT_LEFT_LIFT] = aFLL;
    sServoNextPositionArray[BACK_LEFT_LIFT] = aBLL;
    sServoNextPositionArray[BACK_RIGHT_LIFT] = aBRL;
    sServoNextPositionArray[FRONT_RIGHT_LIFT] = aFRL;
    synchronizeMoveAllServosAndCheckInputAndWait();
}

void setAllServos(uint8_t aFLP, uint8_t aBLP, uint8_t aBRP, uint8_t aFRP, uint8_t aFLL, uint8_t aBLL, uint8_t aBRL, uint8_t aFRL) {
    sServoNextPositionArray[FRONT_LEFT_PIVOT] = aFLP;
    sServoNextPositionArray[BACK_LEFT_PIVOT] = aBLP;
    sServoNextPositionArray[BACK_RIGHT_PIVOT] = aBRP;
    sServoNextPositionArray[FRONT_RIGHT_PIVOT] = aFRP;

    sServoNextPositionArray[FRONT_LEFT_LIFT] = aFLL;
    sServoNextPositionArray[BACK_LEFT_LIFT] = aBLL;
    sServoNextPositionArray[BACK_RIGHT_LIFT] = aBRL;
    sServoNextPositionArray[FRONT_RIGHT_LIFT] = aFRL;
    synchronizeMoveAllServosAndCheckInputAndWait();
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
void basicSimpleHalfCreep(uint8_t aLeftLegIndex, bool aMoveMirrored) {
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
    moveOneServoAndCheckInput(tLeftLegPivotServoIndex + LIFT_SERVO_OFFSET, LIFT_MAX_ANGLE, sServoSpeed);
    RETURN_IF_STOP;

// go CREEP_BODY_MOVE_ANGLE ahead of Y_POSITION_OPEN_ANGLE
    aMoveMirrored ?
            tEffectiveAngle = 180 - (Y_POSITION_OPEN_ANGLE - CREEP_BODY_MOVE_ANGLE) :
            tEffectiveAngle = Y_POSITION_OPEN_ANGLE - CREEP_BODY_MOVE_ANGLE;
    moveOneServoAndCheckInput(tLeftLegPivotServoIndex, tEffectiveAngle, sServoSpeed);
    RETURN_IF_STOP;

    moveOneServoAndCheckInput(tLeftLegPivotServoIndex + LIFT_SERVO_OFFSET, sBodyHeightAngle, sServoSpeed);
    RETURN_IF_STOP;

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
    moveOneServoAndCheckInput(tDiagonalIndex + LIFT_SERVO_OFFSET, LIFT_MAX_ANGLE, sServoSpeed);
    RETURN_IF_STOP;

    aMoveMirrored ? tEffectiveAngle = 180 - Y_POSITION_CLOSE_ANGLE : tEffectiveAngle = Y_POSITION_CLOSE_ANGLE;
    moveOneServoAndCheckInput(tDiagonalIndex, tEffectiveAngle, sServoSpeed);
    RETURN_IF_STOP;

    moveOneServoAndCheckInput(tDiagonalIndex + LIFT_SERVO_OFFSET, sBodyHeightAngle, sServoSpeed);
}

