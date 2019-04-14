/*
 * RobotArmControl.cpp
 *
 * Program for controlling a RobotArm with 4 servos using 4 potentiometers and/or an IR Remote at pin A0
 * Sea also: https://www.instructables.com/id/4-DOF-Mechanical-Arm-Robot-Controlled-by-Arduino
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

#include "RobotArmIRConfiguration.h" // must be before IRLremote.h. Specifies the IR input pin at A0
#include <IRLremote.h>      // include IR Remote library

#include "ServoEasing.h"    // include servo library
#include "ik.h"

//#define TRACE
//#define DEBUG

#define USE_BUTTON_0
#define USE_ATTACH_INTERRUPT // to be compatible with IRLremote
#include "EasyButtonAtInt01.h" // for switching easing modes

#include "ADCUtils.h" // for get getVCCVoltageMillivolt

#define VERSION_EXAMPLE "1.0"

#define ZERO_DEGREE_VALUE_MICROS    544
#define AT_180_DEGREE_VALUE_MICROS 2400

#define PIVOT_INPUT_PIN         A3
#define HORIZONTAL_INPUT_PIN    A1
#define LIFT_INPUT_PIN          A4
#define CLAW_INPUT_PIN          A2

#define LIFT_MAX_ANGLE          150
#define CLAW_MAX_ANGLE          52

#define CLAW_START_ANGLE        CLAW_MAX_ANGLE
#define CLAW_CLOSE_ANGLE        CLAW_MAX_ANGLE
#define CLAW_OPEN_ANGLE         (CLAW_MAX_ANGLE - 30)

// Index into (external) servo array. Order must be the same as of definitions in main.
#define SERVO_BASE_PIVOT 0
#define SERVO_HORIZONTAL 1
#define SERVO_LIFT 2
#define SERVO_CLAW 3
#define NUMBER_OF_SERVOS 4

// Define the 4 servos in exact this order!
ServoEasing BasePivotServo;    // 0 - Front Left Pivot Servo
ServoEasing HorizontalServo;     // 1 - Front Left Lift Servo
ServoEasing LiftServo;     // 2 - Back Left Pivot Servo
ServoEasing ClawServo;      // 3 - Back Left Lift Servo

/*
 * Global control parameters
 */
uint16_t sServoSpeed = 60;      // in degree/second
uint8_t sEasingType = EASE_LINEAR;
bool sInverseKinematicModeActive = true;

uint8_t sBodyPivotAngle = PIVOT_NEUTRAL_ANGLE;
uint8_t sHorizontalServoAngle = HORIZONTAL_NEUTRAL_ANGLE;
uint8_t sLiftServoAngle = LIFT_NEUTRAL_ANGLE;
uint8_t sClawServoAngle = CLAW_START_ANGLE;

uint8_t sCurrentCommand; // to decide if we must change movement
bool sCurrentCommandIsRepeat;
bool sJustExecutingCommand;
bool sRequestToStop;
#define RETURN_IF_STOP if (sRequestToStop) return
uint8_t sNextCommand = COMMAND_EMPTY;    // if != 0 do not wait for IR just take this command as next

#define MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE 10000
#define MODE_IR 0
#define MODE_AUTO 1
#define MODE_MANUAL 3
uint8_t sMode = MODE_MANUAL;

CNec IRLremote;

void changeEasingType(bool aButtonToggleState);
EasyButton Button0AtPin2(true, &changeEasingType);

/*
 * Function declarations
 */
uint8_t getIRCommand(bool doWait);

void synchronizeMoveAllServosAndCheckInputAndWait();
void moveOneServoAndCheckInput(uint8_t aServoIndex, uint8_t aDegree, uint16_t aDegreesPerSecond = sServoSpeed);
void updateCheckInputAndWaitForAllServosToStop();
void delayAndCheckInput(uint16_t aDelayMillis);

void setEasingType(uint8_t aEasingType);

void setAllServos(uint8_t aNumberOfValues, ...);

bool handleManualControl();

float moveInverseKinematicForBase(float aPercentageOfCompletion);
float moveInverseKinematicForHorizontal(float aPercentageOfCompletion);
float moveInverseKinematicForLift(float aPercentageOfCompletion);
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

    /*
     * delay() to avoid uncontrolled servo moving after power on.
     * Then the Arduino is reseted many times which may lead to invalid servo signals.
     */
    delay(200);
    setSpeedForAllServos(sServoSpeed);
    // Attach servos to Arduino Pins and set to start position. Must be done with write()
    BasePivotServo.attach(4);
    BasePivotServo.write(PIVOT_NEUTRAL_ANGLE);
    BasePivotServo.registerUserEaseInFunction(&moveInverseKinematicForBase);
    delay(200);
    HorizontalServo.attach(5);
    HorizontalServo.write(HORIZONTAL_NEUTRAL_ANGLE);
    HorizontalServo.registerUserEaseInFunction(&moveInverseKinematicForHorizontal);
    delay(200);
    LiftServo.attach(7);
    ClawServo.attach(8);
    LiftServo.write(LIFT_NEUTRAL_ANGLE);
    ClawServo.write(CLAW_START_ANGLE);
    LiftServo.registerUserEaseInFunction(&moveInverseKinematicForLift);

    // Start reading the remote. PinInterrupt or PinChangeInterrupt* will automatically be selected
    if (!IRLremote.begin(IR_RECEIVER_PIN)) {
        Serial.println(F("You did not choose a valid pin"));
    }

    Serial.print(F("Value for 0 degree="));
    Serial.print(ZERO_DEGREE_VALUE_MICROS);
    Serial.print(F("us. Value for 180 degree="));
    Serial.print(AT_180_DEGREE_VALUE_MICROS);
    Serial.println(F("us."));

    // Output VCC Voltage
    int tVoltageMillivolts = getVCCVoltageMillivolt();
    Serial.print(F("VCC="));
    Serial.print(tVoltageMillivolts);
    Serial.println(" mV");

} //setup

void loop() {

    uint8_t tIRCode;
    if (sNextCommand == COMMAND_EMPTY) {
        // Get command from remote
        tIRCode = getIRCommand(false);
    } else {
        // Get command from buffer. It was likely sent from remote before.
        tIRCode = sNextCommand;
        sNextCommand = COMMAND_EMPTY;
    }

    // Reset mode to linear for all movements
    if (sInverseKinematicModeActive) {
        setEasingTypeForAllServos(EASE_LINEAR);
    }
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
        delay(20); // Pause before executing next movement
    } else if (sMode == MODE_AUTO) {
        doAutoMove();
        if (sRequestToStop) {
            sMode = MODE_IR;
            Serial.println("Switch to IR mode");
        }
    } else if (sMode == MODE_MANUAL) {
        if (!handleManualControl() && (millis() > MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE)) {
            sMode = MODE_AUTO;
            Serial.println("Switch to auto mode");
        }
    }
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
            Nec_data_t tIRData = IRLremote.read();

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
                sCurrentCommandIsRepeat = false;
                break;
            } else if (tIRData.address == IR_NEC_REPEAT_ADDRESS && tIRData.command == IR_NEC_REPEAT_CODE && sLastIRValue != 0) {
                // received repeat code
                sCurrentCommandIsRepeat = true;
                tIRValue = sLastIRValue;
                break;
            } else {
                // unknown code - maybe here, because other interrupts interfere with the IR Interrupt
                // Disable repeat in order not to repeat the wrong command
                sLastIRValue = COMMAND_EMPTY;
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

/******************************************
 * The Commands to execute
 ******************************************/
struct ArmPosition sStartPosition, sEndPosition, sCurrentPosition, sPositionDelta;

void printPosition(struct ArmPosition * aPositionStruct) {
    Serial.print("LeftRight=");
    Serial.print(aPositionStruct->LeftRight);
    Serial.print("|");
    Serial.print(aPositionStruct->LeftRightDegree);
    Serial.print(" BackFront=");
    Serial.print(aPositionStruct->BackFront);
    Serial.print("|");
    Serial.print(aPositionStruct->BackFrontDegree);
    Serial.print(" DownUp=");
    Serial.print(aPositionStruct->DownUp);
    Serial.print("|");
    Serial.println(aPositionStruct->DownUpDegree);
}
//float sDistance;

float sLastPercentageOfCompletion = 2.0;

// Move smoothly from current position to the new position
bool goToPosition(int aLeftRight, int aBackFront, int aDownUp) {
    sStartPosition = sEndPosition;
#ifdef DEBUG
    Serial.print("Start: ");
    printPosition(&sStartPosition);
#endif
    sEndPosition.LeftRight = aLeftRight;
    sPositionDelta.LeftRight = aLeftRight - sStartPosition.LeftRight;
    sEndPosition.BackFront = aBackFront;
    sPositionDelta.BackFront = aBackFront - sStartPosition.BackFront;
    sEndPosition.DownUp = aDownUp;
    sPositionDelta.DownUp = aDownUp - sStartPosition.DownUp;
//    sDistance = sqrt(
//            (sStartPosition.LeftRight - aLeftRight) * (sStartPosition.LeftRight - aLeftRight)
//                    + (sStartPosition.BackFront - aBackFront) * (sStartPosition.BackFront - aBackFront)
//                    + (sStartPosition.DownUp - aDownUp) * (sStartPosition.DownUp - aDownUp));
    if (!solve(&sEndPosition)) {
        Serial.print("Position cannot be solved: ");
        printPosition(&sEndPosition);
        return false;
    }
#ifdef DEBUG
    Serial.print("End: ");
    printPosition(&sEndPosition);
#endif
    setAllServos(3, sEndPosition.LeftRightDegree, sEndPosition.BackFrontDegree, sEndPosition.DownUpDegree);
    return true;
}

void computeNewCurrentAngles(float aPercentageOfCompletion) {
    sCurrentPosition.LeftRight = sStartPosition.LeftRight + (sPositionDelta.LeftRight * aPercentageOfCompletion);
    sCurrentPosition.BackFront = sStartPosition.BackFront + (sPositionDelta.BackFront * aPercentageOfCompletion);
    sCurrentPosition.DownUp = sStartPosition.DownUp + (sPositionDelta.DownUp * aPercentageOfCompletion);
    solve(&sCurrentPosition);
#ifdef TRACE
    Serial.print("Current: ");
    printPosition(&sCurrentPosition);
#endif
}

float moveInverseKinematicForBase(float aPercentageOfCompletion) {
    if (sLastPercentageOfCompletion != aPercentageOfCompletion) {
        sLastPercentageOfCompletion = aPercentageOfCompletion;
        computeNewCurrentAngles(aPercentageOfCompletion);
    }
    return sCurrentPosition.LeftRightDegree + EASE_FUNCTION_DEGREE_OFFSET;
}

float moveInverseKinematicForHorizontal(float aPercentageOfCompletion) {
    if (sLastPercentageOfCompletion != aPercentageOfCompletion) {
        sLastPercentageOfCompletion = aPercentageOfCompletion;
        computeNewCurrentAngles(aPercentageOfCompletion);
    }
    return sCurrentPosition.BackFrontDegree + EASE_FUNCTION_DEGREE_OFFSET;
}

float moveInverseKinematicForLift(float aPercentageOfCompletion) {
    if (sLastPercentageOfCompletion != aPercentageOfCompletion) {
        sLastPercentageOfCompletion = aPercentageOfCompletion;
        computeNewCurrentAngles(aPercentageOfCompletion);
    }
    return sCurrentPosition.DownUpDegree + EASE_FUNCTION_DEGREE_OFFSET;
}

void testInverseKinematic() {
    // init start position for first move
    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = 148;
    sEndPosition.DownUp = 80;
    // go to start position
    goToPosition(0, 148, 80);
    // go forward, backward
    goToPosition(0, 168, 80);
    goToPosition(0, 128, 80);
    goToPosition(0, 148, 80);
    // go up, down
    goToPosition(0, 148, 100);
    goToPosition(0, 148, 60);
    goToPosition(0, 148, 80);
    // go right, left
    goToPosition(40, 148, 80);
    goToPosition(-40, 148, 80);
    goToPosition(0, 148, 80);
}

void doSwitchEasingType() {
    if (!sInverseKinematicModeActive && !sCurrentCommandIsRepeat) {
        Serial.print(F("Set easing type to "));
        if (sEasingType == EASE_LINEAR) {
            setEasingType(EASE_QUADRATIC_IN_OUT);
            Serial.print(F("quadratic"));
        } else if (sEasingType == EASE_QUADRATIC_IN_OUT) {
            setEasingType(EASE_SINE_IN_OUT);
            Serial.print(F("sine"));
        } else if (sEasingType == EASE_SINE_IN_OUT) {
            setEasingType(EASE_CUBIC_IN_OUT);
            Serial.print(F("cubic"));
        } else if (sEasingType == EASE_CUBIC_IN_OUT) {
            setEasingType(EASE_BOUNCE_OUT);
            Serial.print(F("bounce out"));
        } else if (sEasingType == EASE_BOUNCE_OUT) {
            setEasingType(EASE_LINEAR);
            Serial.print(F("linear"));
        }
        Serial.println();
    }
}

/*
 * Switch mode between Inverse-Kinematic and normal easing
 */
void doInverseKinematicOn() {
    sInverseKinematicModeActive = true;
}

void doInverseKinematicOff() {
    sInverseKinematicModeActive = false;
}

void doToggleInverseKinematic() {
    if (!sCurrentCommandIsRepeat) {
        sInverseKinematicModeActive = !sInverseKinematicModeActive;
    }
}

void doAutoMove() {

    if (sInverseKinematicModeActive) {
        setEasingTypeForAllServos(EASE_USER_DIRECT);
        ClawServo.setEasingType(EASE_LINEAR);
    }

    // init start position for first move
    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = 148;
    sEndPosition.DownUp = 80;

//    testInverseKinematic();

    ClawServo.easeTo(CLAW_OPEN_ANGLE);

    // go down and close claw
    goToPosition(0, 148, -55);
    RETURN_IF_STOP;
    ClawServo.easeTo(CLAW_CLOSE_ANGLE);

// move up a bit
    goToPosition(0, 148, 20);
    RETURN_IF_STOP;

// move up, turn right and open claw
    goToPosition(120, 148, 100);
    RETURN_IF_STOP;

    ClawServo.easeTo(CLAW_OPEN_ANGLE);

// turn left
    goToPosition(-120, 0, 40);
    RETURN_IF_STOP;

    ClawServo.easeTo(CLAW_CLOSE_ANGLE);

// must go back to start
    goToPosition(0, 148, 80);
    RETURN_IF_STOP;

    setEasingTypeForAllServos(EASE_LINEAR);
    delay(1000);
}

void doFolded() {
    setAllServos(4, 90, 0, 100, 70);
}

void doCenter() {
    setAllServos(4, PIVOT_NEUTRAL_ANGLE, HORIZONTAL_NEUTRAL_ANGLE, LIFT_NEUTRAL_ANGLE, CLAW_START_ANGLE);
}

void doGoBack() {
    if (sHorizontalServoAngle > 2) {
        sHorizontalServoAngle -= 2;
        HorizontalServo.easeTo(sHorizontalServoAngle);
    }

}

void doGoForward() {
    if (sHorizontalServoAngle < 178) {
        sHorizontalServoAngle += 2;
        HorizontalServo.easeTo(sHorizontalServoAngle);
    };
}

void doTurnRight() {
    if (sBodyPivotAngle > 2) {
        sBodyPivotAngle -= 2;
        BasePivotServo.easeTo(sBodyPivotAngle);
    }

}

void doTurnLeft() {
    if (sBodyPivotAngle <= 178) {
        sBodyPivotAngle += 2;
        BasePivotServo.easeTo(sBodyPivotAngle);
    }

}

void doLiftUp() {
    if (sLiftServoAngle <= LIFT_MAX_ANGLE - 2) {
        sLiftServoAngle += 2;
        LiftServo.easeTo(sLiftServoAngle);
    }

}

void doLiftDown() {
    if (sLiftServoAngle > 2) {
        sLiftServoAngle -= 2;
        LiftServo.easeTo(sLiftServoAngle);
    }

}

void doOpenClaw() {
    if (sClawServoAngle > 2) {
        sClawServoAngle -= 2;
        ClawServo.easeTo(sClawServoAngle);
    }

}

void doCloseClaw() {
    if (sClawServoAngle <= (CLAW_MAX_ANGLE - 2)) {
        sClawServoAngle += 2;
        ClawServo.easeTo(sClawServoAngle);
    }

}

/*
 * return true sets sRequestToStop in turn
 */
void doSwitchToManual() {
    sMode = MODE_MANUAL;
    sRequestToStop = true;
}

/*
 * Set easing type for all servos except claw
 */
void setEasingType(uint8_t aEasingType) {
    sEasingType = aEasingType;
    for (uint8_t tServoIndex = 0; tServoIndex < NUMBER_OF_SERVOS - 1; ++tServoIndex) {
        sServoArray[tServoIndex]->setEasingType(aEasingType);
    }
}

/*
 *  aBasePivot,  aHorizontal,  aLift,  aClaw
 */
void setAllServos(uint8_t aNumberOfValues, ...) {
#ifdef DEBUG
    printArrayPositions(&Serial);
#endif
    va_list aDegreeValues;
    va_start(aDegreeValues, aNumberOfValues);
    setDegreeForAllServos(aNumberOfValues, &aDegreeValues);
    va_end(aDegreeValues);
#ifdef DEBUG
    printArrayPositions(&Serial);
#endif
    synchronizeMoveAllServosAndCheckInputAndWait();
}

/*******************************************
 * Other stuff
 ******************************************/
void changeEasingType(__attribute__((unused)) bool aButtonToggleState) {
    doSwitchEasingType();

}
/*
 * returns true if potentiometers has once been operated
 */
#define ANALOG_HYSTERESIS 6
bool handleManualControl() {
    static bool sManualActionHappened;
    static bool isInitialized = false;

    static int sLastPivot;
    static int sLastHorizontal;
    static int sLastLift;
    static int sLastClaw;

    int tTargetAngle;
// reset manual action after the first move to manual start position
    if (!isInitialized) {
        sLastPivot = analogRead(PIVOT_INPUT_PIN);
        sLastHorizontal = analogRead(HORIZONTAL_INPUT_PIN);
        sLastLift = analogRead(LIFT_INPUT_PIN);
        sLastClaw = analogRead(CLAW_INPUT_PIN);
        isInitialized = true;
    }

    int tPivot = analogRead(PIVOT_INPUT_PIN);
    if (abs(sLastPivot - tPivot) > ANALOG_HYSTERESIS) {
        sLastPivot = tPivot;
        tTargetAngle = map(tPivot, 0, 1023, 0, 180);
        moveOneServoAndCheckInput(SERVO_BASE_PIVOT, tTargetAngle);
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
        moveOneServoAndCheckInput(SERVO_HORIZONTAL, tTargetAngle);
        // TODO
        printArrayPositions(&Serial);

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
        moveOneServoAndCheckInput(SERVO_LIFT, tTargetAngle);
        Serial.print("LiftServo: micros=");
        Serial.print(LiftServo.getEndMicrosecondsOrUnits());
        Serial.print(" degree=");
        Serial.println(tTargetAngle);
        sManualActionHappened = true;
    }

    int tClaw = analogRead(CLAW_INPUT_PIN);
    if (abs(sLastClaw - tClaw) > (ANALOG_HYSTERESIS * 2)) {
        sLastClaw = tClaw;
        tTargetAngle = map(tClaw, 0, 1023, 0, CLAW_MAX_ANGLE);
        moveOneServoAndCheckInput(SERVO_CLAW, tTargetAngle);
        Serial.print("ClawServo: micros=");
        Serial.print(ClawServo.getEndMicrosecondsOrUnits());
        Serial.print(" degree=");
        Serial.println(tTargetAngle);
        sManualActionHappened = true;
    }

    return sManualActionHappened;
}
