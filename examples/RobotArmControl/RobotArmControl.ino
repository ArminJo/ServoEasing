/*
 * RobotArmControl.cpp
 *
 * Program for controlling a RobotArm with 4 servos using 4 potentiometers and/or an IR Remote at pin A0
 * See also: https://www.instructables.com/id/4-DOF-Mechanical-Arm-Robot-Controlled-by-Arduino
 *
 * To run this example need to install the "ServoEasing", "IRLremote" and "PinChangeInterrupt" libraries under "Tools -> Manage Libraries..." or "Ctrl+Shift+I"
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

#include "Commands.h"
#include "IRCommandDispatcher.h"
#include "RobotArmServoControl.h"

#define VCC_STOP_THRESHOLD_MILLIVOLT 3600 // stop moving if below 3.6 Volt

#define MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE 10000

//#define TRACE
//#define DEBUG

#define USE_BUTTON_0
#define USE_ATTACH_INTERRUPT // to be compatible with IRLremote
#include "EasyButtonAtInt01.h" // for switching easing modes

#include "ADCUtils.h" // for get getVCCVoltageMillivolt

#define VERSION_EXAMPLE "2.0"

void changeEasingType(bool aButtonToggleState);
EasyButton Button0AtPin2(true, &changeEasingType);

void handleManualControl();
bool sManualActionHappened = false;

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
     * Then the Arduino is reset many times which may lead to invalid servo signals.
     */
    delay(200);
    setupRobotArmServos();

    setupIRDispatcher();

    // Output VCC Voltage
    uint16_t tVoltageMillivolts = getVCCVoltageMillivolt();
    Serial.print(F("VCC="));
    Serial.print(tVoltageMillivolts);
    Serial.println(" mV");
    Serial.println("Initialized");
    delay(300);
}

void loop() {

    // Reset mode to linear for all movements using inverse kinematic
    if (sInverseKinematicModeActive) {
        setEasingTypeForAllServos(EASE_LINEAR);
    }

    /*
     * Check for IR commands and execute them
     */
    loopIRDispatcher();

    /*
     * Do auto move if timeout after boot was reached and no IR command was received
     */
    if (!sValidIRCodeReceived && !sManualActionHappened && (millis() > MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE)) {
        doAutoMove();
    }

    if (!sValidIRCodeReceived) {
        handleManualControl();
    }

    /*
     * Stop servos if voltage gets low
     */
    uint16_t tVoltageMillivolts = getVCCVoltageMillivolt();
    if (tVoltageMillivolts < VCC_STOP_THRESHOLD_MILLIVOLT) {
        Serial.print(F("VCC="));
        Serial.print(tVoltageMillivolts);
        Serial.print(" mV -> ");
        shutdownServos();
    }
}

/*
 * Button callback function
 */
void changeEasingType(__attribute__((unused)) bool aButtonToggleState) {
    doSwitchEasingType();
}

void setToAutoMode() {
    sValidIRCodeReceived = false;
    sManualActionHappened = false;
}

/*
 * Sets sManualActionHappened if potentiometers has once been operated
 */
#define ANALOG_HYSTERESIS 6
void handleManualControl() {
    static bool isInitialized = false;

    static int sLastPivot;
    static int sLastHorizontal;
    static int sLastLift;
    static int sLastClaw;

    /*
     * Dummy read to set the reference and let it settle
     */
    analogRead(PIVOT_INPUT_PIN);
    delay(1);

    bool tManualAction = false; // do only one servo at a time
    int tTargetAngle = 0; // to avoid compiler warnings
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
        tTargetAngle = map(tPivot, 0, 1023, 10 + PIVOT_OFFSET, -10 - PIVOT_OFFSET);
        moveOneServoAndCheckInputAndWait(SERVO_BASE_PIVOT, tTargetAngle);
        Serial.print("BasePivotServo: micros=");
        Serial.print(BasePivotServo.getEndMicrosecondsOrUnitsWithTrim());
        tManualAction = true;
    }

    int tHorizontal = analogRead(HORIZONTAL_INPUT_PIN);
    if (!tManualAction && abs(sLastHorizontal - tHorizontal) > ANALOG_HYSTERESIS) {
        sLastHorizontal = tHorizontal;
        tTargetAngle = map(tHorizontal, 0, 1023, 0, 180);
        moveOneServoAndCheckInputAndWait(SERVO_HORIZONTAL, tTargetAngle);
        Serial.print("HorizontalServo: micros=");
        Serial.print(HorizontalServo.getEndMicrosecondsOrUnits());
        tManualAction = true;
    }

    int tLift = analogRead(LIFT_INPUT_PIN);
    if (!tManualAction && abs(sLastLift - tLift) > ANALOG_HYSTERESIS * 2) {
        sLastLift = tLift;
        tTargetAngle = map(tLift, 0, 1023, 0, LIFT_MAX_ANGLE);
        moveOneServoAndCheckInputAndWait(SERVO_LIFT, tTargetAngle);
        Serial.print("LiftServo: micros=");
        Serial.print(LiftServo.getEndMicrosecondsOrUnits());
        tManualAction = true;
    }

    int tClaw = analogRead(CLAW_INPUT_PIN);
    if (!tManualAction && abs(sLastClaw - tClaw) > (ANALOG_HYSTERESIS * 2)) {
        sLastClaw = tClaw;
        tTargetAngle = map(tClaw, 0, 1023, 0, CLAW_MAX_ANGLE);
        moveOneServoAndCheckInputAndWait(SERVO_CLAW, tTargetAngle);
        Serial.print("ClawServo: micros=");
        Serial.print(ClawServo.getEndMicrosecondsOrUnits());
        tManualAction = true;
    }

    if (tManualAction) {
        sManualActionHappened = true;
        Serial.print(" degree=");
        Serial.print(tTargetAngle);
        Serial.print(" | ");
        sEndPosition.LeftRightDegree = sServoNextPositionArray[SERVO_BASE_PIVOT];
        sEndPosition.BackFrontDegree = sServoNextPositionArray[SERVO_HORIZONTAL];
        sEndPosition.DownUpDegree = sServoNextPositionArray[SERVO_LIFT];
        unsolve(&sEndPosition);
        printPositionShortWithUnits(&sEndPosition);
    }
}

