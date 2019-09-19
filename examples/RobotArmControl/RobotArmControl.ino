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
#include "RobotArmServoConfiguration.h"
#include "RobotArmServoControl.h"
#include "uRTCLib.h"
#include "ClockMovements.h"

/*
 * The auto move function. Put your own moves here.
 *
 * Servos available:
 * BasePivotServo (-90 to +90), HorizontalServo, LiftServo, ClawServo
 *
 * Useful commands:
 *
 * goToNeutral()
 * goToPosition(int aLeftRightMilliMeter, int aBackFrontMilliMeter, int aDownUpMilliMeter);
 * goToPositionRelative(int aLeftRightDeltaMilliMeter, int aBackFrontDeltaMilliMeter, int aDownUpDeltaMilliMeter);
 * delayAndCheckIRInput(1000);
 *
 * To move the front left lift servo use:
 * BasePivotServo.easeTo(-90);
 * setLiftServos(LIFT_MIN_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE, LIFT_MAX_ANGLE);
 * setPivotServos(100, 100, 80, 80);
 */

void doAutoMove() {
    /*
     * comment this out and put your own code here
     */
    internalAutoMove();
    return;

}

#define VCC_STOP_THRESHOLD_MILLIVOLT 3500 // We have voltage drop at the connectors, so the battery voltage is assumed higher, than the Arduino VCC.
#define VCC_STOP_MIN_MILLIVOLT 3200         // We have voltage drop at the connectors, so the battery voltage is assumed higher, than the Arduino VCC.
#define VCC_STOP_PERIOD_MILLIS 2000         // Period of VCC checks
#define VCC_STOP_PERIOD_REPETITIONS 9       // Shutdown after 9 times (18 seconds) VCC below VCC_STOP_THRESHOLD_MILLIVOLT or 1 time below VCC_STOP_MIN_MILLIVOLT

#define MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE 30000

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
void checkVCC();
void printRTC();
void testRTC();

bool sManualActionHappened = false;
bool sVCCTooLow = false;

uRTCLib RTC_DS3231;

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

#ifndef ROBOT_ARM_2
    Wire.begin();
    RTC_DS3231.set_model(URTCLIB_MODEL_DS3231);
    Serial.println("Set date");
    RTC_DS3231.set(0, 58, 18, 4, 14, 8, 19);
    //  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
#endif

    // Output VCC Voltage
    uint16_t tVoltageMillivolts = getVCCVoltageMillivolt();
    Serial.print(F("VCC="));
    Serial.print(tVoltageMillivolts);
    Serial.println(" mV");
    Serial.println("Initialized");
    delay(300);
}

void loop() {

    // Reset mode to linear for all servos
    if (sInverseKinematicModeActive) {
        setEasingTypeForAllServos(EASE_LINEAR);
    }

#ifndef ROBOT_ARM_2
    printRTC();
#endif

    checkVCC();

    /*
     * Check for IR commands and execute them
     */
    loopIRDispatcher();
    // check for changed time and draw it
    if (sDrawTime) {
        checkTimeAndDraw(&RTC_DS3231);
    }

    /*
     * Do auto move if timeout after boot was reached and no IR command was received
     */
    if (!sDrawTime && !sAtLeastOneValidIRCodeReceived && !sManualActionHappened && !sVCCTooLow
            && (millis() > MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE)) {
        doAutoMove();
    }

    if (!sDrawTime && !sAtLeastOneValidIRCodeReceived) {
        handleManualControl();
    }

}

/*
 * checks VCC periodically and sets sVCCTooLow
 */
void checkVCC() {
    static uint8_t sVoltageTooLowCounter;
    static long sLastMillisOfVoltageCheck;
    if (millis() - sLastMillisOfVoltageCheck >= VCC_STOP_PERIOD_MILLIS) {
        sLastMillisOfVoltageCheck = millis();
        uint16_t tVCC = getVCCVoltageMillivolt();

        Serial.print(F("VCC="));
        Serial.print(tVCC);
        Serial.println(" mV");

        // one time flag
        if (!sVCCTooLow) {
            if (tVCC < VCC_STOP_THRESHOLD_MILLIVOLT) {
                /*
                 * Voltage too low, wait VCC_STOP_PERIOD_REPETITIONS (9) times and then shut down.
                 */
                if (tVCC < VCC_STOP_MIN_MILLIVOLT) {
                    // emergency shutdown
                    sVoltageTooLowCounter = VCC_STOP_PERIOD_REPETITIONS;
                    Serial.println(F("Voltage < 3.2 Volt detected"));
                } else {
                    sVoltageTooLowCounter++;
                    Serial.println(F("Voltage < 3.4 Volt detected"));
                }
                if (sVoltageTooLowCounter == VCC_STOP_PERIOD_REPETITIONS) {
                    Serial.println(F("Shut down"));
                    sVCCTooLow = true;
                    /*
                     * Do it once and wait for 5 seconds
                     * Afterwards only auto move is disabled
                     */
                    shutdownServos();
                    delay(5000);
                }
            } else {
                sVoltageTooLowCounter = 0;
            }
        }
    }
}

/*
 * Button callback function
 */
void changeEasingType(__attribute__((unused)) bool aButtonToggleState) {
    doSwitchEasingType();
}

void setToAutoMode() {
    sAtLeastOneValidIRCodeReceived = false;
    sManualActionHappened = false;
}

/*
 * Sets sManualActionHappened if potentiometers has once been operated
 */
#define ANALOG_HYSTERESIS 2
#define ANALOG_HYSTERESIS_FOR_MANUAL_ACTION 16
void handleManualControl() {
    static bool isInitialized = false;

    static int sFirstPivot, sLastPivot;
    static int sFirstHorizontal, sLastHorizontal;
    static int sFirstLift, sLastLift;
    static int sFirstClaw, sLastClaw;

    /*
     * Dummy read to set the reference and let it settle
     */
    analogRead(PIVOT_INPUT_PIN);
    delay(1);

    bool tValueChanged = false; // do only one servo at a time
    bool tManualAction = false; // gets true if value changed more than ANALOG_HYSTERESIS_FOR_MANUAL_ACTION

    int tTargetAngle = 0; // to avoid compiler warnings

// reset manual action after the first move to manual start position
    if (!isInitialized) {
        sFirstPivot = sLastPivot = analogRead(PIVOT_INPUT_PIN);
        sFirstHorizontal = sLastHorizontal = analogRead(HORIZONTAL_INPUT_PIN);
        sFirstLift = sLastLift = analogRead(LIFT_INPUT_PIN);
        sFirstClaw = sLastClaw = analogRead(CLAW_INPUT_PIN);
        isInitialized = true;
    }

    int tPivot = analogRead(PIVOT_INPUT_PIN);
    if (abs(sLastPivot - tPivot) > ANALOG_HYSTERESIS) {
        tValueChanged = true;
        // check for manual action
        if (abs(sFirstPivot - tPivot) > ANALOG_HYSTERESIS_FOR_MANUAL_ACTION) {
            tManualAction = true;
        }
        sLastPivot = tPivot;
        tTargetAngle = map(tPivot, 0, 1023, 10 + PIVOT_OFFSET, -10 - PIVOT_OFFSET);
        moveOneServoAndCheckInputAndWait(SERVO_BASE_PIVOT, tTargetAngle);
        Serial.print("BasePivotServo: micros=");
        Serial.print(BasePivotServo.getEndMicrosecondsOrUnitsWithTrim());
    }

    int tHorizontal = analogRead(HORIZONTAL_INPUT_PIN);
    if (!tValueChanged && abs(sLastHorizontal - tHorizontal) > ANALOG_HYSTERESIS) {
        tValueChanged = true;
        // check for manual action
        if (abs(sFirstHorizontal - tHorizontal) > ANALOG_HYSTERESIS_FOR_MANUAL_ACTION) {
            tManualAction = true;
        }
        sLastHorizontal = tHorizontal;
        tTargetAngle = map(tHorizontal, 0, 1023, 0, 180);
        moveOneServoAndCheckInputAndWait(SERVO_HORIZONTAL, tTargetAngle);
        Serial.print("HorizontalServo: micros=");
        Serial.print(HorizontalServo.getEndMicrosecondsOrUnits());
    }

    int tLift = analogRead(LIFT_INPUT_PIN);
    if (!tValueChanged && abs(sLastLift - tLift) > ANALOG_HYSTERESIS) {
        tValueChanged = true;
        // check for manual action
        if (abs(sFirstLift - tLift) > ANALOG_HYSTERESIS_FOR_MANUAL_ACTION) {
            tManualAction = true;
        }
        sLastLift = tLift;
        tTargetAngle = map(tLift, 0, 1023, 0, LIFT_MAX_ANGLE);
        moveOneServoAndCheckInputAndWait(SERVO_LIFT, tTargetAngle);
        Serial.print("LiftServo: micros=");
        Serial.print(LiftServo.getEndMicrosecondsOrUnits());
    }

    int tClaw = analogRead(CLAW_INPUT_PIN);
    if (!tValueChanged && abs(sLastClaw - tClaw) > (ANALOG_HYSTERESIS)) {
        tValueChanged = true;
        // check for manual action
        if (abs(sFirstClaw - tClaw) > ANALOG_HYSTERESIS_FOR_MANUAL_ACTION) {
            tManualAction = true;
        }
        sLastClaw = tClaw;
        tTargetAngle = map(tClaw, 0, 1023, 0, CLAW_MAX_ANGLE);
        moveOneServoAndCheckInputAndWait(SERVO_CLAW, tTargetAngle);
        Serial.print("ClawServo: micros=");
        Serial.print(ClawServo.getEndMicrosecondsOrUnits());
    }

    if (tValueChanged) {
        if (tManualAction) {
            sManualActionHappened = true;
        }
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

/*
 * Only URTCLIB_SQWG_OFF_1 and URTCLIB_SQWG_1H work on my china module
 */
void testRTC() {
    Serial.println("Testing SQWG/INT output:");

    Serial.println("fixed 1:");
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_OFF_1);
    delay(2000);

    Serial.println("1 hertz:");
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_1H);
    delay(3000);

    Serial.println("1024 hertz:");
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_1024H);
    delay(2000);

    Serial.println("4096 hertz:");
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_4096H);
    delay(2000);

    Serial.println("8192 hertz:");
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_8192H);
    delay(2000);

}

void printRTC() {
    static long sLastMillisOfRTCRead;

    if (millis() - sLastMillisOfRTCRead >= 1000) {
        sLastMillisOfRTCRead = millis();
        RTC_DS3231.refresh();

        Serial.print("RTC DateTime: ");
        Serial.print(RTC_DS3231.day());
        Serial.print('.');
        Serial.print(RTC_DS3231.month());
        Serial.print('.');
        Serial.print(RTC_DS3231.year());

        Serial.print(' ');

        Serial.print(RTC_DS3231.hour());
        Serial.print(':');
        Serial.print(RTC_DS3231.minute());
        Serial.print(':');
        Serial.print(RTC_DS3231.second());

        Serial.print(" DOW: ");
        Serial.print(RTC_DS3231.dayOfWeek());

        Serial.print(" - Temp: ");
        Serial.print(RTC_DS3231.temp());

        Serial.println();
    }
}
