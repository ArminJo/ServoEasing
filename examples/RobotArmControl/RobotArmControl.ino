/*
 * RobotArmControl.cpp
 *
 * Program for controlling a RobotArm with 4 servos using 4 potentiometers and/or an IR remote at pin A0
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

#include "RobotArmControl.h"

#define INFO // enable some prints

#if defined(ROBOT_ARM_IR_CONTROL)
#define USE_TINY_IR_RECEIVER // must be specified before including IRCommandDispatcher.hpp to define which IR library to use
#include "IRCommandMapping.h" // must be included before IRCommandDispatcher.hpp to define IR_ADDRESS and IRMapping and string "unknown".
#include "IRCommandDispatcher.hpp"
#include "RobotArmRTCControl.h"
#endif

#if defined(ROBOT_ARM_RTC_CONTROL)
#include "ClockMovements.h"
#endif

#include "Commands.h"
#include "RobotArmServoConfiguration.h"
#include "RobotArmServoControl.h"

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

/*
 * Activate this and put your own code here
 */
//void doAutoMove() {
//    return;
//}
//#define TRACE
//#define DEBUG
#define USE_BUTTON_0
#define USE_ATTACH_INTERRUPT // to be compatible with IRLremote
#include "EasyButtonAtInt01.hpp" // for switching easing modes

#include "ADCUtils.h" // for get getVCCVoltageMillivolt

#define VERSION_EXAMPLE "2.0"

void changeEasingType(bool aButtonToggleState);
EasyButton OnOffButtonAtPin3(&changeEasingType);

void handleManualControl();

bool sManualActionHappened = false;
bool sVCCTooLow = false;

/*
 * Code starts here
 */
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    /*
     * delay() to avoid uncontrolled servo moving after power on.
     * Then the Arduino is reset many times which may lead to invalid servo signals.
     */
    delay(200);
    setupRobotArmServos();

#if defined(ROBOT_ARM_IR_CONTROL)
    IRDispatcher.init();
#endif

#if defined(ROBOT_ARM_RTC_CONTROL)
    initRTC();
    Serial.print(F("RTC temperature="));
    printRTCTemperature();
    Serial.println();
#endif

    // Output VCC voltage
    printVCCVoltageMillivolt(&Serial);
    Serial.println(F("Initialized"));
    delay(300);
}

void loop() {

    // Reset mode to linear for all servos
    if (sInverseKinematicModeActive) {
        setEasingTypeForAllServos(EASE_LINEAR);
    }

    checkVCC();

#if defined(ROBOT_ARM_RTC_CONTROL)
#ifdef DEBUG
    if (printRTCEveryPeriod(1)) {
        printRTC(DATE_FORMAT_EUROPEAN);
        printRTC(DATE_FORMAT_EUROPEAN_LONG);
        printRTC(DATE_FORMAT_AMERICAN);
        printRTC(DATE_FORMAT_AMERICAN_LONG);
    }
#else
    printRTCEveryPeriod(60, DATE_FORMAT_EUROPEAN_LONG);
#endif

    // check for changed time and draw it
    if (sActionType == ACTION_TYPE_DRAW_TIME) {
        checkTimeAndDraw(&RTC_DS3231);
    }
#endif

#if defined(ROBOT_ARM_IR_CONTROL)
    /*
     * Check for IR commands and execute them
     */
    IRDispatcher.loop();

    /*
     * Do auto move if timeout after boot was reached and no IR command was received
     */
    if (sActionType != ACTION_TYPE_DRAW_TIME && IRDispatcher.IRReceivedData.MillisOfLastCode != 0 && !sManualActionHappened && !sVCCTooLow
            && (millis() > MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE)) {
        doRobotArmAutoMove();
    }

    if (sActionType != ACTION_TYPE_DRAW_TIME && IRDispatcher.IRReceivedData.MillisOfLastCode == 0) {
        handleManualControl();
    }
#else
    handleManualControl();
#endif

}

/*
 * Special delay function for the robot arm control.
 * It checks for low voltage and IR input
 * @return  true - if exit condition occurred like stop received
 */
bool delayAndCheckForRobotArm(uint16_t aDelayMillis) {
    uint32_t tStartMillis = millis();

    // check only once per delay
    if (checkVCC()) {
        do {
#if defined(ROBOT_ARM_IR_CONTROL)
            if (IRDispatcher.checkIRInputForAlwaysExecutableCommand()) {
                Serial.println(F("Invalid or recursive regular command received -> set stop and exit from delayAndCheck"));
                sActionType = ACTION_TYPE_STOP;
                return true;
            }
#endif
            yield();
        } while (millis() - tStartMillis < aDelayMillis);
        return false;
    }
#if defined(ROBOT_ARM_IR_CONTROL)
    sActionType = ACTION_TYPE_STOP;
#endif
    return true;
}

/*
 * checks VCC periodically and sets sVCCTooLow
 */
bool checkVCC() {
    static uint8_t sVoltageTooLowCounter;
    static uint16_t sLastVoltage;
    static long sLastMillisOfVoltageCheck;
    if (millis() - sLastMillisOfVoltageCheck >= VCC_CHECK_PERIOD_MILLIS) {
        sLastMillisOfVoltageCheck = millis();
        uint16_t tVCC = getVCCVoltageMillivolt();

#ifdef INFO
        if (sLastVoltage != tVCC) {
            sLastVoltage = tVCC;
            Serial.print(F("VCC="));
            Serial.print(tVCC);
            Serial.println(" mV");
        }
#endif

        // one time flag
        if (!sVCCTooLow) {
            if (tVCC < VCC_STOP_THRESHOLD_MILLIVOLT) {
                /*
                 * Voltage too low, wait VCC_STOP_PERIOD_REPETITIONS (9) times and then shut down.
                 */
                if (tVCC < VCC_STOP_MIN_MILLIVOLT) {
                    // emergency shutdown
                    sVoltageTooLowCounter = VCC_STOP_PERIOD_REPETITIONS;
                    Serial.println(F("Voltage < 3.2 volt detected"));
                } else {
                    sVoltageTooLowCounter++;
                    Serial.println(F("Voltage < 3.4 volt detected"));
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
    return sVCCTooLow;
}

/*
 * Button callback function
 */
void changeEasingType(__attribute__((unused)) bool aButtonToggleState) {
    doSwitchEasingType();
}

void doSetToAutoModeForRobotArm() {
#if defined(ROBOT_ARM_IR_CONTROL)
    IRDispatcher.IRReceivedData.MillisOfLastCode = 1;
#endif
    sManualActionHappened = false;
}

/*
 * Sets sManualActionHappened if potentiometers has once been operated
 */
#define ANALOG_HYSTERESIS 10 // 1 degree at a 100 degree scale
#define ANALOG_HYSTERESIS_FOR_MANUAL_ACTION 20
void handleManualControl() {
    static bool isInitialized = false;

    static int sFirstPivot, sLastPivot;
    static int sFirstHorizontal, sLastHorizontal;
    static int sFirstLift, sLastLift;
    static int sFirstClaw, sLastClaw;

    static long sLastMillisOfManualCheck;
    if (millis() - sLastMillisOfManualCheck >= MANUAL_CHECK_PERIOD_MILLIS) {
        sLastMillisOfManualCheck = millis();

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
#ifdef INFO
            Serial.print("BasePivotServo: micros=");
            Serial.print(BasePivotServo.getEndMicrosecondsOrUnitsWithTrim());
#endif
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
#ifdef INFO
            Serial.print("HorizontalServo: micros=");
            Serial.print(HorizontalServo.getEndMicrosecondsOrUnits());
#endif
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
#ifdef INFO
            Serial.print("LiftServo: micros=");
            Serial.print(LiftServo.getEndMicrosecondsOrUnits());
#endif
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
#ifdef INFO
            Serial.print("ClawServo: micros=");
            Serial.print(ClawServo.getEndMicrosecondsOrUnits());
#endif
        }

        if (tValueChanged) {
            if (tManualAction) {
                sManualActionHappened = true;
            }
#ifdef INFO
            Serial.print(" degree=");
            Serial.print(tTargetAngle);
            Serial.print(" | ");
#endif
            sEndPosition.LeftRightDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_BASE_PIVOT];
            sEndPosition.BackFrontDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_HORIZONTAL];
            sEndPosition.DownUpDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_LIFT];
            unsolve(&sEndPosition);
            printPositionShortWithUnits(&sEndPosition);
        }
    }
}
