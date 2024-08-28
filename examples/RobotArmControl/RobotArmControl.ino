/*
 * RobotArmControl.cpp
 *
 * Program for controlling a RobotArm with 4 servos using 4 potentiometers and/or an IR remote at pin A0
 * See also: https://www.instructables.com/id/4-DOF-Mechanical-Arm-Robot-Controlled-by-Arduino
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

#include "RobotArmControl.h"

#define INFO // enable some prints
//#define ROBOT_ARM_HAS_IR_CONTROL
//#define ROBOT_ARM_HAS_RTC_CONTROL
//#define ROBOT_ARM_1 // My black one
//#define ROBOT_ARM_2 // My transparent one

#if defined(ROBOT_ARM_HAS_IR_CONTROL)
#define USE_TINY_IR_RECEIVER // must be specified before including IRCommandDispatcher.hpp to define which IR library to use
#define IR_RECEIVE_PIN  A0
#if defined(ROBOT_ARM_2)
#define USE_CAR_MP3_REMOTE // Transparent arm
#else
#define USE_MSI_REMOTE // Black arm
#endif
#include "RobotArmIRCommandMapping.h" // must be included before IRCommandDispatcher.hpp to define IR_ADDRESS and IRMapping and string "unknown".
#include "IRCommandDispatcher.hpp"
#endif

#define ROBOT_ARM_INITIAL_SERVO_SPEED   80 // in degree/second or millimeter/second for inverse kinematic
#include "RobotArmServoControl.hpp" // includes ServoEasing.hpp

#if defined(ROBOT_ARM_HAS_RTC_CONTROL)
//#define GERMAN_NAMES_FOR_DATE
#include "RobotArmRTCControl.hpp"
#endif
#define LOCAL_INFO // This enables info output only for ClockMovements
#include "ClockMovements.hpp"

#include "RobotArmIRCommands.hpp"

/*
 * The auto move function. Put your own moves here.
 *
 * Servos available:
 * BasePivotServo (-90 to +90), HorizontalServo, LiftServo, ClawServo
 *
 * Useful commands:
 *
 * goToCenter();
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
#define USE_BUTTON_0 // use button at INT0 / pin 2
#include "EasyButtonAtInt01.hpp" // for switching easing modes
void changeEasingType(bool aButtonToggleState);
EasyButton ButtonAtPin2(&changeEasingType);

/*
 * Default values are suitable for Li-ion batteries.
 * We normally have voltage drop at the connectors, so the battery voltage is assumed slightly higher, than the Arduino VCC.
 * But keep in mind that the ultrasonic distance module HC-SR04 may not work reliable below 3.7 volt.
 */
#define VCC_STOP_THRESHOLD_MILLIVOLT    3300 // Do not stress your battery and we require some power for standby
#define VCC_EMERGENCY_STOP_MILLIVOLT    3000 // Many Li-ions are specified down to 3.0 volt
#define VCC_CHECK_PERIOD_MILLIS        10000 // Period of VCC checks
#define VCC_CHECKS_TOO_LOW_BEFORE_STOP     6 // Shutdown after 6 times (60 seconds) VCC below VCC_STOP_THRESHOLD_MILLIVOLT or 1 time below VCC_EMERGENCY_STOP_MILLIVOLT
#include "ADCUtils.hpp"
void checkAndHandleVCCUndervoltage();

#define VERSION_EXAMPLE "2.0"

void handleManualControl();
void printConfigPinInfo(Print *aSerial, uint8_t aConfigPinNumber, const __FlashStringHelper *aConfigPinDescription);

#define _TIMEOUT_MILLIS_BEFORE_SWITCH_TO_AUTO_MOVE  30000
#define MILLIS_OF_INACTIVITY_BEFORE_ATTENTION       60000
#define MANUAL_CHECK_PERIOD_MILLIS 100          // Period of manual control checks
unsigned long sMillisOfLastManualAction = 0;    // Disables auto move and attention after initial timeout
unsigned long sMillisOfLastAutoMove = 0;

bool sVCCTooLow = false;

#define DEBUG_OUTPUT_ENABLE_PIN 12 // if low, enables debug output
bool sDebugOutputIsEnabled;
/*
 * Code starts here
 */
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(DEBUG_OUTPUT_ENABLE_PIN, INPUT_PULLUP);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
    Serial.print(F("Button pin="));
    Serial.println(INT0_PIN);

    /*
     * delay() to avoid uncontrolled servo moving after power on.
     * Then the Arduino is reset many times which may lead to invalid servo signals.
     */
    delay(200);
    setupRobotArmServos();

#if defined(ROBOT_ARM_HAS_IR_CONTROL)
    IRDispatcher.init();
    Serial.print(F("Listening to IR remote of type "));
    Serial.print(IR_REMOTE_NAME);
    Serial.println(F(" at pin " STR(IR_RECEIVE_PIN)));
#endif

#if defined(ROBOT_ARM_HAS_RTC_CONTROL)
    initRTC();
    Serial.print(F("RTC temperature="));
    printRTCTemperature();
    Serial.println();
#endif

    sDebugOutputIsEnabled = !digitalRead(DEBUG_OUTPUT_ENABLE_PIN);
    printConfigPinInfo(&Serial, DEBUG_OUTPUT_ENABLE_PIN, F("debug output"));

    // Output VCC voltage
    printVCCVoltageMillivolt(&Serial);
    Serial.println(F("Initialized"));
    delay(300);

}

void loop() {

    sDebugOutputIsEnabled = !digitalRead(DEBUG_OUTPUT_ENABLE_PIN); // enabled if LOW

#if defined(ADC_UTILS_ARE_AVAILABLE)
    checkAndHandleVCCUndervoltage();
#endif // defined(__AVR__)

#if defined(ROBOT_ARM_HAS_RTC_CONTROL)
#  if defined(DEBUG)
    if (printRTCEveryPeriod(1)) {
        printRTC(DATE_FORMAT_EUROPEAN);
        printRTC(DATE_FORMAT_EUROPEAN_LONG);
        printRTC(DATE_FORMAT_AMERICAN);
        printRTC(DATE_FORMAT_AMERICAN_LONG);
    }
#  else
    printRTCEveryPeriod(60, DATE_FORMAT_EUROPEAN_LONG);
#  endif

    // check for changed time and draw it
    if (sActionType == ACTION_TYPE_DRAW_TIME) {
        checkTimeAndDraw(&RTC_DS3231);
    }
#endif

#if defined(ROBOT_ARM_HAS_IR_CONTROL)
    /*
     * Check for IR commands and execute them.
     * Returns only AFTER finishing of requested movement
     */
    IRDispatcher.checkAndRunSuspendedBlockingCommands();

    /*
     * Do auto move if timeout after boot was reached and no IR command was received, and no manual command was issued before
     * Do it all 30 seconds
     */
    if (sActionType != ACTION_TYPE_DRAW_TIME && IRDispatcher.IRReceivedData.MillisOfLastCode == 0 && sMillisOfLastManualAction == 0
            && !sVCCTooLow && ((millis() - sMillisOfLastAutoMove) > _TIMEOUT_MILLIS_BEFORE_SWITCH_TO_AUTO_MOVE)) {
        sMillisOfLastAutoMove = millis();
        doRobotArmAutoMove();
    }

    /*
     * Do attention if last user action was before more than 60 seconds
     */
    if (sActionType != ACTION_TYPE_DRAW_TIME
            && ((millis() - IRDispatcher.IRReceivedData.MillisOfLastCode) > MILLIS_OF_INACTIVITY_BEFORE_ATTENTION)
            && ((millis() - sMillisOfLastManualAction) > MILLIS_OF_INACTIVITY_BEFORE_ATTENTION)) {
        sMillisOfLastManualAction = millis();
        doRobotArmAttention();
    }

    /*
     * Enable manual control if no IR command was received
     */
    if (sActionType != ACTION_TYPE_DRAW_TIME && IRDispatcher.IRReceivedData.MillisOfLastCode == 0) {
        IRDispatcher.requestToStopReceived = false; // Stop was processed :-)
        handleManualControl();
    }
#else
    handleManualControl();
#endif // defined(ROBOT_ARM_HAS_IR_CONTROL)

}

/*
 * Special delay function for the robot arm control.
 * It checks for IR input
 * @return  true - if exit condition occurred like stop received
 */
bool delayAndCheckForRobotArm(uint16_t aDelayMillis) {
#if defined(ROBOT_ARM_HAS_IR_CONTROL)
    if (IRDispatcher.delayAndCheckForStop(aDelayMillis)) {
        Serial.println(F("Stop requested"));
        sActionType = ACTION_TYPE_STOP;
        return true;
    }
#endif
    return false;
}

#if defined(ADC_UTILS_ARE_AVAILABLE)
/*
 * If isVCCTooLowMultipleTimes() returns true clear all pattern and activate only 2 MultipleFallingStars pattern on the 2 bars
 */
void checkAndHandleVCCUndervoltage() {
    if (isVCCUndervoltageMultipleTimes()) {
        goToFolded(); // Afterwards only auto move is disabled
#if defined(ROBOT_ARM_HAS_IR_CONTROL)
        sActionType = ACTION_TYPE_STOP;
#endif
        Serial.println(F("Shut down"));
    }
}
#endif // defined(ADC_UTILS_ARE_AVAILABLE)

/*
 * Button callback function
 */
void changeEasingType(__attribute__((unused)) bool aButtonToggleState) {
    doSwitchEasingType();
}

void doEnableAutoModeForRobotArm() {
#if defined(ROBOT_ARM_HAS_IR_CONTROL)
    IRDispatcher.IRReceivedData.MillisOfLastCode = 1;
#endif
    sMillisOfLastManualAction = 0;
}

/*
 * open and close claw
 */
void doRobotArmAttention() {
    Serial.println(F("Do attention"));
    ClawServo.easeTo(90, 120);
    ClawServo.easeTo(0, 120);
}

float mapSpecial(int x, int in_min, int in_max, int out_min, int out_max) {
    return ((float) ((long) (x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}

/*
 * Sets sManualActionHappened if potentiometers has once been operated
 */
#define ANALOG_HYSTERESIS 4 // 0.4 degree at a 100 degree scale
#define ANALOG_HYSTERESIS_FOR_MANUAL_ACTION 20 // is greater than ANALOG_HYSTERESIS to avoid false detection of manual control
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

        float tTargetAngle = 0; // to avoid compiler warnings

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
            tTargetAngle = mapSpecial(tPivot, 0, 1023, 100, -100);
            moveOneServoAndCheckInputAndWait(SERVO_BASE_PIVOT, tTargetAngle, sRobotArmServoSpeed);

            Serial.print(F("BasePivotServo: micros="));
            Serial.print(BasePivotServo.getEndMicrosecondsOrUnits()); // we have an initial trim, this is transparent.
        }

        int tHorizontal = analogRead(HORIZONTAL_INPUT_PIN);
        if (!tValueChanged && abs(sLastHorizontal - tHorizontal) > ANALOG_HYSTERESIS) {
            tValueChanged = true;
            // check for manual action
            if (abs(sFirstHorizontal - tHorizontal) > ANALOG_HYSTERESIS_FOR_MANUAL_ACTION) {
                tManualAction = true;
            }
            sLastHorizontal = tHorizontal;
            tTargetAngle = mapSpecial(tHorizontal, 0, 1023, HORIZONTAL_MINIMUM_DEGREE, HORIZONTAL_MAXIMUM_DEGREE);
            moveOneServoAndCheckInputAndWait(SERVO_HORIZONTAL, tTargetAngle, sRobotArmServoSpeed);

            Serial.print(F("HorizontalServo: micros="));
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
            tTargetAngle = mapSpecial(tLift, 0, 1023, LIFT_MINIMUM_DEGREE - 10, LIFT_MAXIMUM_DEGREE + 10);
            moveOneServoAndCheckInputAndWait(SERVO_LIFT, tTargetAngle, sRobotArmServoSpeed);

            Serial.print(F("LiftServo: micros="));
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
            tTargetAngle = mapSpecial(tClaw, 0, 1023, CLAW_CLOSE_DEGREE - 10, CLAW_MAXIMUM_DEGREE + 10);
            // "sRobotArmServoSpeed * 2" makes claw a lot more responsive
            moveOneServoAndCheckInputAndWait(SERVO_CLAW, tTargetAngle, sRobotArmServoSpeed * 2);

            Serial.print(F("ClawServo: micros="));
            Serial.print(ClawServo.getEndMicrosecondsOrUnits());
        }

        if (tValueChanged) {
            if (tManualAction) {
                sMillisOfLastManualAction = millis();
            }
            Serial.print(F(" degree="));
            Serial.print(tTargetAngle);
            Serial.print(F(" | "));
            // Compute forward kinematics for printing
            sEndPosition.LeftRightDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_BASE_PIVOT];
            sEndPosition.BackFrontDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_HORIZONTAL];
            sEndPosition.DownUpDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_LIFT];
            doForwardKinematics(&sEndPosition);
            printPositionShortWithUnits(&sEndPosition);
        }
    }
}

void printConfigPinInfo(Print *aSerial, uint8_t aConfigPinNumber, const __FlashStringHelper *aConfigPinDescription) {
    aSerial->print(F("Pin "));
    aSerial->print(aConfigPinNumber);
    aSerial->print(F(" is"));
    bool tIsEnabled = digitalRead(aConfigPinNumber) == LOW;
    if (!tIsEnabled) {
        aSerial->print(F(" not"));
    }
    aSerial->print(F(" connected to ground, "));
    aSerial->print(aConfigPinDescription);
    aSerial->print(F(" is "));
    if (tIsEnabled) {
        aSerial->println(F("enabled"));
    } else {
        aSerial->println(F("disabled"));
    }
}

