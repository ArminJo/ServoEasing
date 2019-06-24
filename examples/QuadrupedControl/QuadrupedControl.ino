/*
 * QuadrupedControl.cpp
 *
 * Program for controlling a mePed Robot V2 with 8 servos using an IR Remote at pin A0
 * Supported IR remote are KEYES (the original mePed remote) and WM10
 * Select the one you have at line 20 in IRCommandMapping.h
 *
 * This file mainly contains setup and loop and the basic movements
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

#include "Commands.h" // for doAutoMove
#include "IRCommandDispatcher.h"
#include "QuadrupedServoControl.h"
#include "ADCUtils.h"

#define VCC_STOP_THRESHOLD_MILLIVOLT 3600 // stop moving if below 3.6 Volt
#define MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE 20000 // 20 seconds
#define MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE 120000 // 2 Minutes
#define MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE 60000 // 1 Minute

#define VERSION_EXAMPLE "2.0"
// 2.0 major refactoring
// 1.1 mirror computation at transformAndSetPivotServos and transformOneServoIndex

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

    setupQuadrupedServos();
    setSpeedForAllServos(sServoSpeed);

    // Only fore setting channel and reference
    getVCCVoltageMillivoltSimple();

    /*
     * set servo to 90 degree WITHOUT trim and wait
     */
    resetServosTo90Degree();
    delay(2000);

    /*
     * set servo to 90 degree with trim and wait
     */
    eepromReadAndSetServoTrim();
    resetServosTo90Degree();
    delay(2000);

    /*
     * Set to initial height
     */
    centerServos();

    setupIRDispatcher();

}

void loop() {
    /*
     * Reset moving characteristic
     */
    setEasingTypeToLinear();

    /*
     * Check for IR commands and execute them
     */
    loopIRDispatcher();

    /*
     * Do auto move if timeout after boot was reached and no IR command was received
     */
    if (!sValidIRCodeReceived && (millis() > MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE)) {
        doAutoMove();
    }

    if (millis() - sLastTimeOfValidIRCodeReceived > MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE) {
        // Get attention that no command was received since 2 Minutes and quadruped may be switched off
        doAttention();
        // next attention in 1 minute
        sLastTimeOfValidIRCodeReceived += MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE;
    }

    /*
     * Stop servos if voltage gets low
     */
    uint16_t tVCC = getVCCVoltageMillivoltSimple();
    if (tVCC < VCC_STOP_THRESHOLD_MILLIVOLT) {
        Serial.print(F("VCC "));
        Serial.print(tVCC);
        Serial.print(F(" below 3600 Millivolt "));
        shutdownServos();
    }

    delay(50); // Pause for 50ms before executing next movement
}

/*
 * The auto move function. Put your own moves here.
 *
 * Servos available:
 *  frontLeftPivotServo, frontLeftLiftServo
 *  backLeftPivotServo, backLeftLiftServo
 *  backRightPivotServo, backRightLiftServo
 *  frontRightPivotServo, frontRightLiftServo
 *
 * Useful commands:
 *
 * sMovingDirection = MOVE_DIRECTION_FORWARD;
 * moveCreep(1);
 * centerServos();
 * moveTrot(1);
 * moveTurn(1);
 * doLeanLeft();
 * doLeanRight();
 * basicTwist(30);
 * doWave();
 * To move the front left lift servo use:
 * frontLeftLiftServo.easeTo(LIFT_MIN_ANGLE);
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
