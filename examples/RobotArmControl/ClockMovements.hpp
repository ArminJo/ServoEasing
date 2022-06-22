/*
 * ClockMovements.hpp
 *
 * Contains all the clock related functions.
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef _CLOCK_MOVEMENTS_HPP
#define _CLOCK_MOVEMENTS_HPP

#include <Arduino.h>
#include "RobotArmIRCommands.h"
#include "IRCommandDispatcher.h"

#include "ClockMovements.h"
#include "RobotArmServoControl.h"

#if defined(INFO) && !defined(LOCAL_INFO)
#define LOCAL_INFO
#else
//#define LOCAL_INFO // This enables info output only for this file
#endif
#if defined(LOCAL_INFO)
#define CLOCK_INFO_PRINT(...)    Serial.print(__VA_ARGS__)
#define CLOCK_INFO_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
#define CLOCK_INFO_PRINT(...) void()
#define CLOCK_INFO_PRINTLN(...) void()
#endif

uint8_t sOldHour;
uint8_t sOldMinute;

/******************************************
 * The Commands to execute
 ******************************************/
void lift() {
    goToPositionRelative(0, 0, LIFT_HEIGHT);
}

void lower() {
    goToPositionRelative(0, 0, - LIFT_HEIGHT);
}

void liftPen() {
    CLOCK_INFO_PRINTLN(F("Lift pen"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, CLOCK_DIGITS_Z + PEN_GRIP_OFFSET + LIFT_HEIGHT);
}

void lowerPen() {
    CLOCK_INFO_PRINTLN(F("Lower pen"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, CLOCK_DIGITS_Z + PEN_GRIP_OFFSET);
}

void doDrawNumberOutline() {

}

void doDrawTime() {
    uint_fast8_t tSeconds = (millis() / 1000) % 60;
    uint_fast8_t tMinutes = (millis() / 60000) % 24;
    doGetPen();
    drawTime(tMinutes, tSeconds, false);
    doStorePen();
}

/*
 * Draw only if time changed or parameter aForceDrawing is true
 */
void drawTime(uint_fast8_t aHour, uint_fast8_t aMinute, bool aForceDrawing) {
    if (sOldHour != aHour || sOldMinute != aMinute || aForceDrawing) {
        CLOCK_INFO_PRINT(F("Hour="));
        CLOCK_INFO_PRINT(aHour);
        CLOCK_INFO_PRINT(F(" Minute="));
        CLOCK_INFO_PRINTLN(aMinute);

        /*
         * Hours
         */
        if (sOldHour != aHour || aForceDrawing) {
            sOldHour = aHour;
            CLOCK_INFO_PRINTLN(F("Draw (changed) hours"));

            uint8_t tHourOnes = aHour % 10;
            if (tHourOnes == 0 || aForceDrawing) {
                drawNumber(DIGIT_HOUR_TENS, aHour / 10);
                delayAndCheckForRobotArm(1000);
            }
            drawNumber(DIGIT_HOUR_ONES, tHourOnes);
            delayAndCheckForRobotArm(1000);
        }

        /*
         * Minutes
         */
        if (sOldMinute != aMinute || aForceDrawing) {
            if (sOldMinute / 10 != aMinute / 10 || aForceDrawing) {
                CLOCK_INFO_PRINTLN(F("Draw (changed) tens of minutes"));
                drawNumber(DIGIT_MINUTES_TENS, aMinute / 10);
                delayAndCheckForRobotArm(1000);
            }
            CLOCK_INFO_PRINTLN(F("Draw (changed) ones of minutes"));
            drawNumber(DIGIT_MINUTES_ONES, aMinute % 10);

            sOldMinute = aMinute;
        }
        delayAndCheckForRobotArm(1000);
    }
}

#if defined(ROBOT_ARM_HAS_RTC_CONTROL)
/*
 * Check time every 10 seconds
 */
bool sDrawTimeJustStarted = false;
void checkTimeAndDraw(uRTCLib *aRTC_DS3231) {
    static long sLastMillisOfRTCCheck;
    if (millis() - sLastMillisOfRTCCheck >= 5000 || sDrawTimeJustStarted) {
        if (sDrawTimeJustStarted) {
            doGetPenSpecial();
        } else if (IRDispatcher.IRReceivedData.MillisOfLastCode != 0) {
            // stop time drawing mode, but not at the first call
            sActionType = ACTION_TYPE_STOP;
            doStorePen();
            return;
        }
        // clear flag in order to detect next IR command which stops time drawing mode
        IRDispatcher.IRReceivedData.MillisOfLastCode = 0;

        sLastMillisOfRTCCheck = millis();
        aRTC_DS3231->refresh();
        uint8_t tNewHour = aRTC_DS3231->hour();
        uint8_t tNewMinute = aRTC_DS3231->minute();
        if (sOldMinute != tNewMinute || sDrawTimeJustStarted) {
            sOldMinute = tNewMinute;
            /*
             * Minute has changed -> draw new time
             */
            CLOCK_INFO_PRINTLN(F("Minute has changed -> Start with go neutral"));
            delayAndCheckForRobotArm(5000);
            CLOCK_INFO_PRINTLN(F("Move up and wait"));
            goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH);
            delayAndCheckForRobotArm(1000);

            drawTime(tNewHour, tNewMinute, sDrawTimeJustStarted);

            //    CLOCK_INFO_PRINTLN(F("Go to pen"));
            goToPosition(PEN_POSITION_X, PEN_POSITION_Y, KEEP_POSITION);

            sDrawTimeJustStarted = false;
        }
    }
}
#endif

void doStartClock() {
#if defined(ROBOT_ARM_HAS_RTC_CONTROL)
    sDrawTimeJustStarted = true;
#endif
    sActionType = ACTION_TYPE_DRAW_TIME;
    IRDispatcher.IRReceivedData.MillisOfLastCode = 0;
}

void doSetModeForClockMovement() {
//    sInverseKinematicModeForCombinedMovementsIsActive = true;
//    setEasingTypeForAllServos(EASE_USER_DIRECT);
//    ClawServo.setEasingType(EASE_LINEAR);
    sInverseKinematicModeForCombinedMovementsIsActive = false;
    setEasingTypeForAllServos(EASE_LINEAR);
}

void doGetPen() {
    doSetModeForClockMovement();

    CLOCK_INFO_PRINTLN(F("Get pen: move up"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH + CLAW_HEIGHT);
    CLOCK_INFO_PRINTLN(F("Go to pen and open claw"));
    goToPosition(PEN_POSITION_X, PEN_POSITION_Y, KEEP_POSITION);
    openClaw();
    CLOCK_INFO_PRINTLN(F("Move down and close claw"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z);
    // Get pen
    closeClaw();
    CLOCK_INFO_PRINTLN(F("Move up again"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH);
}

/*
 * No terminating move up
 */
void doGetPenSpecial() {
    doSetModeForClockMovement();

    CLOCK_INFO_PRINTLN(F("Get pen: move up"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH + CLAW_HEIGHT);
    CLOCK_INFO_PRINTLN(F("Go to pen and open claw"));
    goToPosition(PEN_POSITION_X, PEN_POSITION_Y, KEEP_POSITION);
    openClaw();
    CLOCK_INFO_PRINTLN(F("Move down and close claw"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z);
    // Get pen
    closeClaw();
}

void doStorePen() {
    doSetModeForClockMovement();

    CLOCK_INFO_PRINTLN(F("Store pen: move up"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH);
    CLOCK_INFO_PRINTLN(F("Go to pen"));
    goToPosition(PEN_POSITION_X, PEN_POSITION_Y, KEEP_POSITION);
    CLOCK_INFO_PRINTLN(F("Move down and open claw"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z);
    // Release pen
    openClaw();
    CLOCK_INFO_PRINTLN(F("Move up again"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH + CLAW_HEIGHT);
}

void doGetEraser() {
    doSetModeForClockMovement();

    CLOCK_INFO_PRINTLN(F("Get eraser: move up"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, ERASER_POSITION_Z + ERASER_LENGTH + CLAW_HEIGHT);
    CLOCK_INFO_PRINTLN(F("Go to eraser"));
    goToPosition(ERASER_POSITION_X, ERASER_POSITION_Y, KEEP_POSITION);
    openClaw();
    CLOCK_INFO_PRINTLN(F("Move down"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, ERASER_POSITION_Z + ERASER_GRIP_Z);
    // Get eraser
    closeClaw();
    CLOCK_INFO_PRINTLN(F("Lift eraser"));
    goToPositionRelative(0, 0, LIFT_HEIGHT);
}

void doStoreEraser() {

}

void doErase(uint8_t aDigitPosition) {

}

/*
 * Assumes Pen at upper position over origin of number
 */
void drawNumber(uint8_t aNumber) {
    switch (aNumber) {
    case 0:
        // draw clockwise
        lowerPen();
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT, 0);
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative(0, -CLOCK_DIGIT_HEIGHT, 0);
        goToPositionRelative(-CLOCK_DIGIT_WIDTH, 0, 0);
        break;
    case 1:
        goToPositionRelative(0, (CLOCK_DIGIT_HEIGHT / 3) * 2, 0);
        lowerPen();
        goToPositionRelative(CLOCK_DIGIT_WIDTH, CLOCK_DIGIT_HEIGHT - ((CLOCK_DIGIT_HEIGHT / 3) * 2), 0);
        goToPositionRelative(0, -CLOCK_DIGIT_HEIGHT, 0);
        break;
    case 2:
        liftPen();
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT, 0);
        lowerPen();
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative(0, -CLOCK_DIGIT_HEIGHT / 2, 0);
        goToPositionRelative(-CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative(0, -CLOCK_DIGIT_HEIGHT / 2, 0);
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        break;
    case 3:
        liftPen();
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT, 0);
        lowerPen();
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative(0, -CLOCK_DIGIT_HEIGHT, 0);
        goToPositionRelative(-CLOCK_DIGIT_WIDTH, 0, 0);
        liftPen();
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT / 2, 0);
        lowerPen();
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        break;
    case 4:
        liftPen();
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT, 0);
        lowerPen();
        goToPositionRelative(0, -(CLOCK_DIGIT_HEIGHT / 2), 0);
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        liftPen();
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT / 2, 0);
        lowerPen();
        goToPositionRelative(0, -CLOCK_DIGIT_HEIGHT, 0);
        break;
    case 5:
        liftPen();
        goToPositionRelative(CLOCK_DIGIT_WIDTH, CLOCK_DIGIT_HEIGHT, 0);
        lowerPen();
        goToPositionRelative(-CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative(0, -CLOCK_DIGIT_HEIGHT / 2, 0);
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative(0, -CLOCK_DIGIT_HEIGHT / 2, 0);
        goToPositionRelative(-CLOCK_DIGIT_WIDTH, 0, 0);
        break;
    case 6:
        liftPen();
        goToPositionRelative(CLOCK_DIGIT_WIDTH, CLOCK_DIGIT_HEIGHT, 0);
        lowerPen();
        goToPositionRelative(-CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative(0, -CLOCK_DIGIT_HEIGHT, 0);
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT / 2, 0);
        goToPositionRelative(-CLOCK_DIGIT_WIDTH, 0, 0);
        break;
    case 7:
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT, 0);
        lowerPen();
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative((-CLOCK_DIGIT_WIDTH / 2), -CLOCK_DIGIT_HEIGHT, 0);
        break;
    case 8:
        // draw clockwise
        lowerPen();
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT, 0);
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative(0, -CLOCK_DIGIT_HEIGHT, 0);
        goToPositionRelative(-CLOCK_DIGIT_WIDTH, 0, 0);
        liftPen();
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT / 2, 0);
        lowerPen();
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        break;
    case 9:
        liftPen();
        goToPositionRelative(CLOCK_DIGIT_WIDTH, CLOCK_DIGIT_HEIGHT / 2, 0);
        lowerPen();
        goToPositionRelative(-CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT / 2, 0);
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative(0, -CLOCK_DIGIT_HEIGHT, 0);
        goToPositionRelative(-CLOCK_DIGIT_WIDTH, 0, 0);
        break;
    default:
        break;
    }
    liftPen();

}

void drawNumber(uint8_t aDigitPosition, uint8_t aNumber) {

    CLOCK_INFO_PRINT(F("Draw number="));
    CLOCK_INFO_PRINT(aNumber);
    CLOCK_INFO_PRINT(F(" at "));
    CLOCK_INFO_PRINTLN(aDigitPosition);

    switch (aDigitPosition) {
    case DIGIT_MINUTES_ONES:
        // go to lower left position of digit
        goToPosition(CLOCK_MINUTES_ONES_X, CLOCK_DIGITS_Y, KEEP_POSITION);
        break;
    case DIGIT_MINUTES_TENS:
        goToPosition(CLOCK_MINUTES_TENS_X, CLOCK_DIGITS_Y, KEEP_POSITION);
        break;
    case DIGIT_HOUR_ONES:
        goToPosition(CLOCK_HOUR_ONES_X, CLOCK_DIGITS_Y, KEEP_POSITION);
        break;
    case DIGIT_HOUR_TENS:
        goToPosition(CLOCK_HOUR_TENS_X, CLOCK_DIGITS_Y, KEEP_POSITION);
        break;
    default:
        break;
    }
    drawNumber(aNumber);
}

#if defined(LOCAL_INFO)
#undef LOCAL_INFO
#endif
#endif // _CLOCK_MOVEMENTS_HPP
