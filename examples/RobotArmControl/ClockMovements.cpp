/*
 * ClockMovements.cpp
 *
 * Contains all the IR command functions available.
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

#include "ClockMovements.h"
#include "Commands.h"
#include "IRCommandDispatcher.h"
#include "RobotArmServoControl.h"

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
    Serial.println(F("Lift pen"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, CLOCK_DIGITS_Z + PEN_GRIP_LENGTH + LIFT_HEIGHT);
}

void lowerPen() {
    Serial.println(F("Lower pen"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, CLOCK_DIGITS_Z + PEN_GRIP_LENGTH);
}

void doDrawNumberOutline() {

}

void doGetPenSpecial() {
    doSetModeForClockMovement();

//    Serial.println(F("Get pen: move up"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH + CLAW_HEIGHT);
//    Serial.println(F("Go to pen"));
    goToPosition(PEN_POSITION_X, PEN_POSITION_Y, KEEP_POSITION);
    openClaw();
//    Serial.println(F("Move down"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z);
    // Get pen
    closeClaw();

}

void doClockTest() {
    Serial.println(F("Go neutral"));
    goToNeutral();

    doGetPenSpecial();
    delayAndCheckIRInput(5000);

    //    Serial.println(F("Move up again"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH);
    delayAndCheckIRInput(1000);

    Serial.println(F("Hours tens"));
    drawNumber(DIGIT_HOUR_TENS, 1);
    delayAndCheckIRInput(3000);

    Serial.println(F("Hours ones"));
    drawNumber(DIGIT_HOUR_ONES, 0);
    delayAndCheckIRInput(3000);

    drawNumber(DIGIT_MINUTES_TENS, 4);
    delayAndCheckIRInput(2000);

    Serial.println(F("Minutes ones"));
    drawNumber(DIGIT_MINUTES_ONES, 7);
    delayAndCheckIRInput(2000);

    doStorePen();
}

void doSetModeForClockMovement() {
//    sInverseKinematicModeActive = true;
//    setEasingTypeForAllServos(EASE_USER_DIRECT);
//    ClawServo.setEasingType(EASE_LINEAR);
    sInverseKinematicModeActive = false;
    setEasingTypeForAllServos(EASE_LINEAR);
}

void doGetPen() {
    doSetModeForClockMovement();

//    Serial.println(F("Get pen: move up"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH + CLAW_HEIGHT);
//    Serial.println(F("Go to pen"));
    goToPosition(PEN_POSITION_X, PEN_POSITION_Y, KEEP_POSITION);
    openClaw();
//    Serial.println(F("Move down"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z);
    // Get pen
    closeClaw();
//    Serial.println(F("Move up again"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH);
}

void doStorePen() {
    doSetModeForClockMovement();

//    Serial.println(F("Store pen: move up"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH);
//    Serial.println(F("Go to pen"));
    goToPosition(PEN_POSITION_X, PEN_POSITION_Y, KEEP_POSITION);
//    Serial.println(F("Move down"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z);
    // Release pen
    openClaw();
//    Serial.println(F("Move up again"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, PEN_GRIP_Z + PEN_HOLDER_DEPTH + CLAW_HEIGHT);
}

void doGetEraser() {
    doSetModeForClockMovement();

    //    Serial.println(F("Get eraser: move up"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, ERASER_POSITION_Z + ERASER_LENGTH + CLAW_HEIGHT);
    //    Serial.println(F("Go to eraser"));
    goToPosition(ERASER_POSITION_X, ERASER_POSITION_Y, KEEP_POSITION);
    openClaw();
    //    Serial.println(F("Move down"));
    goToPosition(KEEP_POSITION, KEEP_POSITION, ERASER_POSITION_Z + ERASER_GRIP_Z);
    // Get eraser
    closeClaw();
    //    Serial.println(F("Lift eraser"));
    goToPositionRelative(0, 0, LIFT_HEIGHT);
}

void doStoreEraser() {

}

void doErase(uint8_t aDigitPosition) {

}

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
        lowerPen();

        break;
    case 3:
        lowerPen();

        break;
    case 4:
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
        lowerPen();

        break;
    case 6:
        lowerPen();

        break;
    case 7:
        goToPositionRelative(0, CLOCK_DIGIT_HEIGHT, 0);
        lowerPen();
        goToPositionRelative(CLOCK_DIGIT_WIDTH, 0, 0);
        goToPositionRelative((-CLOCK_DIGIT_WIDTH / 2), -CLOCK_DIGIT_HEIGHT, 0);
        break;
    case 8:
        lowerPen();

        break;
    case 9:
        lowerPen();

        break;
    default:
        break;
    }
    liftPen();

}

void drawNumber(uint8_t aDigitPosition, uint8_t aNumber) {

    Serial.print(F("Draw number="));
    Serial.print(aNumber);
    Serial.print(F(" at "));
    Serial.println(aDigitPosition);

    switch (aDigitPosition) {
    case DIGIT_MINUTES_ONES:
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

/*void drawNumber(float aXOrigin, float aYOrigin, int aNumber, float aScale) {

 switch (aNumber) {

 case 0:
 drawTo(bx + 12 * scale, by + 6 * scale);
 lift(0);
 bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
 lift(1);
 break;
 case 1:

 drawTo(bx + 3 * scale, by + 15 * scale);
 lift(0);
 drawTo(bx + 10 * scale, by + 20 * scale);
 drawTo(bx + 10 * scale, by + 0 * scale);
 lift(1);
 break;
 case 2:
 drawTo(bx + 2 * scale, by + 12 * scale);
 lift(0);
 bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
 drawTo(bx + 1 * scale, by + 0 * scale);
 drawTo(bx + 14 * scale, by + 0 * scale);
 lift(1);
 break;
 case 3:
 drawTo(bx + 2 * scale, by + 17 * scale);
 lift(0);
 bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
 bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
 lift(1);
 break;
 case 4:
 drawTo(bx + 10 * scale, by + 0 * scale);
 lift(0);
 drawTo(bx + 10 * scale, by + 20 * scale);
 drawTo(bx + 2 * scale, by + 6 * scale);
 drawTo(bx + 12 * scale, by + 6 * scale);
 lift(1);
 break;
 case 5:
 drawTo(bx + 2 * scale, by + 5 * scale);
 lift(0);
 bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
 drawTo(bx + 5 * scale, by + 20 * scale);
 drawTo(bx + 12 * scale, by + 20 * scale);
 lift(1);
 break;
 case 6:
 drawTo(bx + 2 * scale, by + 10 * scale);
 lift(0);
 bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
 drawTo(bx + 11 * scale, by + 20 * scale);
 lift(1);
 break;
 case 7:
 drawTo(bx + 2 * scale, by + 20 * scale);
 lift(0);
 drawTo(bx + 12 * scale, by + 20 * scale);
 drawTo(bx + 2 * scale, by + 0);
 lift(1);
 break;
 case 8:
 drawTo(bx + 5 * scale, by + 10 * scale);
 lift(0);
 bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
 bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
 lift(1);
 break;

 case 9:
 drawTo(bx + 9 * scale, by + 11 * scale);
 lift(0);
 bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
 drawTo(bx + 5 * scale, by + 0);
 lift(1);
 break;

 case 11: // Doppelpunkt
 drawTo(bx + 5 * scale, by + 15 * scale);
 lift(0);
 bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
 lift(1);
 drawTo(bx + 5 * scale, by + 5 * scale);
 lift(0);
 bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
 lift(1);
 break;

 }
 }
 */

