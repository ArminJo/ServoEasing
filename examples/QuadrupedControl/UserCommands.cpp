/*
 * UserCommands.cpp
 *
 * Put your code here to implement your own functions.
 * If you want to use the default function, just comment / deactivate the function in this file.
 *
 *  Copyright (C) 2019-2021  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of QuadrupedControl https://github.com/ArminJo/QuadrupedControl.
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

// Better define it globally (not possible for the Arduino IDE). For Sloeber, do insert -DUSE_USER_DEFINED_MOVEMENTS in:  Properties / Arduino/ Compile Options / append to C++
//#define USE_USER_DEFINED_MOVEMENTS
#if defined(USE_USER_DEFINED_MOVEMENTS)
#include "QuadrupedControl.h"
#if defined(QUADRUPED_HAS_IR_CONTROL)
#include "IRCommandDispatcher.h"
#endif
#include "QuadrupedServoControl.h"
#include "QuadrupedMovements.h"

#include "Commands.h"

#if defined(QUADRUPED_PLAYS_RTTTL)
#include <PlayRtttl.h>
#endif
#if defined(QUADRUPED_HAS_NEOPIXEL)
#include "QuadrupedNeoPixel.h"
#endif
#if defined(QUADRUPED_HAS_US_DISTANCE)
#include "HCSR04.h"
#endif

/*
 * ---------------------------------
 * QUADRUPED PROGRAMMING CHEAT SHEET
 * ---------------------------------
 *
 * Servos available:
 *  frontLeftPivotServo, frontLeftLiftServo
 *  backLeftPivotServo, backLeftLiftServo
 *  backRightPivotServo, backRightLiftServo
 *  frontRightPivotServo, frontRightLiftServo
 *
 * Useful variables:
 * sBodyHeightAngle contains the normal angle of lift servos.
 * sMovingDirection  controls and contains the current direction. Can be MOVE_DIRECTION_FORWARD, MOVE_DIRECTION_LEFT etc.
 *
 * High level movements:
 * moveCreep(1);
 * centerServos();
 * moveTrot(1);
 * moveTurn(1);
 * doLeanLeft();
 * doLeanRight();
 * basicTwist(30);
 * doWave();
 *
 * Timing:
 * delayAndCheck(1000);
 *
 * To move the front left lift servo, use e.g.:
 * frontLeftLiftServo.easeTo(convertLegPercentHeightToAngle(100));
 *
 * To move multiple servos simultaneously:
 * frontLeftPivotServo.setEaseTo(0);
 * frontLeftLiftServo.setEaseTo(LIFT_MAX_ANGLE);
 * synchronizeAllServosStartAndWaitForAllServosToStop();
 *
 * To move all lift servos simultaneously:
 * setLiftServos(LIFT_LOWEST_ANGLE, LIFT_LOWEST_ANGLE, LIFT_HIGHEST_ANGLE, LIFT_HIGHEST_ANGLE);
 *
 * To move all pivot servos simultaneously:
 * setPivotServos(100, 100, 80, 80);
 *
 * To get the current angle of a servo:
 * getCurrentAngle();
 *
 * Quadrupeds with NeoPixel have the strips:
 * RightNeoPixelBar, FrontNeoPixelBar, LeftNeoPixelBar
 * QuadrupedNeoPixelBar (all bars chained together)
 *
 * The following patterns are available:
 * RainbowCycle(), ColorWipe(), Fade()
 * Stripes(), Heartbeat(), ScannerExtended(), Fire(), Delay()
 *
 */

/*
 * Here you can create your own movements.
 *
 * The speed and height commands on the remote are already active.
 * They modify the variables sServoSpeed and sBodyHeightAngle.
 *
 * I recommend to implement the following movements:
 * 1. A twist forth and back.
 * 2. Bows in different directions.
 * 3. Wave with one leg. Lift up diagonal legs and modify the positions of the other two so that the body tilts in the right direction.
 * 4. Turn. To get the current angle of a servo, use getCurrentAngle().
 *
 */

// doTest() is mapped to the star on the remote
void doTest() {
    sActionType = ACTION_TYPE_TEST;

    tone(PIN_BUZZER, 2000, 400);
    delayAndCheck(400);
    centerServos();

#if defined(QUADRUPED_HAS_NEOPIXEL)
    QuadrupedNeoPixelBar.clear();
    LeftNeoPixelBar.ScannerExtended(COLOR32_CYAN, 3, sServoSpeed / 2, 0, FLAG_SCANNER_EXT_ROCKET | FLAG_SCANNER_EXT_VANISH_COMPLETE,
    DIRECTION_DOWN);
#endif

    /*
     * Here the movement starts
     * Set multiple servos simultaneously
     */
    frontLeftLiftServo.setEaseTo(convertLegPercentHeightToAngle(100)); // 100 is leg at upper limit
    backRightLiftServo.setEaseTo(convertLegPercentHeightToAngle(20)); // 20 is leg low
    synchronizeAllServosStartAndWaitForAllServosToStop();

    frontLeftPivotServo.easeTo(25);
    /*
     * Set back to actual body height angle (which is modified by increase and decrease height commands)
     * For demonstration purpose, this is not done simultaneously.
     */
    backRightLiftServo.easeTo(sBodyHeightAngle);
    frontLeftLiftServo.easeTo(sBodyHeightAngle);

    centerServos();
    sActionType = ACTION_TYPE_STOP;
}

/*
 * These movements are accessible by the remote:
 */

// doTwist() is mapped to the 7 on the remote
void doTwist() {
    sActionType = ACTION_TYPE_TWIST;
    /*
     * Put your own code here
     */

    int8_t tTwistAngle = 30;
    // Move all pivot servos simultaneously
    setPivotServos(90 + tTwistAngle, 90 + tTwistAngle, 90 + tTwistAngle, 90 + tTwistAngle);

    sActionType = ACTION_TYPE_STOP;
}

// doDance() is mapped to the 1 on the remote
void doDance() {
    sActionType = ACTION_TYPE_DANCE;
    doBeep();
    /*
     * Put your own code here
     */
    sActionType = ACTION_TYPE_STOP;
}

// doWave() is mapped to the 3 on the remote
void doWave() {
    sActionType = ACTION_TYPE_WAVE;
    doBeep();
    /*
     * Put your own code here
     */
    sActionType = ACTION_TYPE_STOP;
}

// doQuadrupedAutoMove() is mapped to the 5 on the remote
void doQuadrupedAutoMove() {
    sActionType = ACTION_TYPE_AUTO_MOVE;
    doBeep();
    /*
     * Put your own code here
     */
    sActionType = ACTION_TYPE_STOP;
}

// doTrot() is mapped to the 9 on the remote
void doTrot() {
    sActionType = ACTION_TYPE_TROT;
    doBeep();
    /*
     * Put your own code here
     */
    sActionType = ACTION_TYPE_STOP;
}

// doTurnRight() is mapped to the right arrow on the remote
void doTurnRight() {
    sActionType = ACTION_TYPE_TURN;
    doBeep();
    /*
     * Put your own code here
     */
    sActionType = ACTION_TYPE_STOP;
}

// doTurnLeft() is mapped to the left arrow on the remote
void doTurnLeft() {
    sActionType = ACTION_TYPE_TURN;
    doBeep();
    /*
     * Put your own code here
     */
    sActionType = ACTION_TYPE_STOP;
}

// doTurnLeft() is mapped to the up arrow on the remote
void doCreepForward() {
    sActionType = ACTION_TYPE_CREEP;
    doBeep();
    /*
     * Put your own code here
     */
    sActionType = ACTION_TYPE_STOP;
}
#endif
