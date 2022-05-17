/*
 * RobotArmIRCommands.hpp
 *
 * Contains all the IR command functions available.
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

#ifndef _ROBOT_ARM_IR_COMMANDS_HPP
#define _ROBOT_ARM_IR_COMMANDS_HPP

#include <Arduino.h>
#include "RobotArmIRCommands.h"

#include "RobotArmServoConfiguration.h"
#include "RobotArmServoControl.h"

bool sInverseKinematicModeForCombinedMovementsIsActive = true; // If active then easing type is switched to USER for test and automatic move

uint8_t sActionType;

/******************************************
 * The Commands to execute
 ******************************************/

void __attribute__((weak)) doGoCenter() {
    setAllServos(4, 0, 0, 0, 0);
}

void __attribute__((weak)) doGoFolded() {
    goToFolded();
}

void __attribute__((weak)) doGoBack() {
    if (sHorizontalServoAngle > 2) {
        sHorizontalServoAngle -= 2;
        HorizontalServo.easeTo(sHorizontalServoAngle);
    }
}

void __attribute__((weak)) doGoForward() {
    if (sHorizontalServoAngle < 178) {
        sHorizontalServoAngle += 2;
        HorizontalServo.easeTo(sHorizontalServoAngle);
    }
}

// Allow to go to -92
void __attribute__((weak)) doTurnRight() {
    if (sBodyPivotAngle > -90) {
        sBodyPivotAngle -= 2;
        BasePivotServo.easeTo(sBodyPivotAngle);
        Serial.println(sBodyPivotAngle);
    }
}

// Allow to go to 82
void __attribute__((weak)) doTurnLeft() {
    if (sBodyPivotAngle <= 90) {
        sBodyPivotAngle += 2;
        BasePivotServo.easeTo(sBodyPivotAngle);
        Serial.println(sBodyPivotAngle);
    }
}

void __attribute__((weak)) doLiftUp() {
    if (sLiftServoAngle <= LIFT_MAXIMUM_DEGREE - 2) {
        sLiftServoAngle += 2;
        LiftServo.easeTo(sLiftServoAngle);
        Serial.println(sLiftServoAngle);
    }
}

void __attribute__((weak)) doLiftDown() {
    if (sLiftServoAngle > LIFT_MINIMUM_DEGREE + 2) {
        sLiftServoAngle -= 2;
        LiftServo.easeTo(sLiftServoAngle);
        Serial.println(sLiftServoAngle);
    }
}

void __attribute__((weak)) doOpenClaw() {
    if (sClawServoAngle < 90) {
        sClawServoAngle += 2;
        ClawServo.easeTo(sClawServoAngle);
        Serial.println(sClawServoAngle);
    }
}

void __attribute__((weak)) doCloseClaw() {
    if (sClawServoAngle >= 2) {
        sClawServoAngle -= 2;
        ClawServo.easeTo(sClawServoAngle);
        Serial.println(sClawServoAngle);

    }
}

void __attribute__((weak)) doSwitchToManual() {
#if defined(ROBOT_ARM_HAS_IR_CONTROL)
    // this enables manual mode
    IRDispatcher.IRReceivedData.MillisOfLastCode = 0;
#endif
}

/*
 * Switch mode between Inverse-Kinematic and normal easing
 */
void __attribute__((weak)) doInverseKinematicOn() {
    sInverseKinematicModeForCombinedMovementsIsActive = true;
}

void __attribute__((weak)) doInverseKinematicOff() {
    sInverseKinematicModeForCombinedMovementsIsActive = false;
}

void __attribute__((weak)) doToggleInverseKinematic() {
#if defined(ROBOT_ARM_HAS_IR_CONTROL)
    if (!IRDispatcher.IRReceivedData.isRepeat) {
        sInverseKinematicModeForCombinedMovementsIsActive = !sInverseKinematicModeForCombinedMovementsIsActive;
    }
#endif
}

void __attribute__((weak)) doSwitchEasingType() {

    Serial.print(F("Set easing type to "));
    if (sEasingType == EASE_LINEAR) {
        sEasingType = EASE_QUADRATIC_IN_OUT;
    } else if (sEasingType == EASE_QUADRATIC_IN_OUT) {
        sEasingType = EASE_CIRCULAR_IN_OUT;
    } else if (sEasingType == EASE_CIRCULAR_IN_OUT) {
        sEasingType = EASE_QUARTIC_IN_OUT;
    } else if (sEasingType == EASE_QUARTIC_IN_OUT) {
        sEasingType = EASE_BOUNCE_OUT;
    } else if (sEasingType == EASE_BOUNCE_OUT) {
        sEasingType = EASE_LINEAR;
    }
    setEasingTypeForMultipleServos(3, sEasingType); // do not change type for claw
    ServoEasing::printEasingType(&Serial, sEasingType);
    Serial.println();
}

/*
 * Can be tested with IK (USER Easing) or with standard easings
 */
void __attribute__((weak)) doRobotArmTestMove() {
//    testInverseAndForwardKinematic(); // just print outputs

    if (sInverseKinematicModeForCombinedMovementsIsActive) {
        setEasingTypeForMultipleServos(3, EASE_USER_DIRECT); // set to IK mode but do not change easing type for claw
    }

    Serial.print(F("Start test move: EasingType="));
    ServoEasing::printEasingType(&Serial, BasePivotServo.mEasingType); // print actual easing type of servo
    Serial.println();

// init start position for first move
    sEndPosition.LeftRight = 0;
    sEndPosition.BackFront = HORIZONTAL_NEUTRAL_MILLIMETER; // 148;
    sEndPosition.DownUp = VERTICAL_NEUTRAL_MILLIMETER; // 80
// go to start position
    goToPositionRelative(0, 0, 0);

    Serial.println(F("Go forward, backward"));
    goToPositionRelative(0, 50, 0);
    goToPositionRelative(0, -100, 0);   // here only BackFront is moving if not IK
    goToPositionRelative(0, 50, 0);

    Serial.println(F("Go up, down"));
    goToPositionRelative(0, 0, 80);
    goToPosition(0, KEEP_POSITION, MINIMUM_HEIGHT_MILLIMETER + 5);

    Serial.println(F("Go bottom line"));
    goToPosition(0, 120, KEEP_POSITION);
    goToPosition(0, 195, KEEP_POSITION);
    goToPosition(0, 120, KEEP_POSITION);

    Serial.println(F("Go home"));
    goToPosition(0, HORIZONTAL_NEUTRAL_MILLIMETER, VERTICAL_NEUTRAL_MILLIMETER);

    Serial.println(F("Go right, left"));
    goToPosition(100, KEEP_POSITION, KEEP_POSITION);
    goToPosition(-100, KEEP_POSITION, KEEP_POSITION); // here only LeftRight is moving if not IK
    goToPosition(0, KEEP_POSITION, KEEP_POSITION);

    if (sInverseKinematicModeForCombinedMovementsIsActive) {
        setEasingTypeForMultipleServos(3, sEasingType); // set back to non IK mode
    }
}

void __attribute__((weak)) doRobotArmAutoMove() {

    if (sInverseKinematicModeForCombinedMovementsIsActive) {
        setEasingTypeForMultipleServos(3, EASE_USER_DIRECT); // set to IK mode but do not change easing type for claw
    }
    Serial.print(F("Start auto move: EasingType="));
    ServoEasing::printEasingType(&Serial, BasePivotServo.mEasingType); // print actual easing type of servo
    Serial.println();

    // init start position for first move
    sEndPosition.LeftRightDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_BASE_PIVOT];
    sEndPosition.BackFrontDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_HORIZONTAL];
    sEndPosition.DownUpDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_LIFT];
    doForwardKinematics(&sEndPosition);

    // goto neutral position
    goToPosition(0, LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER, HORIZONTAL_ARM_LENGTH_MILLIMETER); // 0, 148, 80 left/right, back/front, up/down
    RETURN_IF_STOP;

    // Open claw
    ClawServo.easeTo(45);

    // go down and close claw
    goToPosition(KEEP_POSITION, KEEP_POSITION, MINIMUM_HEIGHT_MILLIMETER);
    RETURN_IF_STOP;
    ClawServo.easeTo(0);

    // move up a bit
    goToPositionRelative(0, 0, 20);
    RETURN_IF_STOP;

    // turn right move up, and open claw
    goToPosition(120, KEEP_POSITION, 100); // left/right, back/front, up/down
    RETURN_IF_STOP;

    ClawServo.easeTo(45);

    // turn left, go back and down
    goToPosition(-120, 0, 40);
    RETURN_IF_STOP;

    ClawServo.easeTo(0);

    // go back to start
    goToPosition(0, LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER, HORIZONTAL_ARM_LENGTH_MILLIMETER); // 0, 148, 80 left/right, back/front, up/down
    DELAY_AND_RETURN_IF_STOP(200);

    if (sInverseKinematicModeForCombinedMovementsIsActive) {
        setEasingTypeForMultipleServos(3, sEasingType); // set back to non IK mode
    }
}
/*************************
 * Instant Commands
 *************************/
/*
 * Decrease moving speed by 25%
 */
void __attribute__((weak)) doIncreaseSpeed() {
    sRobotArmServoSpeed += sRobotArmServoSpeed / 4;
    if (sRobotArmServoSpeed > 0xBF) {
        sRobotArmServoSpeed = 0xBF;
    }
    setSpeedForAllServos(sRobotArmServoSpeed);
    Serial.println(sRobotArmServoSpeed);
}

/*
 * Increase moving speed by 25%
 */
void __attribute__((weak)) doDecreaseSpeed() {
    sRobotArmServoSpeed -= sRobotArmServoSpeed / 4;
    if (sRobotArmServoSpeed < 4) {
        sRobotArmServoSpeed = 4;
    }
    setSpeedForAllServos(sRobotArmServoSpeed);
    Serial.println(sRobotArmServoSpeed);
}

#endif // _ROBOT_ARM_IR_COMMANDS_HPP
