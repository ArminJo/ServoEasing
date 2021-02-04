/*
 * Commands.cpp
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

#include "Commands.h"
#include "RobotArmServoControl.h"

bool sInverseKinematicModeActive = true;

uint8_t sActionType;

/******************************************
 * The Commands to execute
 ******************************************/

void __attribute__((weak)) doCenter() {
    setAllServos(4, PIVOT_NEUTRAL_OFFSET_DEGREE, HORIZONTAL_NEUTRAL_OFFSET_DEGREE, LIFT_NEUTRAL_OFFSET_DEGREE, CLAW_START_ANGLE);
}

void __attribute__((weak)) doFolded() {
    shutdownServos();
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
    }
}

// Allow to go to 82
void __attribute__((weak)) doTurnLeft() {
    if (sBodyPivotAngle <= 90) {
        sBodyPivotAngle += 2;
        BasePivotServo.easeTo(sBodyPivotAngle);
    }
}

void __attribute__((weak)) doLiftUp() {
    if (sLiftServoAngle <= LIFT_MAX_ANGLE - 2) {
        sLiftServoAngle += 2;
        LiftServo.easeTo(sLiftServoAngle);
    }
}

void __attribute__((weak)) doLiftDown() {
    if (sLiftServoAngle > 2) {
        sLiftServoAngle -= 2;
        LiftServo.easeTo(sLiftServoAngle);
    }
}

void __attribute__((weak)) doOpenClaw() {
    if (sClawServoAngle > 2) {
        sClawServoAngle -= 2;
        ClawServo.easeTo(sClawServoAngle);
    }
}

void __attribute__((weak)) doCloseClaw() {
    if (sClawServoAngle <= (CLAW_MAX_ANGLE - 2)) {
        sClawServoAngle += 2;
        ClawServo.easeTo(sClawServoAngle);
    }
}

void __attribute__((weak)) doSwitchToManual() {
#if defined(ROBOT_ARM_IR_CONTROL)
    // this enables manual mode
    IRDispatcher.IRReceivedData.MillisOfLastCode = 0;
#endif
}

/*
 * Switch mode between Inverse-Kinematic and normal easing
 */
void __attribute__((weak)) doInverseKinematicOn() {
    sInverseKinematicModeActive = true;
}

void __attribute__((weak)) doInverseKinematicOff() {
    sInverseKinematicModeActive = false;
}

void __attribute__((weak)) doToggleInverseKinematic() {
#if defined(ROBOT_ARM_IR_CONTROL)
    if (!IRDispatcher.IRReceivedData.isRepeat) {
        sInverseKinematicModeActive = !sInverseKinematicModeActive;
    }
#endif
}

void __attribute__((weak)) doRobotArmAutoMove() {
    Serial.print(F("Start auto move: InverseKinematicModeActive="));
    Serial.println(sInverseKinematicModeActive);

    doSetToAutoModeForRobotArm();

    if (sInverseKinematicModeActive) {
        setEasingTypeForAllServos(EASE_USER_DIRECT);
        ClawServo.setEasingType(EASE_LINEAR);
    }

    // init start position for first move
    sEndPosition.LeftRightDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_BASE_PIVOT];
    sEndPosition.BackFrontDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_HORIZONTAL];
    sEndPosition.DownUpDegree = ServoEasing::ServoEasingNextPositionArray[SERVO_LIFT];
    unsolve(&sEndPosition);

    // goto neutral position
//    goToPosition(0, LIFT_ARM_LENGTH_MILLIMETER + CLAW_LENGTH_MILLIMETER, HORIZONTAL_ARM_LENGTH_MILLIMETER);
    goToPosition(0, 148, 80);
    RETURN_IF_STOP;

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

    // go back to start
    goToPosition(0, 148, 80);
    RETURN_IF_STOP;

    delayAndCheckForRobotArm(2000);
}
/*************************
 * Instant Commands
 *************************/
/*
 * Decrease moving speed by 25%
 */
void __attribute__((weak)) doIncreaseSpeed() {
    sServoSpeed += sServoSpeed / 4;
    if (sServoSpeed > 0xBF) {
        sServoSpeed = 0xBF;
    }
    setSpeedForAllServos(sServoSpeed);
    Serial.print(sServoSpeed);
}

/*
 * Increase moving speed by 25%
 */
void __attribute__((weak)) doDecreaseSpeed() {
    if (sServoSpeed > 2) {
        sServoSpeed -= sServoSpeed / 4;
        if (sServoSpeed < 4) {
            sServoSpeed = 4;
        }
    }
    setSpeedForAllServos(sServoSpeed);
    Serial.print(sServoSpeed);
}

void __attribute__((weak)) doSwitchEasingType() {
#if defined(ROBOT_ARM_IR_CONTROL)
    if (!sInverseKinematicModeActive && !IRDispatcher.IRReceivedData.isRepeat) {
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
#endif
}
