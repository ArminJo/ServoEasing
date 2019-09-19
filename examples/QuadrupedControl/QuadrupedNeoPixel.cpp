/*
 * QuadrupedNeoPixel.cpp
 *
 * This file mainly contains the control of the attached 3 NeoPixel 8 pixel bars.
 * These 3 bars are chained, in order to need only one pin, resulting in a (electrical) 24 pixel bar.
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

#include "QuadrupedNeoPixel.h"
#include "QuadrupedControl.h"
#include "IRCommandDispatcher.h"
#include "QuadrupedServoControl.h"
#include "QuadrupedMovements.h"  // for sMovingDirection

void QuadrupedPatterns(NeoPatterns * aLedsPtr);

#if defined(NUM_PIXELS)
NeoPatterns QuadrupedNeoPixelBar = NeoPatterns(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800, &QuadrupedPatterns);
NeoPatterns RightNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_RIGHT_BAR, 8, &QuadrupedPatterns);
NeoPatterns FrontNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_FRONT_BAR, 8, &QuadrupedPatterns);
NeoPatterns LeftNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_LEFT_BAR, 8, &QuadrupedPatterns);
#endif

void initNeoPatterns() {
    QuadrupedNeoPixelBar.begin(); // This sets the output pin.
    RightNeoPixelBar.ColorWipe(COLOR32_GREEN_QUARTER, sServoSpeed);
    FrontNeoPixelBar.ScannerExtended(COLOR32_BLUE_HALF, 2, sServoSpeed, 2,
    FLAG_SCANNER_EXT_ROCKET | FLAG_SCANNER_EXT_START_AT_BOTH_ENDS);
    LeftNeoPixelBar.ColorWipe(COLOR32_RED_QUARTER, sServoSpeed, 0, DIRECTION_DOWN);
}

/*
 * This let the Servo ISR look if a new pattern is applicable
 */
void triggerNeoPatterns() {
    sJustCalledMainCommand = true; // To trigger NeoPatterns generation
}

/*
 * The servo ISR extended with NeoPixel handling.
 *
 * Update all servos from list and check if all servos have stopped.
 * Can not call yield() here, since we are in an ISR context here.
 */
void handleServoTimerInterrupt() {
#if defined(USE_PCA9685_SERVO_EXPANDER)
    // Otherwise it will hang forever in I2C transfer
    sei();
#endif
    if (TCCR1B & _BV(ICNC1)) {
        // interrupt start flag was set -> call update
        if (updateAllServos()) {
            // do not disable interrupt but reset flag
            // disableServoEasingInterrupt();
            TCCR1B &= ~_BV(ICNC1);    // reset flag
            Serial.println(F("Do not disable interrupt, reset only the interrupt flag"));
        }
    }

    /*
     * NeoPixel handling.
     */
    if (sJustCalledMainCommand) {
        sJustCalledMainCommand = false;
        handleMovementPattern();
    }

    bool tNeedShow = false;
    if (RightNeoPixelBar.ActivePattern != PATTERN_NONE) {
        // Use false, since PIXEL_OFFSET_RIGHT_BAR is 0 to avoid unnecessary call of show() for the first 8 pixels.
        tNeedShow |= RightNeoPixelBar.update(false);
    }
    if (FrontNeoPixelBar.ActivePattern != PATTERN_NONE) {
        tNeedShow |= FrontNeoPixelBar.update();
    }
    if (LeftNeoPixelBar.ActivePattern != PATTERN_NONE) {
        tNeedShow |= LeftNeoPixelBar.update();
    }
    if (QuadrupedNeoPixelBar.ActivePattern != PATTERN_NONE) {
        // One pattern for all 3 bars
        QuadrupedNeoPixelBar.update();
    } else if (tNeedShow) {
        QuadrupedNeoPixelBar.show();
    }
}

/*
 * Must be called if pattern has ended or must be changed
 */
void handleMovementPattern() {
    Serial.print(F("Action="));
    Serial.println(sActionType);
    if (sActionType != ACTION_TYPE_STOP) {
        /*
         * Action ongoing. Start or restart pattern according to sActionType and other parameter.
         */
        if (sActionType == ACTION_TYPE_CREEP) {
            Serial.println(F("Starting ColorWipe"));
            RightNeoPixelBar.ColorWipe(Adafruit_NeoPixel::Color(0, NeoPixel::gamma5(sBodyHeight), 0), sServoSpeed * 4, 0,
                    sMovingDirection);
            LeftNeoPixelBar.ColorWipe(Adafruit_NeoPixel::Color(NeoPixel::gamma5(sBodyHeight), 0, 0), sServoSpeed * 4, 0,
                    (sMovingDirection + MOVE_DIRECTION_BACKWARD) & MOVE_DIRECTION_MASK);
        } else if (sActionType == ACTION_TYPE_TURN) {
            Serial.println(F("Starting Stripes"));
            QuadrupedNeoPixelBar.Stripes(COLOR32_RED_HALF, 2, COLOR32_GREEN_HALF, 2, sServoSpeed, 100, 0, sMovingDirection);
        } else {
            Serial.println(F("Start nothing"));
        }
    }
}

/*
 * The completion callback for each pattern
 */
void QuadrupedPatterns(NeoPatterns * aLedsPtr) {
    Serial.print(F("Pattern="));
    Serial.print(aLedsPtr->ActivePattern);
    Serial.println(F(" finished"));

    aLedsPtr->ActivePattern = PATTERN_NONE;
    handleMovementPattern();
}
