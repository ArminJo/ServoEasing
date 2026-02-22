/*
 * QuadrupedNeoPixel.hpp
 *
 * This file mainly contains the control of the attached 3 NeoPixel 8 pixel bars.
 * These 3 bars are chained, in order to use only one pin, and are electrically one 24 pixel bar.
 *
 * The NeopPixel updates are synchronized with the ServoEasing updates by overwriting the ServoEasing
 * function handleServoTimerInterrupt() with a function, which calls handleQuadrupedNeoPixelUpdate()
 * in order not to disturb the servo pulse generation.
 *
 * New automatic movement patterns are triggered by (sLastEvaluatedActionForNeopatterns != sCurrentlyRunningAction)
 * which is evaluated in handleQuadrupedNeoPixelUpdate() in ISR context.
 *
 *  Copyright (C) 2019-2026  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of QuadrupedControl https://github.com/ArminJo/QuadrupedControl.
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  QuadrupedControl and ServoEasing are free software: you can redistribute it and/or modify
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

#ifndef _QUADRUPED_NEOPIXEL_HPP
#define _QUADRUPED_NEOPIXEL_HPP

#include <Arduino.h>

#include "QuadrupedNeoPixel.h"
#include "QuadrupedControlCommands.h"
#include <NeoPatterns.hpp>

#if defined(QUADRUPED_HAS_IR_CONTROL)
#include "IRCommandDispatcher.h"
#include "TinyIR.h" // for isTinyReceiverIdle()
#endif

#if defined(QUADRUPED_ENABLE_RTTTL)
#define SUPPRESS_HPP_WARNING
#include <PlayRtttl.h>
#endif

#include "QuadrupedServoControl.h"
#include "QuadrupedBasicMovements.h"  // for sMovingDirection

// This block must be located after the includes of other *.hpp files
//#define LOCAL_INFO  // This enables info output only for this file
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#include "LocalDebugLevelStart.h"

void QuadrupedOnPatternCompleteHandler(NeoPatterns *aLedsPtr);
volatile bool sAtLeastOnePatternsIsActive; // True if at least one pattern is active => call update()
bool sCleanPatternAfterEnd; // Do a wipe out after pattern ended
bool sShowPatternSynchronizedWithServos; // Flag set e.g. by main loop to force to show the pattern (e.g.distance value) synchronized with servo interrupts.
uint8_t sLastEvaluatedActionForNeopatterns; // do determine changes of sCurrentlyRunningAction

NeoPatterns QuadrupedNeoPixelBar = NeoPatterns(NUM_PIXELS, PIN_NEOPIXEL_TRACK, NEO_GRB + NEO_KHZ800,
        &QuadrupedOnPatternCompleteHandler, SHOW_ONLY_AT_PATTERN_UPDATE);
// false -> do not allow show() on partial NeoPixel bar, true
NeoPatterns RightNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_RIGHT_BAR, PIXELS_ON_ONE_BAR,
DISABLE_CALLING_SHOW_OF_PARENT, &QuadrupedOnPatternCompleteHandler, SHOW_ONLY_AT_PATTERN_UPDATE);
NeoPatterns FrontNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_FRONT_BAR, PIXELS_ON_ONE_BAR,
DISABLE_CALLING_SHOW_OF_PARENT, &QuadrupedOnPatternCompleteHandler, SHOW_ONLY_AT_PATTERN_UPDATE);
NeoPatterns LeftNeoPixelBar = NeoPatterns(&QuadrupedNeoPixelBar, PIXEL_OFFSET_LEFT_BAR, PIXELS_ON_ONE_BAR,
DISABLE_CALLING_SHOW_OF_PARENT, &QuadrupedOnPatternCompleteHandler, SHOW_ONLY_AT_PATTERN_UPDATE);

// The color background for front distance bar
color32_t sBarBackgroundColorArrayForDistance[PIXELS_ON_ONE_BAR] = { COLOR32_RED_QUARTER, COLOR32_RED_QUARTER, COLOR32_RED_QUARTER,
COLOR32_YELLOW, COLOR32_YELLOW, COLOR32_GREEN_QUARTER, COLOR32_GREEN_QUARTER, COLOR32_GREEN_QUARTER };

/*
 * @brief This function checks all patterns for update and calls show() of the parent 24 pixel bar if required.
 * It is called in ISR context by handleServoTimerInterrupt() since the show() function blocks interrupts
 * and must therefore be synchronized with the servo pulse generation.
 */
void handleQuadrupedNeoPixelUpdate() {
#if defined(QUADRUPED_HAS_IR_CONTROL)
    // do not disturb started receive of an IR command
    if (isTinyReceiverIdle())
#endif
    {
        /*
         * Update patterns if active, do not call child patterns if parent pattern for 24 pixel is active
         */
        if (sAtLeastOnePatternsIsActive
#if defined(QUADRUPED_HAS_US_DISTANCE)
                && !sUSDistanceMeasurmentIsRunning
#endif
        ) {
            sAtLeastOnePatternsIsActive = QuadrupedNeoPixelBar.updateAndShowAlsoAllChildPatterns(false);
        }

        /*
         * Check for changing of action to trigger new patterns
         * For combined actions the pattern is determined by the current sub/real action/move
         */
        if (sLastEvaluatedActionForNeopatterns != sCurrentlyRunningAction) {
#if defined(LOCAL_DEBUG)
            Serial.print(F("NP last action="));
            Serial.print(sLastEvaluatedActionForNeopatterns);
            Serial.print(' ');
#endif
            sLastEvaluatedActionForNeopatterns = sCurrentlyRunningAction;
            handleAutomaticMovementPattern(); // To trigger NeoPatterns generation
        }

        /*
         * Check if main loop requires pattern display for e.g. showing distance at front bar
         * This flag is only set if no patterns are running
         */
        if (sShowPatternSynchronizedWithServos) {
            sShowPatternSynchronizedWithServos = false;
            QuadrupedNeoPixelBar.show();
        }
    }
}

/*
 * Must be called if one pattern has ended or movement has changed
 * If we have a distance sensor the front bar always shows the measured distance,
 * except if FrontNeoPixelBar or QuadrupedNeoPixelBar are running patterns.
 * The shortest pattern of the child bars determines the time until next pattern generation if moving action does not change.
 * Is always called in ISR context by handleQuadrupedNeoPixelUpdate() directly or by it calling updateAndShowAlsoAllChildPatterns().
 */
void handleAutomaticMovementPattern() {
#if defined(LOCAL_INFO)
    Serial.print(F("NeoPatterns current action="));
    Serial.print(sCurrentlyRunningAction);
    Serial.print('|');
#endif

    sAtLeastOnePatternsIsActive = true;
    uint16_t tDelayFromSpeed = getDelayFromSpeed();
    QuadrupedNeoPixelBar.stop();
    NeoPatterns *tScannerBar = &FrontNeoPixelBar; // let scanner run on virtual front of quadruped

    if (sCurrentlyRunningAction == ACTION_TYPE_STOP) {
#if defined(LOCAL_INFO)
        Serial.print(F("Stop"));
#endif
        // Run 4 times rainbow pattern and then stop
        RightNeoPixelBar.RainbowCycle(tDelayFromSpeed / 8, DIRECTION_DOWN, 4);
        LeftNeoPixelBar.RainbowCycle(tDelayFromSpeed / 8, DIRECTION_UP, 4);

    } else {
        /*
         * First stop all running patterns
         */
        FrontNeoPixelBar.stop();
        RightNeoPixelBar.stop();
        LeftNeoPixelBar.stop();
        RightNeoPixelBar.clear();
        LeftNeoPixelBar.clear();

        /*
         * Action ongoing. Start pattern according to sCurrentlyRunningAction and other parameter.
         */
        color32_t tColorLeft = COLOR32_BLACK;
        color32_t tColorRight = COLOR32_BLACK;
        uint8_t tGamma5OfHeight = NeoPixel::gamma5(sBodyHeight);
        if (sMovingDirection == MOVE_DIRECTION_FORWARD) {
            tColorRight = Adafruit_NeoPixel::Color(0, tGamma5OfHeight, 0);
            tColorLeft = Adafruit_NeoPixel::Color(tGamma5OfHeight, 0, 0);
        } else if (sMovingDirection == MOVE_DIRECTION_BACKWARD) {
            tColorRight = Adafruit_NeoPixel::Color(tGamma5OfHeight, 0, 0);
            tColorLeft = Adafruit_NeoPixel::Color(0, tGamma5OfHeight, 0);
        }
        switch (sCurrentlyRunningAction) {
        case ACTION_TYPE_CREEP:
#if defined(LOCAL_INFO)
            Serial.print(F("Creep"));
#endif
            if (!(sMovingDirection & MOVE_DIRECTION_SIDE_MASK)) {
                // only for forward and backward movement
                RightNeoPixelBar.ColorWipe(tColorRight, tDelayFromSpeed, 0, sMovingDirection);
                LeftNeoPixelBar.ColorWipe(tColorLeft, tDelayFromSpeed, 0,
                        (sMovingDirection + MOVE_DIRECTION_BACKWARD) & MOVE_DIRECTION_MASK);
            } else if (sMovingDirection == MOVE_DIRECTION_LEFT) {
                tScannerBar = &LeftNeoPixelBar; // let scanner be at virtual front of quadruped
#if !defined(QUADRUPED_HAS_US_DISTANCE)
                FrontNeoPixelBar.clear(); // clear old front pattern
#endif
            } else if (sMovingDirection == MOVE_DIRECTION_RIGHT) {
                tScannerBar = &RightNeoPixelBar;
#if !defined(QUADRUPED_HAS_US_DISTANCE)
                FrontNeoPixelBar.clear(); // clear old front pattern
#endif
            }
            break;

        case ACTION_TYPE_TURN:
#if defined(LOCAL_INFO)
            Serial.print(F("Turn"));
#endif
            QuadrupedNeoPixelBar.Stripes(COLOR32_RED_HALF, 2, COLOR32_GREEN_HALF, 2, 100, tDelayFromSpeed, sMovingDirection);
            break;

        case ACTION_TYPE_TWIST:
#if defined(LOCAL_INFO)
            Serial.print(F("Twist"));
#endif
            QuadrupedNeoPixelBar.Stripes(COLOR32_RED_HALF, 2, COLOR32_GREEN_HALF, 2, 30, tDelayFromSpeed, sMovingDirection);
            break;

        case ACTION_TYPE_TROT:
#if defined(LOCAL_INFO)
            Serial.print(F("Trot"));
#endif
            RightNeoPixelBar.ScannerExtended(tColorRight, 3, tDelayFromSpeed, 0, FLAG_SCANNER_EXT_ROCKET, sMovingDirection);
            LeftNeoPixelBar.ScannerExtended(tColorLeft, 3, tDelayFromSpeed, 0, FLAG_SCANNER_EXT_ROCKET,
                    (sMovingDirection + MOVE_DIRECTION_BACKWARD) & MOVE_DIRECTION_MASK);
            break;

        case ACTION_TYPE_SHUTDOWN:
        case ACTION_TYPE_DOWN_UP:
#if defined(LOCAL_INFO)
            Serial.print(F("Attention / Down and up"));
#endif
            RightNeoPixelBar.Flash(COLOR32_GREEN_HALF, 200, COLOR32_BLACK, 200, 5);
            FrontNeoPixelBar.Flash(COLOR32_BLUE_HALF, 200, COLOR32_BLACK, 200, 5);
            LeftNeoPixelBar.Flash(COLOR32_RED_HALF, 200, COLOR32_BLACK, 200, 5);
            break;

        case ACTION_TYPE_PAUSE:
#if defined(LOCAL_INFO)
            Serial.print(F("Pause"));
#endif
            QuadrupedNeoPixelBar.Heartbeat(COLOR32_BLUE_QUARTER, tDelayFromSpeed / 2, 2, FLAG_DO_NOT_CLEAR);
            break;

        case ACTION_TYPE_TEST:
        case ACTION_TYPE_MELODY:
#if defined(LOCAL_INFO)
            Serial.print(F("Melody"));
#endif
#if !defined(QUADRUPED_HAS_US_DISTANCE)
            QuadrupedNeoPixelBar.Twinkle(COLOR32_SPECIAL, 6, tDelayFromSpeed / 2, 200); // 6 is aAverageNumberOfActivePixel
#else
            RightNeoPixelBar.Twinkle(COLOR32_SPECIAL, 2, tDelayFromSpeed / 2, 200); // 2 is aAverageNumberOfActivePixel
            LeftNeoPixelBar.Twinkle(COLOR32_SPECIAL, 2, tDelayFromSpeed / 2, 200); // 2 is aAverageNumberOfActivePixel
#endif
            break;

        default:
            sAtLeastOnePatternsIsActive = false;
#if defined(LOCAL_INFO)
            Serial.print(F("Not yet implemented"));
#endif
            break;
        }
    }
#if !defined(QUADRUPED_HAS_US_DISTANCE)
    if (QuadrupedNeoPixelBar.ActivePattern == PATTERN_NONE && tScannerBar->ActivePattern == PATTERN_NONE
            && sMovingDirection != MOVE_DIRECTION_BACKWARD) {
        // No scanner for backward movement
        tScannerBar->ScannerExtended(COLOR32_BLUE_HALF, 2, tDelayFromSpeed, 4,
        FLAG_SCANNER_EXT_ROCKET | FLAG_SCANNER_EXT_START_AT_BOTH_ENDS);
    }
#else
    if (tScannerBar != &FrontNeoPixelBar) {
    // run only for sidewards creeps
    tScannerBar->ScannerExtended(COLOR32_BLUE_HALF, 2, tDelayFromSpeed, 4,
    FLAG_SCANNER_EXT_ROCKET | FLAG_SCANNER_EXT_START_AT_BOTH_ENDS);
    }
#endif

#if defined(LOCAL_INFO)
    Serial.print(F(" -> Left:"));
    LeftNeoPixelBar.printPattern(&Serial);
    Serial.print(F("Front:"));
    FrontNeoPixelBar.printPattern(&Serial);
    Serial.print(F("Right:"));
    RightNeoPixelBar.printPattern(&Serial);
    Serial.print(F("All:"));
    QuadrupedNeoPixelBar.printlnPattern(&Serial);
#endif
}

/*
 * The completion callback for each pattern
 */
void QuadrupedOnPatternCompleteHandler(NeoPatterns *aLedsPtr) {
#if defined(LOCAL_DEBUG)
    Serial.print(F("Offset="));
    Serial.print(aLedsPtr->PixelOffset);
    Serial.print(F(" Pattern \""));
    aLedsPtr->printPatternName(aLedsPtr->ActivePattern, &Serial);
    Serial.println(F("\" finished"));
#endif
    // Reset ActivePattern if no new one will be started.
    aLedsPtr->ActivePattern = PATTERN_NONE;

    if (sCurrentlyRunningAction != ACTION_TYPE_STOP) {
        // do not start stop pattern again, after it ends.
        handleAutomaticMovementPattern();
    }
    /*
     * Check for cleanup finished pattern
     */
    if (sCleanPatternAfterEnd && aLedsPtr->ActivePattern == PATTERN_NONE) {
        sCleanPatternAfterEnd = false;
        Serial.println(F("Do wipe out"));
        doWipeOutPatterns();
    }
}

/*
 * @ return 133 for 90 degree per second, which is start speed
 */
uint16_t getDelayFromSpeed() {
    uint16_t tDelay = 12000 / sQuadrupedServoSpeed;
#if defined(LOCAL_DEBUG)
    Serial.print(F("Speed="));
    Serial.print(sQuadrupedServoSpeed);
    Serial.print(F(" Delay="));
    Serial.println(tDelay);
#endif
    return tDelay;
}

#if defined(HAS_ADDITIONAL_REMOTE_COMMANDS)
void doPattern1() {
    RightNeoPixelBar.RainbowCycle(getDelayFromSpeed() / 8);
    LeftNeoPixelBar.RainbowCycle(getDelayFromSpeed() / 8);
    sCleanPatternAfterEnd = true;
    sAtLeastOnePatternsIsActive = true;

    LeftNeoPixelBar.printlnPattern(&Serial);
}

void doPattern2() {
    uint16_t tDelay = getDelayFromSpeed();
    RightNeoPixelBar.Fade(COLOR32_GREEN_QUARTER, COLOR32_RED_QUARTER, 32, tDelay);
    LeftNeoPixelBar.Fade(COLOR32_RED_QUARTER, COLOR32_GREEN_QUARTER, 32, tDelay);
    sAtLeastOnePatternsIsActive = true;

    LeftNeoPixelBar.printlnPattern(&Serial);
}

/*
 * Not used yet
 */
void doPatternStripes() {
    RightNeoPixelBar.Stripes(COLOR32_GREEN_QUARTER, 2, COLOR32_RED_QUARTER, 2, 128, getDelayFromSpeed());
    LeftNeoPixelBar.Stripes(COLOR32_GREEN_QUARTER, 2, COLOR32_RED_QUARTER, 2, 128, getDelayFromSpeed());
    sAtLeastOnePatternsIsActive = true;

    LeftNeoPixelBar.printlnPattern(&Serial);
}

void doPatternHeartbeat() {
    uint16_t tDelay = getDelayFromSpeed();
    RightNeoPixelBar.Heartbeat(COLOR32_GREEN_HALF, tDelay, 2);
    FrontNeoPixelBar.Heartbeat(COLOR32_BLUE_HALF, tDelay, 2);
    LeftNeoPixelBar.Heartbeat(COLOR32_RED_HALF, tDelay, 2);
    sAtLeastOnePatternsIsActive = true;

    LeftNeoPixelBar.printlnPattern(&Serial);
}

void doPatternFire() {
    uint16_t tDelay = getDelayFromSpeed();
    RightNeoPixelBar.Fire(tDelay, 80);
    LeftNeoPixelBar.Fire(tDelay, 80, DIRECTION_DOWN);
    sCleanPatternAfterEnd = true;
    sAtLeastOnePatternsIsActive = true;
    LeftNeoPixelBar.printlnPattern(&Serial);
}
#endif // HAS_ADDITIONAL_REMOTE_COMMANDS

void initNeoPatterns() {
    QuadrupedNeoPixelBar.begin(); // This sets the output pin.
    RightNeoPixelBar.ColorWipe(COLOR32_GREEN_QUARTER, 120);
    FrontNeoPixelBar.ScannerExtended(COLOR32_BLUE_HALF, 2, 120, 2, FLAG_SCANNER_EXT_ROCKET | FLAG_SCANNER_EXT_START_AT_BOTH_ENDS);
    LeftNeoPixelBar.ColorWipe(COLOR32_RED_QUARTER, 120, 0, DIRECTION_DOWN);
    setTimer1InterruptMarginMicros(4000); // To have the last 4 ms (of available 8) of the 20 ms time slot for servo for processing NeoPatterns and PlayRtttl
    sAtLeastOnePatternsIsActive = true; // enable updates for pattern
}

void doWipeOutPatterns() {
    RightNeoPixelBar.ColorWipe(COLOR32_BLACK, getDelayFromSpeed(), FLAG_DO_NOT_CLEAR, DIRECTION_DOWN);
    FrontNeoPixelBar.clear();
    LeftNeoPixelBar.ColorWipe(COLOR32_BLACK, getDelayFromSpeed(), FLAG_DO_NOT_CLEAR);
    sAtLeastOnePatternsIsActive = true; // enable updates for pattern
}

bool isAtLeastOnePatternActive() {
    return (RightNeoPixelBar.ActivePattern != PATTERN_NONE || FrontNeoPixelBar.ActivePattern != PATTERN_NONE
            || LeftNeoPixelBar.ActivePattern != PATTERN_NONE);
}

#include "LocalDebugLevelEnd.h"

#endif // _QUADRUPED_NEOPIXEL_HPP
