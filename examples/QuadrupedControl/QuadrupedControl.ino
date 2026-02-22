/*
 * QuadrupedControl.cpp
 *
 * Program for controlling a mePed Robot V2 with 8 servos. http://www.meped.io/mepedv2
 * Full version also controls IR remote receiver, 3 NeoPixel bars and a HCSR04 US distance module.
 * The IR remote receiver is attached at pin A0. Supported IR remote controls are:
 *          KEYES (the original mePed remote)
 *          KEYES_CLONE (the one with number pad and direction control swapped, which you get when you buy a KEYES at aliexpress).
 *          WM10
 * If you use another than the KEYES_CLONE, you have to select the one you use in QuadrupedConfiguration.h
 *
 * IF IR control enabled, the AutoMove demo starts once after 40 seconds.
 *
 * If QUADRUPED_HAS_IR_CONTROL not defined (no IR control attached) the AutoMove demo starts after 20 seconds and repeats every 4 minutes.
 *
 * 2 Minutes after the last movement, Attention movement is performed and repeated every minute.
 *
 * If US distance sensor and NeoPixel are attached, the distance is shown at the front NeopixelBar.
 * If distance is between MIN and MAX DISTANCE_FOR_WAVE_CM ( 25 - 35 cm) a doWave is performed every MILLIS_BETWEEN_WAVES (10 seconds)
 * If distance is between MIN and MAX DISTANCE_FOR_MELODY_CM ( 5 - 10 cm) a random melody is played.
 *
 * To run this example you need to install the "ServoEasing" library.
 * For full control install also "IRremote", "PlayRtttl", "NeoPatterns" and "Adafruit_NeoPixel" libraries.
 * These libraries can be installed under "Tools -> Manage Libraries..." or "Ctrl+Shift+I".
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

#include <Arduino.h>

#define VERSION_EXAMPLE "3.4"
// 3.4 Many improvements
// 3.3 Extended demo mode
// 3.2 Demo mode added
// 3.1 Moving US Servo and pause + resume added and pattern handling improved
// 3.0 NeoPixel and distance sensor added
// 2.1 auto move
// 2.0 major refactoring
// 1.1 mirror computation at transformAndSetPivotServos and transformOneServoIndex

#include "QuadrupedConfiguration.h" // Contains the feature definitions as well as the pin layout
#include "digitalWriteFast.h"

/*
 * Demo mode:
 * run doTableMove() every minute
 */
#define ENABLE_QUADRUPED_DEMO_MODE // Instead of going up and down to remind that the Quadruped can be switched off, run the 2 1/2 minutes demo for a restricted space / table area.

#if defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
#define VCC_STOP_THRESHOLD_MILLIVOLT 3500   // stop moving if below 3.5 volt. Below 3.7 volt, the US distance sensor does not work :-(
#else
#define VCC_STOP_THRESHOLD_MILLIVOLT 3200   // stop moving if below 3.2 volt.
#endif
#define NUMBER_OF_VOLTAGE_LOW_FOR_SHUTDOWN        5     // If 5 times voltage low, start shutdown
#define MILLIS_BETWEEN_VOLTAGE_MEASUREMENTS     200     // 5 per second
uint8_t sShutdownCount = 0;

#if defined(QUADRUPED_HAS_US_DISTANCE)
#define MILLIS_BETWEEN_DISTANCE_MEASUREMENTS    200     // 5 measurements per second
#define MILLIS_BETWEEN_DISTANCE_MEASUREMENTS_FOR_MELODY 500 // 2 measurements per second to update front bar
#define MIN_DISTANCE_FOR_MELODY_CM               10     // Play melody triggered by distance below 10 cm.
#define MIN_DISTANCE_FOR_WAVE_CM                 25     // Waves triggered by distance between 25 cm and 35 cm.
#define MAX_DISTANCE_FOR_WAVE_CM                 35     // Waves triggered by distance between 25 cm and 35 cm.
#define MAX_DISTANCE_TIMEOUT_CM                 200     // Do not try to measure distances above 200 cm.
#define MILLIS_BETWEEN_WAVES                  10000     // 10 second between waves triggered by distance.
#include "HCSR04.hpp"
bool sUSDistanceMeasurmentIsRunning; // To block running of blocking Neopixels during measurement
void handleUSDistanceSensor();
#endif

#if defined(QUADRUPED_HAS_IR_CONTROL)
#  if defined(INFO) || defined(DEBUG)
#define USE_DISPATCHER_COMMAND_STRINGS  // Activate this if need the printing of command strings. Requires additional 2 bytes RAM for each command mapping. Requires program memory for strings, but saves snprintf() code (1.5k) if INFO or DEBUG is activated, which has no effect if snprintf() is also used in other parts of your program / libraries.
#  endif
// Include the header only IRCommandDispatcher library in the main program
#include "QuadrupedIRCommandMapping.h"  // Must be included before IRCommandDispatcher.hpp to define IR_ADDRESS and IRMapping and string "unknown".
#include "IRCommandDispatcher.hpp"
#define QUADRUPED_MOVEMENT_BREAK_FLAG (IRDispatcher.requestToStopReceived)
#else
#define QUADRUPED_MOVEMENT_BREAK_FLAG false
#endif

#if defined(QUADRUPED_ENABLE_RTTTL)
#define USE_NO_RTX_EXTENSIONS // Disables RTX format definitions `'s'` (style) and `'l'` (loop). Saves up to 332 bytes program memory
#define DELAY_COUNT_FOR_MELODY 3 // 2 * MILLIS_BETWEEN_DISTANCE_MEASUREMENTS = 400 ms
uint8_t sDelayCounterUntilMelody; // used to disable melody on distance glitches. E.g. avoid playing melody if we measure occasionally once a distance below MIN_DISTANCE_FOR_MELODY_CM
#include <PlayRtttl.hpp>
void handleMelody();
#endif

#include "QuadrupedHelper.hpp"          // For checkForLowVoltage() and playShutdownMelody(). Must be after USE_DISPATCHER_COMMAND_STRINGS

#if defined(QUADRUPED_HAS_NEOPIXEL)
#define ENABLE_EXTERNAL_SERVO_TIMER_HANDLER // Evaluated by ServoEasing.hpp
#include "QuadrupedNeoPixel.hpp"
#endif
#include "QuadrupedControlCommands.hpp" // In turn includes ServoEasing. Commands can also be used e.g. in loop().

#include "ADCUtils.hpp" // for getVCCVoltageMillivoltSimple() and printVCCVoltageMillivolt()

#if defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
ServoEasing USServo;
#endif

/*
 * Loop control
 */
#if defined(QUADRUPED_HAS_IR_CONTROL)
#define MILLIS_OF_INACTIVITY_AFTER_BOOT_BEFORE_SWITCH_TO_AUTO_MOVE     40000   // 40 seconds
#else
#define MILLIS_OF_INACTIVITY_AFTER_BOOT_BEFORE_SWITCH_TO_AUTO_MOVE     20000   // 20 seconds
#define MILLIS_OF_INACTIVITY_BETWEEN_AUTO_MOVE             240000   // 4 minutes
uint32_t sMillisOfLastSpecialAction = 0;                            // millis() of last doAttention() or doWave()
#endif
// In demo mode AutoMove is disabled since demo runs earlier
#if defined(ENABLE_QUADRUPED_DEMO_MODE)
#define MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE           60000   // 1 minute before demo move
#else
#define MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE          120000   // 2 Minutes
#endif

void checkAndHandleAutoMove();
void checkAndHandleAttention();
void resetAttentionTimeout();

void setup() {
    pinModeFast(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    initializeAllQuadrupedServos(90); // 90 degree per second
    delay(2000);

    // Just for pre setting channel and reference
    getVCCVoltageMillivoltSimple();

#if defined(QUADRUPED_HAS_US_DISTANCE)
    Serial.println(F("Init US distance sensor"));
    initUSDistancePins(TRIGGER_OUT_PIN, ECHO_IN_PIN);
#endif

#if defined(QUADRUPED_ENABLE_RTTTL)
    playRtttlBlockingPGM(PIN_BUZZER, Short);
#else
    tone(PIN_BUZZER, 2000, 300);
#endif

#if defined(QUADRUPED_HAS_IR_CONTROL)
    IRDispatcher.init();
    Serial.print(F("Listening to IR remote of type "));
    Serial.print(IR_REMOTE_NAME);
    Serial.println(F(" at pin " STR(IR_RECEIVE_PIN)));
#endif

#if defined(QUADRUPED_HAS_NEOPIXEL)
    Serial.println(F("Init NeoPixel"));
    initNeoPatterns();
    enableServoEasingInterrupt(); // This enables the ServoEasing 20 ms interrupt, which we use to synchronize the NeoPixel update with the servo pulse generation.
#endif

#if defined(QUADRUPED_HAS_US_DISTANCE)
    getUSDistanceAsCentimeter(10000); // this seems to suppress the initial low reading of US distance
#endif

//#if defined(QUADRUPED_HAS_NEOPIXEL)
//    while (FrontNeoPixelBar.ActivePattern != PATTERN_NONE) {
//        ; // wait for pattern to end
//    }
//#endif

    Serial.println(F("Start loop"));
}

void loop() {
#if defined(ADC_UTILS_ARE_AVAILABLE)
    /*
     * Check for low voltage
     */
    checkForVCCUnderVoltageAndShutdown();
#endif

    /*
     * Check if all moving actions have set ACTION_TYPE_STOP after finishing
     */
    if (sCurrentlyRunningAction != ACTION_TYPE_STOP && sCurrentlyRunningAction != ACTION_TYPE_MELODY) {
        Serial.print(F("Unexpected action type="));
        Serial.println(sCurrentlyRunningAction);
    }

#if defined(QUADRUPED_HAS_US_DISTANCE)
    /*
     * US distance sensor handling
     * Displays distance on front bar and starts wave or melody at special distances
     */
    handleUSDistanceSensor();
#endif

    /*
     * Play melody if requested
     * Melody is stopped by other actions
     */
#if defined(QUADRUPED_ENABLE_RTTTL)
    handleMelody();
#endif

    /*
     * IR control handling
     * Check for IR commands and execute them. Returns only AFTER finishing of requested blocking movement
     */
#if defined(QUADRUPED_HAS_IR_CONTROL)
    if (IRDispatcher.checkAndRunSuspendedBlockingCommands()) {
        delay(50); // for the voltage to stabilize after the blocking movement
        resetAttentionTimeout();
    }
#endif

    /*
     * Check for auto move, which runs only once 40 seconds after boot if no input was detected.
     * Auto move, needs more space than demo move.
     */
    checkAndHandleAutoMove();

    /*
     * Run demo or get attention that no IR command was received since 2 minutes and quadruped may be switched off.
     * VCC undervoltage is also signaled here.
     */
    checkAndHandleAttention();
}

/*
 * This disables auto move and allows next attention in 2 minutes and next wave in 10 seconds
 */
void resetAttentionTimeout() {
#if defined(QUADRUPED_HAS_IR_CONTROL)
    IRDispatcher.IRReceivedData.MillisOfLastCode = millis();
#else
    sMillisOfLastSpecialAction = millis();
#endif
}

#if defined(QUADRUPED_ENABLE_RTTTL)
void handleMelody() {
    if (isPlayRtttlRunning()) {
        if (!updatePlayRtttl()) {
            // Melody has just stopped - start new timeout period for autorun and attentions
            resetAttentionTimeout();
            setActionToStop();
        } else if (sCurrentlyRunningAction == ACTION_TYPE_MELODY) {
            if (QUADRUPED_MOVEMENT_BREAK_FLAG) {
                // Melody still playing but IR stop request received -> stop melody
                stopPlayRtttl();
                setActionToStop();
            }
        } else {
            // Another action was started -> stop melody, but do not reset sCurrentlyRunningAction
            stopPlayRtttl();
        }
    }
}
#endif // defined(QUADRUPED_ENABLE_RTTTL)

#if defined(QUADRUPED_HAS_US_DISTANCE)
/**
 * Get US distance and display it on front bar and start wave or melody at special distances
 * Can introduce a delay up to 10 ms
 * This function is also called periodically from blocking movements by delayAndCheckByApplication(),
 * so we can check sCurrentlyRunningAction
 */
void handleUSDistanceSensor() {
    static uint8_t sLastBarLength = 0;
    if (isShutDownByUndervoltage) {
        return; // no measurement required
    }

#  if defined(QUADRUPED_ENABLE_RTTTL)
    // Allow only 2 measurements per second when melody is playing
    if (isPlayRtttlRunning() && (millis() - sLastUSDistanceMeasurementMillis) <= MILLIS_BETWEEN_DISTANCE_MEASUREMENTS_FOR_MELODY) {
        return; // otherwise showPatternSynchronizedWithServos() disturbs the melody
    }
#  endif

    sUSDistanceMeasurmentIsRunning = true;
    uint8_t tDistanceMeasurmentResult = getUSDistanceAsCentimeterWithCentimeterTimeoutPeriodicallyAndPrintIfChanged(
    MAX_DISTANCE_TIMEOUT_CM, MILLIS_BETWEEN_DISTANCE_MEASUREMENTS, &Serial);
    sUSDistanceMeasurmentIsRunning = false;
    if (tDistanceMeasurmentResult != HCSR04_DISTANCE_NO_MEASUREMENT) {
        // 200 ms are gone
#  if defined(QUADRUPED_HAS_NEOPIXEL)
        /*
         * Show distance bar if no other pattern is active on front bar
         */
        if (FrontNeoPixelBar.ActivePattern == PATTERN_NONE && QuadrupedNeoPixelBar.ActivePattern == PATTERN_NONE) {
            /*
             * The first 3 pixel represent a distance of each 5 cm
             * The 4. pixel is active if distance is >= 20 cm and < 30 cm
             * The 7. pixel is active if distance is >= 80 cm and < 120 cm
             * The 8. pixel is active if distance is >= 1.2 meter
             */
            uint8_t tBarLength;
            if (sUSDistanceCentimeter < 20) {
                tBarLength = sUSDistanceCentimeter / 5;
            } else if (sUSDistanceCentimeter < 30) {
                tBarLength = 4;
            } else if (sUSDistanceCentimeter < 50) {
                tBarLength = 5;
            } else if (sUSDistanceCentimeter < 80) {
                tBarLength = 6;
            } else if (sUSDistanceCentimeter < 120) {
                tBarLength = 7;
            } else {
                tBarLength = 8;
            }
            if (sLastBarLength != tBarLength) {
                sLastBarLength = tBarLength;
#if defined(LOCAL_INFO)
                Serial.print(F("New distance bar length="));
                Serial.println(tBarLength);
#endif
                FrontNeoPixelBar.drawBarFromColorArray(tBarLength, sBarBackgroundColorArrayForDistance);
                sShowPatternSynchronizedWithServos = true; // To trigger show() in handleQuadrupedNeoPixelUpdate()
            }
        }
#  endif
        if (sCurrentlyRunningAction == ACTION_TYPE_STOP && sCurrentlyRunningCombinedAction == COMBINED_ACTION_TYPE_STOP) {
            /*
             * No action running here, we are called my loop()
             */
#  if defined(QUADRUPED_ENABLE_RTTTL)
            /*
             * Play melody if distance is 2 consecutive times below 11 cm and above 3 cm
             */
            if (sUSDistanceCentimeter > 3 && sUSDistanceCentimeter <= MIN_DISTANCE_FOR_MELODY_CM && !isPlayRtttlRunning()) {
                sDelayCounterUntilMelody++;
                if (sDelayCounterUntilMelody >= DELAY_COUNT_FOR_MELODY) {
                    doRandomMelody();
                    return; // do not check for wave distance!
                }
            } else {
                sDelayCounterUntilMelody = 0;
            }
#  endif
            /*
             * Do wave if distance is between 35 cm and 25 cm
             * Fist wave can be done 10 seconds after boot
             */
            if (sUSDistanceCentimeter >= MIN_DISTANCE_FOR_WAVE_CM && sUSDistanceCentimeter <= MAX_DISTANCE_FOR_WAVE_CM
#if defined(QUADRUPED_HAS_IR_CONTROL)
                    && (millis() - IRDispatcher.IRReceivedData.MillisOfLastCode) > MILLIS_BETWEEN_WAVES
#else
                    && (millis() - sMillisOfLastSpecialAction) > MILLIS_BETWEEN_WAVES
#endif
            ) {
                doWave();
                resetAttentionTimeout();
            }
        }
    }
}
#endif // #if defined(QUADRUPED_HAS_US_DISTANCE)

/*
 * Check for auto move, which runs only once 40 seconds after boot if no input was detected.
 * Auto move, needs more space than demo move.
 */
void checkAndHandleAutoMove() {
    if ((millis() > MILLIS_OF_INACTIVITY_AFTER_BOOT_BEFORE_SWITCH_TO_AUTO_MOVE)
#if defined(ADC_UTILS_ARE_AVAILABLE)
            && !isShutDownByUndervoltage
#endif
#if defined(QUADRUPED_HAS_IR_CONTROL)
            && IRDispatcher.IRReceivedData.MillisOfLastCode == 0 /* Do auto move only if no IR command received */
#else
            && sMillisOfLastSpecialAction == 0
#endif
            && (!isVCCUSBPowered())) {
        /*
         * Only one auto move 40 s after boot was reached and no IR command was received and not powered by USB
         */
        doQuadrupedAutoMove(); // Can be terminated by IR command
        resetAttentionTimeout();
    }
}

/*
 * If no IR command was received since 2 minutes and quadruped may be switched off, run attention or demo.
 * if we have VCC undervoltage signaled do not run movements but do attention by Melody and optional Neopixel.
 */
void checkAndHandleAttention() {
#if defined(QUADRUPED_HAS_IR_CONTROL)
    if (millis() - IRDispatcher.IRReceivedData.MillisOfLastCode > MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE)
#else
if ((millis() - sMillisOfLastSpecialAction) > MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE)
#endif
    {
        if (isShutDownByUndervoltage) {
            playShutdownMelody(); // Do attention for Quadruped switch off because of undervoltage
#if defined(QUADRUPED_HAS_NEOPIXEL)
            // Do not move servos, just light the Neopixel
            sCurrentlyRunningAction = ACTION_TYPE_SHUTDOWN; // signal it to Neopixel handler
            delay(40); // Neopixel handler is called every 20 ms, so it gets our request
            sCurrentlyRunningAction = ACTION_TYPE_STOP; // Do this animation only once now
#endif

        } else {
#  if defined(ENABLE_QUADRUPED_DEMO_MODE)
            doQuadrupedDemoMove(); // Can be terminated by IR command
#  else
            doAttention();
#  endif
            resetAttentionTimeout();
        }
#if defined(ADC_UTILS_ARE_AVAILABLE)
        printVCCVoltageMillivolt(&Serial);
#endif
    }
}
