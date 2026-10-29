/*
 * QuadrupedControl.cpp
 *
 * Program for controlling a mePed Robot V2 with 8 servos. http://www.meped.io/mepedv2
 * Full version also controls IR remote receiver, 3 NeoPixel bars and a HCSR04 US distance module.
 * The IR remote receiver is attached at pin A0. Supported IR remote controls are:
 *          KEYES (the original mePed remote)
 *          KEYES_CLONE (the one with number pad and direction control swapped, which you get when you buy a KEYES at aliexpress).
 *          WM10
 *          Another check card size remote from a DVBT stick
 *          My white DVD remote as example
 * If you use another than the KEYES_CLONE, you have to select the one you use in QuadrupedConfiguration.h
 *
 * If IR control is enabled (i.e., QUADRUPED_HAS_IR_CONTROL is defined), the following behavior applies:
 * - If no IR command has been received, a demo starts after 1 minute and repeats until an IR command is received.
 * - If SHORT_DEMO_MODE_PIN is LOW, various short demo moves are started instead of one longer demo.
 * - 2 minutes after the last IR command, attention movement is performed and repeated every minute.
 *
 * If QUADRUPED_HAS_IR_CONTROL is not defined (no IR control attached) the FeatureMove demo starts after 20 seconds and repeats every 4 minutes.
 *
 *
 * If US distance sensor and NeoPixel are attached, the distance is shown at the front NeopixelBar.
 * If only the first neopixel is on (bar length is 1) (5 - 9 cm) a random melody is played.
 * If (bar length is 4) (20 - 29 cm) a doWave is performed every MILLIS_BETWEEN_WAVES (10 seconds)
 *
 * To run this example you need to install the "ServoEasing" library.
 * For full control install also "IRremote", "PlayRtttl", "NeoPatterns" and "Adafruit_NeoPixel" libraries.
 * These libraries can be installed under "Tools -> Manage Libraries..." or "Ctrl+Shift+I".
 *
 *  Copyright (C) 2019-2026  Armin Joachimsmeyer
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
// 2.1 auto / feature move
// 2.0 major refactoring
// 1.1 mirror computation at transformAndSetPivotServos and transformOneServoIndex

#include "QuadrupedConfiguration.h" // Contains the feature definitions as well as the pin layout
#include "digitalWriteFast.h"

#if defined(QUADRUPED_HAS_US_DISTANCE_SENSOR)
// default is 3400
#define LI_ION_VCC_UNDERVOLTAGE_THRESHOLD_MILLIVOLT 3700   // Below 3.7 volt, the US distance sensor does not work :-(
#endif

#if defined(QUADRUPED_HAS_US_DISTANCE_SENSOR)
#define MILLIS_BETWEEN_DISTANCE_MEASUREMENTS    200     // 5 measurements per second
#define MILLIS_BETWEEN_DISTANCE_MEASUREMENTS_FOR_MELODY 500 // 2 measurements per second to update front bar
#define BAR_LENGTH_FOR_MELODY                     1     // Play melody triggered by distance between 5 and 9 cm.
#define BAR_LENGTH_FOR_WAVE                       4     // Waves triggered by distance between 20 cm and 29 cm.
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
unsigned long MillisOfLastAttention;
#define QUADRUPED_MOVEMENT_BREAK_FLAG false
#endif

#if defined(QUADRUPED_ENABLE_RTTTL)
#define USE_NO_RTX_EXTENSIONS // Disables RTX format definitions `'s'` (style) and `'l'` (loop). Saves up to 332 bytes program memory
#define DELAY_COUNT_FOR_MELODY 4 // 4 consecutive distances in melody range. 4 * MILLIS_BETWEEN_DISTANCE_MEASUREMENTS = 800 ms
uint8_t sDelayCounterUntilMelody; // used to disable melody on distance glitches
#include <PlayRtttl.hpp>
void handleMelody();
#endif

/*
 * Now include the QuadrupedServoControl library
 */
#if defined(QUADRUPED_HAS_NEOPIXEL)
#define ENABLE_EXTERNAL_SERVO_TIMER_HANDLER // Evaluated by ServoEasing.hpp
#endif
#include "QuadrupedServoControl.hpp"

#include "QuadrupedHelper.hpp" // For checkForVCCUnderVoltageAndShutdownPeriodically() and playShutdownMelody(). Must be after USE_DISPATCHER_COMMAND_STRINGS

#if defined(QUADRUPED_HAS_NEOPIXEL)
#  if defined(QUADRUPED_HAS_US_DISTANCE_SENSOR)
uint8_t sLastBarLength = 0; // For updates only on changes
#  endif
#include "QuadrupedNeoPixel.hpp"
#endif

#include "ADCUtils.hpp" // for getVCCVoltageMillivoltSimple() and printVCCVoltageMillivolt()

#if defined(QUADRUPED_HAS_US_DISTANCE_SENSOR_SERVO)
ServoEasing USServo;
#endif

/*
 * Loop control
 */
#if defined(QUADRUPED_HAS_IR_CONTROL)
#define MILLIS_BETWEEN_DEMO_MOVES                           60000   // 1 minute
void checkAndHandleDemoMove();
unsigned long sMillisOfNextDemo = MILLIS_BETWEEN_DEMO_MOVES;

#define MILLIS_OF_INACTIVITY_BEFORE_ATTENTION_MOVE          120000   // 2 minutes before first attention move
void checkAndHandleAttention();
#else
#define MILLIS_OF_INACTIVITY_AFTER_BOOT_BEFORE_SWITCH_TO_FEATURE_MOVE    20000   // 20 seconds
#define MILLIS_OF_INACTIVITY_BETWEEN_FEATURE_MOVE                       240000   // 4 minutes
void checkAndHandleFeatureMove();
unsigned long sMillisOfLastDemo = 0;
#endif
void resetAttentionTimeout();

void setup() {
    pinModeFast(LED_BUILTIN, OUTPUT);
    pinModeFast(SHORT_DEMO_MODE_PIN, INPUT_PULLUP);
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

#if defined(QUADRUPED_HAS_US_DISTANCE_SENSOR)
    Serial.println(F("Init US distance sensor"));
    initUSDistancePins(TRIGGER_OUT_PIN, ECHO_IN_PIN);
#endif

#if defined(QUADRUPED_ENABLE_RTTTL)
    playRtttlBlockingPGM(BUZZER_PIN, Short);
#elif defined(BUZZER_PIN)
    tone(BUZZER_PIN, 2000, 300);
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

#if defined(QUADRUPED_HAS_US_DISTANCE_SENSOR)
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
    checkForVCCUnderVoltageAndShutdownPeriodically();
#endif

    /*
     * Check if all moving actions have set ACTION_TYPE_STOP after finishing
     */
    if (sCurrentlyRunningAction != ACTION_TYPE_STOP && sCurrentlyRunningAction != ACTION_TYPE_MELODY) {
        printCurrentActionString(&Serial);
        Serial.println(F(" is unexpected action type"));
    }

#if defined(QUADRUPED_HAS_US_DISTANCE_SENSOR)
    /*
     * US distance sensor handling
     * Displays distance on front bar and starts wave or melody at special distances
     */
    handleUSDistanceSensor();
#endif

#if defined(QUADRUPED_ENABLE_RTTTL)
    /*
     * Play melody if requested
     * Melody is stopped by other actions
     */
    handleMelody();
#endif

#if defined(QUADRUPED_HAS_IR_CONTROL)
    /*
     * IR control handling
     * Check for IR commands and execute them. Returns only AFTER finishing of requested blocking movement
     */
    if (IRDispatcher.checkAndRunSuspendedBlockingCommands()) {
        delay(50); // for the voltage to stabilize after the blocking movement
        resetAttentionTimeout();
    }

    checkAndHandleDemoMove();

    /*
     * Run demo move or get attention that no IR command was received since 2 minutes and quadruped may be switched off.
     * VCC undervoltage is also signaled here.
     */
    checkAndHandleAttention();

#else
    /*
     * Check for feature move, which runs only once 20 seconds after boot if no IR input was detected.
     * Feature move, needs more space than demo move and lasts around 3 minutes.
     */
    checkAndHandleFeatureMove();
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

#if defined(QUADRUPED_HAS_US_DISTANCE_SENSOR)
/**
 * Get US distance and display it on front bar and start wave or melody at special distances
 * Can introduce a delay up to 12 ms
 * This function is also called periodically from blocking movements by delayAndCheckByApplication(),
 * so we can check sCurrentlyRunningAction
 */
void handleUSDistanceSensor() {
    sUsedMillisForUSDistanceMeasurement = 0;
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
    MAX_DISTANCE_TIMEOUT_CM, MILLIS_BETWEEN_DISTANCE_MEASUREMENTS, &Serial); // sets sLastUSDistanceMeasurementMillis :-)
    sUSDistanceMeasurmentIsRunning = false;
    if (tDistanceMeasurmentResult != HCSR04_DISTANCE_NO_MEASUREMENT) {
        // 200 ms (MILLIS_BETWEEN_DISTANCE_MEASUREMENTS) are gone here
        /*
         * tBarLength is also used for melody and wave so compute it anyway
         *
         * The first 3 pixel represent a distance of each 5 cm
         * 1. pixel: 5 to 9 cm, 2. pixel: 10 to 14 cm, 3. pixel: 15 to 19 cm
         * The 4. to 7. pixel represents a distance of each 10 cm
         * 4. pixel: 20 to 29 cm, 5. pixel: 30 to 39 cm, 6. pixel: 40 to 49 cm, 7. pixel: 50 to 59 cm,
         * The 8. pixel is active if distance is >= 60 cm
         */
        uint8_t tBarLength;
        if (sUSDistanceCentimeter < 20) {
            tBarLength = sUSDistanceCentimeter / 5;
        } else if (sUSDistanceCentimeter < 60) {
            tBarLength = (sUSDistanceCentimeter / 10) + 2;
        } else {
            tBarLength = 8; // >= 60 cm
        }

#  if defined(QUADRUPED_HAS_NEOPIXEL)
        /*
         * Show distance bar if no other pattern is active on front bar
         */
        if (FrontNeoPixelBar.ActivePattern == PATTERN_NONE && QuadrupedNeoPixelBar.ActivePattern == PATTERN_NONE
                && sUSDistanceCentimeter != DISTANCE_TIMEOUT_RESULT) {
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
             * No action running here, we are called by loop()
             */
#  if defined(QUADRUPED_ENABLE_RTTTL)
            /*
             * Play melody if only first distance led is active 4 consecutive times, i.e. distance is between 5 and 9 cm (including)
             */
            if (tBarLength == BAR_LENGTH_FOR_MELODY && !isPlayRtttlRunning()) {
                sDelayCounterUntilMelody++;
                if (sDelayCounterUntilMelody >= DELAY_COUNT_FOR_MELODY) {
                    doRandomMelody();
                    sDelayCounterUntilMelody = 0;
                    return; // do not check for wave distance!
                }
            } else {
                sDelayCounterUntilMelody = 0;
            }
#  endif
            /*
             * Do wave if 4. distance Pixel is active and 5. pixel is off, i.e. distance is between 20 and 29 cm
             * First wave can be done 10 seconds after boot to avoid spurious wave at placing the quadruped
             * The wave can be repeated every MILLIS_BETWEEN_WAVES (10 seconds)
             */
            if (tBarLength == BAR_LENGTH_FOR_WAVE &&
#if defined(QUADRUPED_HAS_IR_CONTROL)
                    (millis() - IRDispatcher.IRReceivedData.MillisOfLastCode) > MILLIS_BETWEEN_WAVES
#else
                    (millis() - MillisOfLastAttention) > MILLIS_BETWEEN_WAVES
#endif
            ) {
                doWave();
                resetAttentionTimeout();
            }
        }
    }
}
#endif // #if defined(QUADRUPED_HAS_US_DISTANCE_SENSOR)

#if defined(ADC_UTILS_ARE_AVAILABLE)
/**
 *
 * @return true if undervoltage
 */
bool checkAndSignalUndervoltage() {
    if (isShutDownByUndervoltage) {
        playShutdownMelody(); // Do attention for Quadruped to switch off because of undervoltage
#if defined(QUADRUPED_HAS_NEOPIXEL)
        // Do not move servos, just light the Neopixel
        sCurrentlyRunningAction = ACTION_TYPE_SHUTDOWN; // signal it to Neopixel handler
        delay(40); // Neopixel handler is called every 20 ms, so it gets our request
        sCurrentlyRunningAction = ACTION_TYPE_STOP; // Do this animation only once now
#endif
    }
    return isShutDownByUndervoltage;
}
#endif

/*
 * This disables feature move and allows next attention in 2 minutes and next wave in 10 seconds
 */
void resetAttentionTimeout() {
#if defined(QUADRUPED_HAS_IR_CONTROL)
    IRDispatcher.IRReceivedData.MillisOfLastCode = millis();
#else
    MillisOfLastAttention = millis();
#endif
}

#if defined(QUADRUPED_HAS_IR_CONTROL)
/*
 * Check for demo move, which runs only if no IR command was ever received.
 * Short move repeat after 1 minute, long move after 2 minutes
 */
void checkAndHandleDemoMove() {
    if (IRDispatcher.IRReceivedData.MillisOfLastCode == 0 && millis() >= sMillisOfNextDemo) {

#  if defined(ADC_UTILS_ARE_AVAILABLE)
        if (!checkAndSignalUndervoltage())
#  endif
        {
            tone(BUZZER_PIN, 2200, 100);

            // If SHORT_DEMO_MODE_PIN is LOW, do short demo every minute
            bool tDoLongDemo = digitalRead(SHORT_DEMO_MODE_PIN);
            doQuadrupedDemoMoves(tDoLongDemo); // Can be terminated by IR command
            sMillisOfNextDemo = millis() + MILLIS_BETWEEN_DEMO_MOVES;
            if (tDoLongDemo) {
                sMillisOfNextDemo += MILLIS_BETWEEN_DEMO_MOVES; // next move in 2 x MILLIS_BETWEEN_DEMO_MOVES
            }
        }
    }
}

/*
 * If no IR command was received since 2 minutes and quadruped may be switched off, run attention or demo.
 * if we have VCC undervoltage signaled do not run attention but do attention by Melody and optional Neopixel.
 */
void checkAndHandleAttention() {
    if ((IRDispatcher.IRReceivedData.MillisOfLastCode != 0)
            && (millis() - IRDispatcher.IRReceivedData.MillisOfLastCode) > MILLIS_OF_INACTIVITY_BEFORE_ATTENTION_MOVE) {

#  if defined(ADC_UTILS_ARE_AVAILABLE)
        if (!checkAndSignalUndervoltage())
#  endif
        {
            doAttention();
        }
        // next check and movement in 1 minute
        IRDispatcher.IRReceivedData.MillisOfLastCode = millis() - (MILLIS_OF_INACTIVITY_BEFORE_ATTENTION_MOVE / 2);
#  if defined(ADC_UTILS_ARE_AVAILABLE)
        printVCCVoltageMillivolt(&Serial);
#endif
    }
}

#else
/*
 * Check for feature move, which runs 20 seconds after boot and is repeated every 4 minutes.
 * Feature move, needs more space than demo move and lasts around 3 minutes.
 */
void checkAndHandleFeatureMove() {
    if (((sMillisOfLastDemo == 0 && (millis() >= MILLIS_OF_INACTIVITY_AFTER_BOOT_BEFORE_SWITCH_TO_FEATURE_MOVE))
            || (millis() - sMillisOfLastDemo) > MILLIS_OF_INACTIVITY_BETWEEN_FEATURE_MOVE)) {
#  if defined(ADC_UTILS_ARE_AVAILABLE)
        if (!checkAndSignalUndervoltage())
#  endif
        {
            doQuadrupedFeatureMove();
        }
        sMillisOfLastDemo = millis();
    }
}
#endif

