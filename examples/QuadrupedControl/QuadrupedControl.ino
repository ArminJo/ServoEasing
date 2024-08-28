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
 *
 * To run this example you need to install the "ServoEasing" library.
 * For full control install also "IRremote", "PlayRtttl", "NeoPatterns" and "Adafruit_NeoPixel" libraries.
 * These libraries can be installed under "Tools -> Manage Libraries..." or "Ctrl+Shift+I".
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of QuadrupedControl https://github.com/ArminJo/QuadrupedControl.
 *
 *  QuadrupedControl is free software: you can redistribute it and/or modify
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

#define VERSION_EXAMPLE "3.3"
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
#define ENABLE_QUADRUPED_DEMO_MODE

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
#define MILLIS_BETWEEN_DISTANCE_MEASUREMENTS_FOR_MELODY 500 // 2 measurements per second
#define MIN_DISTANCE_FOR_MELODY_CM               10     // Play melody triggered by distance below 5 cm.
#define MIN_DISTANCE_FOR_WAVE_CM                 25     // Waves triggered by distance between 25 cm and 35 cm.
#define MAX_DISTANCE_FOR_WAVE_CM                 35     // Waves triggered by distance between 25 cm and 35 cm.
#define MAX_DISTANCE_TIMEOUT_CM                 200     // Do not try to measure distances above 200 cm.
#define MILLIS_BETWEEN_WAVES                  10000     // 10 second between waves triggered by distance.
#include "HCSR04.hpp"
#endif

#include "QuadrupedHelper.hpp"              // for checkForLowVoltage() and playShutdownMelody()

#if defined(QUADRUPED_HAS_IR_CONTROL)
// Include the header only IRCommandDispatcher library in the main program
#include "QuadrupedIRCommandMapping.h"      // Must be included before IRCommandDispatcher.hpp to define IR_ADDRESS and IRMapping and string "unknown".
#include "IRCommandDispatcher.hpp"
#define QUADRUPED_MOVEMENT_BREAK_FLAG (IRDispatcher.requestToStopReceived)
#else
#define QUADRUPED_MOVEMENT_BREAK_FLAG false
#endif

#define USE_NO_RTX_EXTENSIONS // Disables RTX format definitions `'s'` (style) and `'l'` (loop). Saves up to 332 bytes program memory
#if defined(QUADRUPED_HAS_NEOPIXEL)
#define ENABLE_EXTERNAL_SERVO_TIMER_HANDLER // Evaluated by ServoEasing.hpp
#include "QuadrupedNeoPixel.hpp"
#endif
#include "QuadrupedControlCommands.hpp" // In turn includes ServoEasing. Commands can also be used e.g. in loop().
#if defined(QUADRUPED_ENABLE_RTTTL)
#include <PlayRtttl.hpp>
#endif

#include "ADCUtils.hpp" // for getVCCVoltageMillivoltSimple() and printVCCVoltageMillivolt()

#if defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
ServoEasing USServo;
#endif

//#define INFO // activate this to see serial info output

/*
 * Loop control
 */
#if defined(QUADRUPED_HAS_IR_CONTROL)
#define MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE     40000   // 40 seconds
#else
#define MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE     20000   // 20 seconds
#define MILLIS_OF_INACTIVITY_BETWEEN_AUTO_MOVE             240000   // 4 minutes
uint32_t sMillisOfLastSpecialAction = 0;                            // millis() of last doAttention() or doWave()
#endif
// In demo mode AutoMove is disabled since demo runs earlier
#if defined(ENABLE_QUADRUPED_DEMO_MODE)
#define MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE           60000   // 1 minute before demo move
#define MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE          60000   // 1 minute between each demo move
#else
#define MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE          120000   // 2 Minutes
#define MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE          60000   // 1 Minute
#endif

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

#if defined(QUADRUPED_HAS_US_DISTANCE)
    /*
     * US distance sensor handling
     * Currently distance display on front bar and wave or melody at different distances
     */
    handleUSSensor();
#endif

    /*
     * Play melody if requested
     */
#if defined(QUADRUPED_ENABLE_RTTTL)
    if (isPlayRtttlRunning()) {
        if (!updatePlayRtttl()) {
            // Melody has just stopped - delay autoruns and attentions
#if defined(QUADRUPED_HAS_IR_CONTROL)
            IRDispatcher.IRReceivedData.MillisOfLastCode = millis(); // disable next auto move, next attention in 2 minutes
#else
            sMillisOfLastSpecialAction = millis(); // disable next auto move, next attention in 2 minutes
#endif
            sCurrentlyRunningAction = ACTION_TYPE_STOP; // to enable next melody
        } else if (sCurrentlyRunningAction == ACTION_TYPE_STOP) {
            stopPlayRtttl();
        }
    }
#endif

    /*
     * IR control handling
     */
#if defined(QUADRUPED_HAS_IR_CONTROL)
    //Check for IR commands and execute them. Returns only AFTER finishing of requested blocking movement
    if (IRDispatcher.checkAndRunSuspendedBlockingCommands()) {
        delay(50); // for the voltage to stabilize after the blocking movement
    }
#endif

    /*
     * Auto move if timeout (40 s) after boot was reached (and no IR command was received and not powered by USB)
     */
    if (!isShutDown) {
        if ((millis() > MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE)
#if defined(QUADRUPED_HAS_IR_CONTROL)
                && IRDispatcher.IRReceivedData.MillisOfLastCode == 0 /* Do auto move only if no IR command received */
#else
                && sMillisOfLastSpecialAction == 0
#endif
                && (!isVCCUSBPowered())) {
            doQuadrupedAutoMove(); // Can be terminated by IR command
#if defined(QUADRUPED_HAS_IR_CONTROL)
            IRDispatcher.IRReceivedData.MillisOfLastCode = millis(); // disable next auto move, next attention in 2 minutes
#else
            sMillisOfLastSpecialAction = millis(); // disable next auto move, next attention in 2 minutes
#endif
        }
    }

    /*
     * Run demo or get attention that no IR command was received since 2 minutes and quadruped may be switched off
     */
#if defined(QUADRUPED_HAS_IR_CONTROL)
    if (millis() - IRDispatcher.IRReceivedData.MillisOfLastCode > MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE)
#else
    if ((millis() - sMillisOfLastSpecialAction) > MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE)
#endif
    {
        if (isShutDown) {
            // do not move servos, just light the Neopixel
            sCurrentlyRunningAction = ACTION_TYPE_ATTENTION;
            delay(40); // Neopixel handler is called every 20 ms
            sCurrentlyRunningAction = ACTION_TYPE_STOP; // do not light forever
        } else {
#  if defined(ENABLE_QUADRUPED_DEMO_MODE)
            doQuadrupedDemoMove(); // Can be terminated by IR command
#  else
            doAttention();
#  endif

#if defined(QUADRUPED_HAS_IR_CONTROL)
            IRDispatcher.IRReceivedData.MillisOfLastCode = millis()
                    - (MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE - MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE); // next attention in 1 minute if no IR command
#else
            sMillisOfLastSpecialAction = millis()
                    - (MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE - MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE); // next attention in 1 minute
#endif
        }
#if defined(ADC_UTILS_ARE_AVAILABLE)
        printVCCVoltageMillivolt(&Serial);
#endif
    }
}
