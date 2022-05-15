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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

#include "QuadrupedConfiguration.h"

#if defined(QUADRUPED_HAS_IR_CONTROL)
// Include the header only IRCommandDispatcher library in the main program
#include "IRCommandMapping.h" // Must be included before IRCommandDispatcher.hpp to define IR_ADDRESS and IRMapping and string "unknown".
#include "IRCommandDispatcher.h"
#include "IRCommandDispatcher.hpp"
#define QUADRUPED_MOVEMENT_BREAK_FLAG (IRDispatcher.requestToStopReceived)
#endif

#include "QuadrupedBasicMovements.h" // This helps the Eclipse indexer
#include "QuadrupedControlCommands.hpp" // Commands can also be used e.g. in loop().
#if defined(QUADRUPED_HAS_NEOPIXEL)
#include "QuadrupedNeoPixel.hpp"
#endif

#include "QuadrupedHelper.h"

#include "ADCUtils.hpp" // for getVCCVoltageMillivoltSimple() and printVCCVoltageMillivolt()

#if defined(QUADRUPED_HAS_US_DISTANCE) && defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
Servo USServo; // Servo for US sensor use Servo library direct, we do nor require easings here ( and we have only 8 Servo easings allocated)
#define NO_LED_FEEDBACK_CODE // Disable IR LED feedback because servo is at the same pin. Must be included before IRCommandDispatcher.hpp
#endif

#if defined(QUADRUPED_ENABLE_RTTTL)
#include <PlayRtttl.hpp>
#endif

//#define INFO // activate this to see serial info output

/*
 * Loop control
 */
#define MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE 20000 // 20 seconds
#define MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE 120000 // 2 Minutes
#define MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE 60000 // 1 Minute

#define VERSION_EXAMPLE "3.0"
// 3.0 NeoPixel and distance sensor added
// 2.1 auto move
// 2.0 major refactoring
// 1.1 mirror computation at transformAndSetPivotServos and transformOneServoIndex

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    initializeAllQuadrupedServos(90);
    delay(2000);

    // Just for pre setting channel and reference
    getVCCVoltageMillivoltSimple();

#if defined(QUADRUPED_HAS_US_DISTANCE)
    Serial.println(F("Init US distance sensor"));
    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);
#endif
#if defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
    USServo.attach(PIN_US_SERVO);
    USServo.write(90);
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
    Serial.println(F(" at pin " STR(IR_INPUT_PIN)));
#endif

#if defined(QUADRUPED_HAS_NEOPIXEL)
    Serial.println(F("Init NeoPixel"));
    initNeoPatterns();
    enableServoEasingInterrupt(); // This enables the ServoEasing 20 ms interrupt, which we use to synchronize the NeoPixel update with the servo pulse generation.
#endif
}

void loop() {
    /*
     * Check for low voltage
     */
    if (checkForLowVoltage()) {
        shutdownServos();
        playShutdownMelody();
#if defined(QUADRUPED_HAS_NEOPIXEL)
        wipeOutPatternsBlocking(); // to save power
#endif
        delay(10000);  // blocking wait for next check
    }

    /*
     * US distance sensor handling
     */
#if defined(QUADRUPED_HAS_US_DISTANCE)
    handleUSSensor(); // currently only distance display on front bar
#endif

    /*
     * IR control handling
     */
#if defined(QUADRUPED_HAS_IR_CONTROL)
    //Check for IR commands and execute them. Returns only AFTER finishing of requested blocking movement
    IRDispatcher.checkAndRunSuspendedBlockingCommands();

    // Call doQuadrupedAutoMove() if timeout (20 s) after boot was reached and no IR command was received
    if (IRDispatcher.IRReceivedData.MillisOfLastCode == 0 && (millis() > MILLIS_OF_INACTIVITY_BEFORE_SWITCH_TO_AUTO_MOVE)) {
        doQuadrupedAutoMove(); // Can be terminated by IR command
    }

    //Get attention that no IR command was received since 1 minutes and quadruped may be switched off
    if (millis() - IRDispatcher.IRReceivedData.MillisOfLastCode > MILLIS_OF_INACTIVITY_BEFORE_REMINDER_MOVE) {
        IRDispatcher.IRReceivedData.MillisOfLastCode += MILLIS_OF_INACTIVITY_BETWEEN_REMINDER_MOVE; // next attention in 1 minute
        doAttention();
        printVCCVoltageMillivolt(&Serial);
    }
#endif
}
