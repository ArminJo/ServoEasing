/*
 * IRCommandDispatcher.cpp
 *
 * Receives command by IR and calls functions specified in a mapping array.
 *
 * Program for controlling a mePed Robot V2 with 8 servos using an IR Remote at pin A0
 * Supported IR remote are KEYES (the original mePed remote) and WM10
 * Select the one you have at line 20 in IRCommandMapping.h
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

#include "IRCommandDispatcher.h"
#include <IRLremote.h>      // include IR Remote library

#include "IRCommandMapping.h"

#if (IR_CONTROL_CODING == 'P')
CPanasonic IRLremote;
#else
CNec IRLremote;
#endif

bool sJustCalledMainCommand;
bool sExecutingMainCommand;             // set if we just execute a command by dispatcher
bool sCurrentCommandIsRepeat;
bool sAtLeastOneValidIRCodeReceived = false; // one time flag. Set if we received a valid IR code. Used for breaking timeout for auto move.
uint32_t sLastTimeOfValidIRCodeReceived; // millis of last IR command
uint8_t sActionType;
/*
 * Flag for movements to stop, set by checkIRInput().
 * It works like an exception so we do not need to propagate the return value from the delay up to the movements.
 * Instead we can use "if (sRequestToStopReceived) return;" (available as macro RETURN_IF_STOP).
 */
bool sRequestToStopReceived;

// internally used
uint8_t sCurrentIRCode;                // to decide if we have a repetition of non instant commands. Not needed for NEC remotes.
uint8_t sNextCommand = COMMAND_EMPTY; // Next command for main loop, set by checkIRInput(). If != 0 do not wait for next IR command just take this command.

void setupIRDispatcher() {

    // Start reading the remote. PinInterrupt or PinChangeInterrupt* will automatically be selected
    if (!IRLremote.begin(IR_RECEIVER_PIN)) {
        Serial.println(F("You did not choose a valid pin"));
    }
}

/**
 * @return true if valid main or instant command received
 */
bool loopIRDispatcher() {
    uint8_t tIRCode;

    // here we are stopped, so lets start a new turn
    sRequestToStopReceived = false;

    /*
     * Handle sNextCommand received while waiting for a movement to end
     */
    if (sNextCommand == COMMAND_EMPTY) {
        tIRCode = getIRCommand(false);
    } else {
        tIRCode = sNextCommand;
        sNextCommand = COMMAND_EMPTY;
    }

    /*
     * search IR code and call associated function
     */
    if (tIRCode != COMMAND_EMPTY) {
        if (checkAndCallMainCommands(tIRCode)) {
            return true;
        } else {
            // must be after checkAndCallMainCommands
            return checkAndCallInstantCommands(tIRCode);
        }
    }
    return false;
}

/*
 * Wait for next IR command
 */
uint8_t getIRCommand(bool doWait) {
    static uint8_t sLastIRValue = 0;
//    static uint16_t sReferenceAddress = 0; // store first received address here for better IR-receive error handling

    uint8_t tIRReturnValue = COMMAND_EMPTY;

    do {
        if (IRLremote.available()) {
            // Get the new data from the remote
            auto tIRData = IRLremote.read();

            if (tIRData.address == IR_REPEAT_ADDRESS && tIRData.command == IR_REPEAT_CODE) {
                /*
                 * Handle repeat code
                 */
                Serial.println(F("Repeat received"));
                if (sLastIRValue != COMMAND_EMPTY) {
                    sCurrentCommandIsRepeat = true;
                    tIRReturnValue = sLastIRValue;
                }
            } else {
                /*
                 * Regular code here
                 */
                Serial.print(F("A=0x"));
                Serial.print(tIRData.address, HEX);
                Serial.print(F(" C=0x"));
                Serial.print(tIRData.command, HEX);

                tIRReturnValue = tIRData.command;
//              if (sReferenceAddress == 0) {
//                  // store reference address for error detection
//                  sReferenceAddress = tIRData.address;
//              }
                if (tIRData.address == IR_ADDRESS) {
                    // Received new code (with right address)
                    sCurrentCommandIsRepeat = false;
                    sLastIRValue = tIRReturnValue;
                    Serial.println();
                    break;
                } else {
                    Serial.println(F(" Unknown"));
                    // unknown code - maybe here, because other interrupts interfere with the IR Interrupt
                    // Disable repeat in order not to repeat the wrong command
                    sLastIRValue = COMMAND_EMPTY;
                }
            }
        }
    } while (doWait);

    return tIRReturnValue;
}

/*
 * @return  true if main command found.
 * @note    Main command are commands which can only be executed if no movement happens.
 */
bool checkAndCallMainCommands(uint8_t aIRCode) {
    for (uint8_t i = 0; i < sizeof(IRMapping) / sizeof(struct IRToCommandMapping); ++i) {
        if (aIRCode == IRMapping[i].IRCode) {
            sCurrentIRCode = aIRCode;

            Serial.print(F("Calling "));
            Serial.println(reinterpret_cast<const __FlashStringHelper *>(IRMapping[i].CommandString));

            sLastTimeOfValidIRCodeReceived = millis();
            // one time flag used for breaking timeout for auto move.
            sAtLeastOneValidIRCodeReceived = true;

            /*
             * Call the function specified in IR mapping
             */
            sJustCalledMainCommand = true;
            sExecutingMainCommand = true;
            IRMapping[i].CommandToCall(); // Blocking call. Will only return after the move is finished.
            sExecutingMainCommand = false;
            return true;
        }
    }
    return false;
}
/*
 * @return  true if instant command found.
 * @note    Instant command are commands which can be executed at each time in movement.
 */
bool checkAndCallInstantCommands(uint8_t aIRCode) {
// search IR code and call associated function
    for (uint8_t i = 0; i < sizeof(IRMappingInstantCommands) / sizeof(struct IRToCommandMapping); ++i) {
        if (aIRCode == IRMappingInstantCommands[i].IRCode) {

            Serial.print(F("Calling "));
            Serial.print(reinterpret_cast<const __FlashStringHelper *>(IRMappingInstantCommands[i].CommandString));

            sLastTimeOfValidIRCodeReceived = millis();
            // one time flag used for breaking timeout for auto move.
            sAtLeastOneValidIRCodeReceived = true;

            /*
             * Call the instant function specified in IR mapping
             * The command can print in the same line
             */
            Serial.print(' ');
            IRMappingInstantCommands[i].CommandToCall();
            Serial.println();
            return true;
        }
    }
    return false;
}

/*
 * Suppress repeated IR non instant commands
 * Sets sRequestToStopReceived if stop received
 * @return  true - if IR command stop received
 *
 */
bool checkIRInput() {
    uint8_t tIRCode = getIRCommand(false);
    if (tIRCode == COMMAND_EMPTY) {
        return false;
    } else if (tIRCode == sCurrentIRCode) {
        // suppress repeated main (non instant) command
        return false;
    } else if (checkAndCallInstantCommands(tIRCode)) {
        return false;
    } else {
        sNextCommand = tIRCode;
        sRequestToStopReceived = true; // return to loop

        sLastTimeOfValidIRCodeReceived = millis();
        // one time flag
        sAtLeastOneValidIRCodeReceived = true;
        return true;
    }
}

void printIRCommandString(uint8_t aIRCode) {
    Serial.print(F("IRCommand="));
    for (uint8_t i = 0; i < sizeof(IRMapping) / sizeof(struct IRToCommandMapping); ++i) {
        if (aIRCode == IRMapping[i].IRCode) {
            Serial.println(reinterpret_cast<const __FlashStringHelper *>(IRMapping[i].CommandString));
            return;
        }
    }
    Serial.println(reinterpret_cast<const __FlashStringHelper *>(unknown));
}

// to be overwritten by example or user function.
bool __attribute__((weak)) checkOncePerDelay() {
    return false; // everything OK
}

/*
 * Special delay function for the quadruped control.
 * It checks for low voltage, IR input and US distance sensor
 * @return  true - if exit condition occurred like stop received
 */
bool delayAndCheck(uint16_t aDelayMillis) {
    uint32_t tStartMillis = millis();

    // check only once per delay
    if (!checkOncePerDelay()) {
        do {
            if (checkIRInput()) {
                Serial.println(F("IR stop received -> exit from delayAndCheck"));
                sActionType = ACTION_TYPE_STOP;
                return true;
            }
            yield();
        } while (millis() - tStartMillis < aDelayMillis);
        return false;
    }
    sActionType = ACTION_TYPE_STOP;
    return true;
}
