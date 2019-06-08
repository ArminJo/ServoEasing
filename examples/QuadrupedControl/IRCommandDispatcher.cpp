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

#include "IRCommandMapping.h" // must be before IRLremote.h. Specifies the IR input pin at A0

#if (IR_CONTROL_CODING == 'P')
CPanasonic IRLremote;
#else
CNec IRLremote;
#endif

bool sJustExecutingCommand;             // set if we just execute a command by dispatcher
bool sCurrentCommandIsRepeat;
bool sValidIRCodeReceived = false;      // set if we received a valid IR code. Used for breaking timeout for auto move.
uint32_t sLastTimeOfValidIRCodeReceived; // millis of last IR command
bool sRequestToStopReceived;            // flag for movements to stop, set by checkIRInput()

// internally used
uint8_t sCurrentIRCode;                 // to decide if we have a repetition of non instant commands
uint8_t sNextCommand = COMMAND_EMPTY; // Next command for main loop, set by checkIRInput(). If != 0 do not wait for next IR command just take this command.

void setupIRDispatcher() {

    // Start reading the remote. PinInterrupt or PinChangeInterrupt* will automatically be selected
    if (!IRLremote.begin(IR_RECEIVER_PIN)) {
        Serial.println(F("You did not choose a valid pin"));
    }
}

void loopIRDispatcher() {
    uint8_t tIRCode;

    sRequestToStopReceived = false;
    bool tValidIRCodeReceived = false;
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
        for (uint8_t i = 0; i < sizeof(IRMapping) / sizeof(struct IRToCommandMapping); ++i) {
            if (tIRCode == IRMapping[i].IRCode) {
                sCurrentIRCode = tIRCode;
                Serial.print(F("Calling "));
                Serial.println(reinterpret_cast<const __FlashStringHelper *>(IRMapping[i].CommandString));
                // use locally
                tValidIRCodeReceived = true;

                sLastTimeOfValidIRCodeReceived = millis();
                // one time flag
                sValidIRCodeReceived = true;

                /*
                 * Call the function specified in IR mapping
                 */
                sJustExecutingCommand = true;
                IRMapping[i].CommandToCall();
                sJustExecutingCommand = false;
                break;
            }
        }
        if (!tValidIRCodeReceived) {
            // must be after IRMapping search
            if (checkAndCallInstantCommands(tIRCode)){
                sLastTimeOfValidIRCodeReceived = millis();
                // one time flag
                sValidIRCodeReceived = true;
            }
        }
    }
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

            Serial.print(F("A=0x"));
            Serial.print(tIRData.address, HEX);
            Serial.print(F(" C=0x"));
            Serial.print(tIRData.command, HEX);
            tIRReturnValue = tIRData.command;
//            if (sReferenceAddress == 0) {
//                // store reference address for error detection
//                sReferenceAddress = tIRData.address;
//            }
            if (tIRData.address == IR_ADDRESS) {
                // new code for right address
                sCurrentCommandIsRepeat = false;
                sLastIRValue = tIRReturnValue;
                break;
            } else if (tIRData.address == IR_REPEAT_ADDRESS && tIRData.command == IR_REPEAT_CODE && sLastIRValue != 0) {
                // received repeat code
                Serial.print(F(" R"));
                sCurrentCommandIsRepeat = true;
                tIRReturnValue = sLastIRValue;
                break;
            } else {
                Serial.print(F(" unknown"));
                // unknown code - maybe here, because other interrupts interfere with the IR Interrupt
                // Disable repeat in order not to repeat the wrong command
                sLastIRValue = COMMAND_EMPTY;
            }
            Serial.println();
        }
    } while (doWait);

    return tIRReturnValue;
}

/*
 * Instant Command are commands which can be executed at each time in movement.
 * return true if instant command found.
 */
bool checkAndCallInstantCommands(uint8_t aIRCode) {
// search IR code and call associated function
    for (uint8_t i = 0; i < sizeof(IRMappingInstantCommands) / sizeof(struct IRToCommandMapping); ++i) {
        if (aIRCode == IRMappingInstantCommands[i].IRCode) {

            Serial.print(F("Calling "));
            Serial.print(reinterpret_cast<const __FlashStringHelper *>(IRMappingInstantCommands[i].CommandString));
            Serial.println(':');
            /*
             * Call the instant function specified in IR mapping
             */
            IRMappingInstantCommands[i].CommandToCall();
            return true;
        }
    }
    return false;
}

/*
 * Suppress repeated IR non instant commands
 * Sets sRequestToStopReceived if stop received
 */
void checkIRInput() {
    uint8_t tIRCode = getIRCommand(false);
    if (tIRCode == COMMAND_EMPTY) {
        return;
    } else if (tIRCode == sCurrentIRCode) {
        // suppress repeated main (non instant) command
        return;
    } else if (checkAndCallInstantCommands(tIRCode)) {
        return;
    } else {
        sNextCommand = tIRCode;
        sRequestToStopReceived = true; // return to loop
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

/*
 * Returns true if stop received
 */
void delayAndCheckIRInput(uint16_t aDelayMillis) {
    uint32_t tStartMillis = millis();
    do {
        checkIRInput();
        RETURN_IF_STOP;
    }while (millis() - tStartMillis > aDelayMillis);
}
