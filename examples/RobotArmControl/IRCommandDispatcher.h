/*
 * IRCommandDispatcher.h
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#ifndef SRC_IRCOMMANDDISPATCHER_H_
#define SRC_IRCOMMANDDISPATCHER_H_

#include <stdint.h>

#if ! defined(IR_RECEIVER_PIN)
#define IR_RECEIVER_PIN  A0
#endif

#if (IR_RECEIVER_PIN != 2) && (IR_RECEIVER_PIN != 3)
#include <PinChangeInterrupt.h> // must be included if we do not use pin 2 or 3
#endif

extern bool sJustCalledMainCommand;
extern bool sExecutingMainCommand;  // set if we just execute a command by dispatcher
extern bool sCurrentCommandIsRepeat;
extern bool sAtLeastOneValidIRCodeReceived;   // set if we received a valid IR code. Used for breaking timeout for auto move.
extern uint32_t sLastTimeOfValidIRCodeReceived; // millis of last IR command
extern bool sRequestToStopReceived; // flag for main loop, set by checkIRInput()

#define RETURN_IF_STOP if (sRequestToStopReceived) return

void setupIRDispatcher();
bool loopIRDispatcher();

uint8_t getIRCommand(bool doWait);
bool checkIRInput();

bool checkAndCallMainCommands(uint8_t aIRCode);
bool checkAndCallInstantCommands(uint8_t aIRCode); // function to search in MappingInstantCommands array

void printIRCommandString(uint8_t aIRCode);

#endif /* SRC_IRCOMMANDDISPATCHER_H_ */

#pragma once
