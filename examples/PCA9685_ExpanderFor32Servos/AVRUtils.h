/*
 * AVRUtils.h
 *
 *  Copyright (C) 2016-2024  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Utils https://github.com/ArminJo/Arduino-Utils.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
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
 *
 */

#include "Arduino.h"

#if defined(__AVR__) && defined (SPMCSR) && !(defined(__AVR_ATtiny1616__)  || defined(__AVR_ATtiny3216__) || defined(__AVR_ATtiny3217__))
#ifndef _AVR_UTILS_H
#define _AVR_UTILS_H

#include <stdint.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "avr/boot.h"

/*
 * The largest address just not allocated so far
 * Under Unix, the "break value" was the end of the data
 * segment as dynamically requested from the operating system.
 * Since we don't have an operating system, just make sure
 * that we don't collide with the stack.
 */
extern void *__brkval; // The largest address just not allocated so far / start of available / free heap, initialized at first malloc()
extern void *__flp; //
extern char __heap_start; // = __bss_end, the linker address of heap start
#define DEFAULT_MALLOC_MARGIN   128

/*
 * The stack amount used for call of malloc(), i.e. Stack is lowered by this value before applying __malloc_margin.
 * No malloc() possible if size at caller (stack) position is lower than (__malloc_margin + HEURISTIC_ADDITIONAL_MALLOC_MARGIN).
 */
#define HEURISTIC_ADDITIONAL_MALLOC_MARGIN 14

/*
 * storage for millis value to enable compensation for interrupt disable at signal acquisition etc.
 */
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)  || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
#  if !defined(_MILLIS_UTILS_H)
#define timer0_millis millis_timer_millis // The ATTinyCore libraries use other variable name in wiring.c - copied from MillisUtils.h
#  endif
#  if !defined(DEFAULT_MILLIS_FOR_WAKEUP_AFTER_POWER_DOWN)
#define DEFAULT_MILLIS_FOR_WAKEUP_AFTER_POWER_DOWN  65
#  endif
#else
#  if !defined(DEFAULT_MILLIS_FOR_WAKEUP_AFTER_POWER_DOWN)
#define DEFAULT_MILLIS_FOR_WAKEUP_AFTER_POWER_DOWN  0   // default for Uno / Nano etc.
#  endif
#endif

extern volatile unsigned long timer0_millis;

void initSleep(uint8_t tSleepMode);
void initPeriodicSleepWithWatchdog(uint8_t tSleepMode, uint8_t aWatchdogPrescaler);
void initTimeoutWithWatchdog(uint8_t aWatchdogPrescaler);
uint16_t computeSleepMillis(uint8_t aWatchdogPrescaler);
void sleepWithWatchdog(uint8_t aWatchdogPrescaler, bool aAdjustMillis = false);

#include <Print.h>

uint8_t* getAvailableHeapStart() __attribute__ ((deprecated ("Renamed to getStartOfAvailableHeap()")));
void printAvailableHeapStart(Print *aSerial) __attribute__ ((deprecated ("Renamed to printStartOfAvailableHeap()")));
uint8_t* getStartOfAvailableHeap(void);
void printStartOfAvailableHeap(Print *aSerial);
uint16_t getCurrentAvailableStackSize(void);
void printCurrentAvailableStackSize(Print *aSerial);
uint16_t getCurrentAvailableHeapSize(void);
uint16_t getTheoreticalMaximumAvailableHeapSize(void);
void printCurrentAvailableHeapSize(Print *aSerial);
void printCurrentAvailableHeapSizeSimple(Print *aSerial);
// print available heap at current program (SP value matters) position
#define PRINT_AVAILABLE_HEAP   Serial.print(F("available="));Serial.println(SP - (uint16_t) __brkval + 1 - HEURISTIC_ADDITIONAL_MALLOC_MARGIN - ((uint16_t) __malloc_margin))

#define HEAP_STACK_UNTOUCHED_VALUE 0x5A
void initStackFreeMeasurement();

int16_t getStackMaxUsedAndUnusedSizes(uint16_t *aStackUnusedSizePointer);
int16_t getHeapMaxUsedSize();
void printStackMaxUsedAndUnusedSizes(Print *aSerial);
bool printStackMaxUsedAndUnusedSizesIfChanged(Print *aSerial);

void printBaseRAMData(Print *aSerial);
void printRAMAndStackInfo(Print *aSerial);
void printRAMInfo(Print *aSerial) __attribute__ ((deprecated ("Renamed to printRAMAndStackInfo()")));

bool isAddressInRAM(void *aAddressToCheck);
bool isAddressBelowAvailableHeapStart(void *aAddressToCheck);

void set__malloc_margin(uint8_t aNewMallocMargin);
void reset__malloc_margin();

void testCallocSizesAndPrint(Print *aSerial);

#endif // _AVR_UTILS_H
#endif //  defined(__AVR__)
