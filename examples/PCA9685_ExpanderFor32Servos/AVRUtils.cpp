/*
 *  AVRUtils.cpp
 *
 *  Stack, Ram and Heap utilities.
 *  Sleep utilities.
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
#include "AVRUtils.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdlib.h> // for __malloc_margin

/*
 * Returns actual start of available / free heap
 * Usage for print:
 Serial.print(F("AvailableHeapStart=0x"));
 Serial.println((uint16_t) getStartOfAvailableHeap(), HEX);
 */

uint8_t* getStartOfAvailableHeap(void) {
    if (__brkval == 0) {
        // __brkval is 0 if no malloc() has happened before
//        __brkval = __malloc_heap_start;
        __brkval = &__heap_start;
    }
    return (uint8_t*) __brkval;
}

void printStartOfAvailableHeap(Print *aSerial) {
    aSerial->print(F("Heap start="));
    aSerial->println((uint16_t) getStartOfAvailableHeap());
}

/*
 * Initialize RAM between current stack and actual heap start (__brkval) with pattern 0x5A
 */
void initStackFreeMeasurement() {
    uint8_t *tHeapPtr = getStartOfAvailableHeap(); // This sets __brkval

// Fill / paint stack
    do {
        *tHeapPtr++ = HEAP_STACK_UNTOUCHED_VALUE;
    } while (tHeapPtr < (uint8_t*) SP);
}

/*
 * @param aStackUnusedSizePointer points to variable which is written with amount of stack/heap not used/touched.
 * @return The amount of stack/heap touched since the last call to initStackFreeMeasurement()
 *          -1 if stack was completely used
 * A downward search fails, because it finds an allocated variable / array on stack, which was unused!
 * An upward search may be wrong, and claiming too much stack used, because malloc does not initialize the memory
 * and the search fails with multiple mallocs and partial writing of allocated regions.
 * In this case you should initialize stack free measurement after releasing last heap block.
 */
//#include <Arduino.h> // for Serial
int16_t getStackMaxUsedAndUnusedSizes(uint16_t *aStackUnusedSizePointer) {
    /*
     * Search for first touched value from end of current heap.
     */
    uint16_t tStackUnused = 0;
    uint8_t *tHeapPtr = getStartOfAvailableHeap(); // __brkval
    while (*tHeapPtr == HEAP_STACK_UNTOUCHED_VALUE && tHeapPtr <= (uint8_t*) SP) {
        tHeapPtr++;
        tStackUnused++;
    }

    int16_t tStackMaxUsedSize = (RAMEND + 1) - (uint16_t) tHeapPtr;

    *aStackUnusedSizePointer = tStackUnused;
    if (tStackUnused == 0) {
        return -1;
    }
    return tStackMaxUsedSize;
}

/*
 * Prints the amount of stack NOT used/touched and used/touched since the last call to initStackFreeMeasurement()
 * Example: "Stack unused=0, used=16" if stack runs into data
 */
void printStackMaxUsedAndUnusedSizes(Print *aSerial) {
    uint16_t tStackUnusedBytes;
    aSerial->print(F("Stack used="));
    aSerial->print(RAMEND - SP);
    aSerial->print(F(", max used="));
    aSerial->print(getStackMaxUsedAndUnusedSizes(&tStackUnusedBytes));
    aSerial->print(F(", unused="));
    aSerial->print(tStackUnusedBytes);
    aSerial->print(F(" of current total "));
    aSerial->println((RAMEND + 1) - (uint16_t) getStartOfAvailableHeap());
}

/*
 * Search upwards the first two HEAP_STACK_UNTOUCHED_VALUE values after current begin of heap
 */
uint16_t getHeapMaxUsedSize() {
    uint8_t *tHeapPtr = getStartOfAvailableHeap();
    while (*tHeapPtr != HEAP_STACK_UNTOUCHED_VALUE && *(tHeapPtr + 1) != HEAP_STACK_UNTOUCHED_VALUE && tHeapPtr <= (uint8_t*) SP) {
        tHeapPtr++;
    }
    // tHeapPtr points now to lowest untouched stack position or to lowest current stack byte
    return tHeapPtr - (uint8_t*) __malloc_heap_start;
}

/*
 * Prints the amount of stack NOT used/touched and used/touched since the last call to initStackFreeMeasurement()
 * Print only if value changed.
 * @return true, if values changed
 */
bool printStackMaxUsedAndUnusedSizesIfChanged(Print *aSerial) {
    static int16_t tOldStackUsedBytes = 0;

    uint16_t tStackUnusedBytes;
    int16_t tStackMaxUsedBytes = getStackMaxUsedAndUnusedSizes(&tStackUnusedBytes);
    if (tOldStackUsedBytes != tStackMaxUsedBytes) {
        tOldStackUsedBytes = tStackMaxUsedBytes;
        aSerial->print(F("Stack used="));
        aSerial->print(RAMEND - SP);
        aSerial->print(F(", max used="));
        aSerial->print(tStackMaxUsedBytes);
        aSerial->print(F(", unused="));
        aSerial->println(tStackUnusedBytes);
        return true;
    }
    return false;
}

/*
 * Get amount of free Stack = CURRENT stackpointer - heap end
 * Value computed depends on current stackpointer!
 */
uint16_t getCurrentAvailableStackSize(void) {
    uint16_t tAvailableHeapStart = (uint16_t) getStartOfAvailableHeap(); // __brkval
    if (tAvailableHeapStart >= SP) {
        return 0;
    }
    return (SP - tAvailableHeapStart);
}
void printCurrentAvailableStackSize(Print *aSerial) {
    aSerial->print(F("Currently available Stack[bytes]="));
    aSerial->println(getCurrentAvailableStackSize());
}

/*
 * Get amount of maximum available memory for malloc()
 * Value computed depends on current stackpointer!
 * FreeRam - __malloc_margin (128 for ATmega328)
 */
uint16_t getCurrentAvailableHeapSize(void) {
    if (getCurrentAvailableStackSize() <= __malloc_margin) {
        return 0;
    }
    // SP - __brkval - __malloc_margin
    return getCurrentAvailableStackSize() - __malloc_margin; // (128)
}

/*
 * malloc() computes the margin to maximum heap end as __malloc_heap_end - __malloc_margin if __malloc_margin != 0,
 * but it seems to be 0 so it falls back to STACK_POINTER() - __malloc_margin, wherever the stackpointer is when malloc() is called
 * This value is never reached, since it assumes, that malloc() does not use RAMEND, but SP for computing the margin :-(
 */
uint16_t getTheoreticalMaximumAvailableHeapSize(void) {
    if (RAMEND <= __malloc_margin) {
        return 0;
    }
    return (RAMEND - RAMSTART) -__malloc_margin; // (128)
}

/*
 * Value computed depends on current stackpointer!
 */
void printCurrentAvailableHeapSize(Print *aSerial) {
    aSerial->print(F("Currently available Heap[bytes]="));
    aSerial->println(getCurrentAvailableHeapSize());
}

/*
 * Simple and short implementation, does not work before initStackFreeMeasurement() or first malloc()
 * The STACK required for this function is 4 bytes, so available numbers are 4 less than for caller.
 */
void printCurrentAvailableHeapSizeSimple(Print *aSerial) {
    aSerial->print(F("available="));
    aSerial->println(SP - (uint16_t) __brkval + 1 - ((uint16_t) __malloc_margin));
}

// This define is in AVRUtils.h
//#define PRINT_AVAILABLE_HEAP   Serial.print(F("available="));Serial.println(SP - (uint16_t) __brkval + 1 - HEURISTIC_ADDITIONAL_MALLOC_MARGIN - ((uint16_t) __malloc_margin))

void printBaseRAMData(Print *aSerial) {
    // __malloc_heap_end seems to be 0
    aSerial->print(F("__malloc_heap_start="));
    aSerial->print((uint16_t) __malloc_heap_start); // = initialized with __bss_end, __heap_start in lst file
    aSerial->print(F("|0x"));
    aSerial->print((uint16_t) __malloc_heap_start, HEX);

    aSerial->print(F(", &__heap_start="));
    aSerial->print((uint16_t) &__heap_start); //  = __bss_end, the linker address of heap start
    aSerial->print(F("|0x"));
    aSerial->print((uint16_t) &__heap_start, HEX);

    aSerial->print(F(", __brkval="));
    aSerial->print((uint16_t) __brkval); // The largest address just not allocated so far / start of available / free heap, initialized at first malloc()
    aSerial->print(F("|0x"));
    aSerial->print((uint16_t) __brkval, HEX);

    aSerial->print(F(", __malloc_margin="));
    aSerial->print((uint16_t) __malloc_margin); // =128

    aSerial->print(F(", SP="));
    aSerial->print((uint16_t) SP);
    aSerial->print(F("|0x"));
    aSerial->print((uint16_t) SP, HEX);

    /*
     * The next 2 entries seems to be always 0
     */
    aSerial->print(F(", __malloc_heap_end="));
    aSerial->print((uint16_t) __malloc_heap_end);

    aSerial->print(F(", __flp="));
    aSerial->print((uint16_t) __flp); // The largest address just not allocated so far / start of available / free heap, initialized at first malloc()
    aSerial->println();
}

/*
 * RAM starts with Data, i.e. variables initialized with values != 0,
 * followed by BSS, i.e. uninitialized variables (which are initialized with 0)
 * and variables not initialized by using attribute "__attribute__((section(".noinit")))".
 * It ends with the heap and the stack.
 *
 * The STACK required for this function is 8 bytes, so available numbers are 8 less than for caller.
 *
 * Sample output:
 * Data+BSS=445. Heap: used=770, max used=1096, available=663. Stack: available=791, used=42, max used=319, unused=188 of current total 833
 * Formulas:
 * Stack available + used = current total
 * Heap available + __malloc_margin (128) = Stack available
 * Data+BSS + Heap max used + Stack unused + Stack max used = RAMSIZE
 */
void printRAMInfo(Print *aSerial) {

    aSerial->print(F("Data+BSS="));
    aSerial->print((uint16_t) &__heap_start - RAMSTART);

    aSerial->print(F(". Heap: used="));
    aSerial->print((uint16_t) getStartOfAvailableHeap() - (uint16_t) &__heap_start);
    aSerial->print(F(", max written=")); // if Stack uses total heap, we see the stack size here :-(
    aSerial->print(getHeapMaxUsedSize());
    aSerial->print(F(", max available="));
    aSerial->print(RAMEND - (uint16_t) getStartOfAvailableHeap() + 1 - (uint16_t) __malloc_margin);

    aSerial->print(F(". Stack: available="));
    aSerial->print(SP - (uint16_t) getStartOfAvailableHeap() + 1);
    aSerial->print(F(", used="));
    aSerial->print(RAMEND - SP);
    uint16_t tStackUnusedBytes;
    aSerial->print(F(", max used="));
    aSerial->print(getStackMaxUsedAndUnusedSizes(&tStackUnusedBytes));
    aSerial->print(F(", unused="));
    aSerial->print(tStackUnusedBytes);
    aSerial->print(F(" of current total "));
    aSerial->print((RAMEND + 1) - (uint16_t) getStartOfAvailableHeap()); // getStartOfAvailableHeap()

    aSerial->println();
}

/*
 * The minimal margin from Heap End to to Stack Start for malloc()
 * use set__malloc_margin(DEFAULT_MALLOC_MARGIN - <value of unused stack>);
 */
void set__malloc_margin(uint8_t aNewMallocMargin) {
    __malloc_margin = aNewMallocMargin; // default __malloc_margin is 128
}

void reset__malloc_margin() {
    __malloc_margin = DEFAULT_MALLOC_MARGIN; // 128
}

bool isAddressInRAM(void *aAddressToCheck) {
    return (aAddressToCheck <= (void*) RAMEND);
}

bool isAddressBelowAvailableHeapStart(void *aAddressToCheck) {
    return (aAddressToCheck < getStartOfAvailableHeap());
}

/*
 * Test available heap by callocing 128 bytes chunks,
 * If no memory available, try with 64, 32 etc up to 2, 1 byte chunks
 */
void testCallocSizesAndPrint(Print *aSerial) {
    uint8_t *tLastMallocPtr;
    uint16_t tMallocSize = 128;
    while (true) {
        aSerial->print(F("SP=0x"));
        aSerial->print(SP, HEX);
        aSerial->print(F(" available="));
        aSerial->print(SP - (uint16_t) __brkval + 1 - ((uint16_t) __malloc_margin)- HEURISTIC_ADDITIONAL_MALLOC_MARGIN);
        aSerial->print(F(" max available="));
        aSerial->print(RAMEND - (uint16_t) __brkval + 1 - ((uint16_t) __malloc_margin));
        uint8_t *tMallocPtr = (uint8_t*) calloc(tMallocSize, 1);

        aSerial->print(F(" -> calloc("));
        aSerial->print(tMallocSize);
        aSerial->print(F(",1)"));

        if (tMallocPtr == nullptr) {
            aSerial->print(F("failed ->"));
            tMallocSize = tMallocSize >> 1;
            if (tMallocSize < 1) {
                aSerial->println();
                break;
            }
        } else {
            tLastMallocPtr = tMallocPtr;
            aSerial->print(F("=0x"));
            aSerial->print((uint16_t) tLastMallocPtr, HEX);
            aSerial->print(F(" ->"));

            *tLastMallocPtr = HEAP_STACK_UNTOUCHED_VALUE; // For testing detection using 2 consecutive HEAP_STACK_UNTOUCHED_VALUE
            *(tLastMallocPtr + tMallocSize - 1) = 0x11;
        }
        printCurrentAvailableHeapSizeSimple(aSerial);
    }
}
/********************************************
 * SLEEP AND WATCHDOG STUFF
 ********************************************/

#ifndef _MILLIS_UTILS_H
// copied from MillisUtils.h
/*
 * storage for millis value to enable compensation for interrupt disable at signal acquisition etc.
 */
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)  || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
#define timer0_millis millis_timer_millis // The ATTinyCore libraries use other variable name in wiring.c
#endif

extern volatile unsigned long timer0_millis;
#endif // MILLIS_UTILS_H_

/*
 * For sleep modes see sleep.h
 * SLEEP_MODE_IDLE
 * SLEEP_MODE_ADC
 * SLEEP_MODE_PWR_DOWN
 * SLEEP_MODE_PWR_SAVE
 * SLEEP_MODE_STANDBY
 * SLEEP_MODE_EXT_STANDBY
 */
// required only once
void initSleep(uint8_t tSleepMode) {
    sleep_enable();
    set_sleep_mode(tSleepMode);
}

/*
 * Watchdog wakes CPU periodically and all we have to do is call sleep_cpu();
 * aWatchdogPrescaler (see wdt.h) can be one of
 * WDTO_15MS, 30, 60, 120, 250, WDTO_500MS
 * WDTO_1S to WDTO_8S
 */
void initPeriodicSleepWithWatchdog(uint8_t tSleepMode, uint8_t aWatchdogPrescaler) {
    sleep_enable()
    ;
    set_sleep_mode(tSleepMode);
    MCUSR = ~_BV(WDRF); // Clear WDRF in MCUSR

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) \
    || defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) \
    || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
#define WDTCSR  WDTCR
#endif
    // Watchdog interrupt enable + reset interrupt flag -> needs ISR(WDT_vect)
    uint8_t tWDTCSR = _BV(WDIE) | _BV(WDIF) | (aWatchdogPrescaler & 0x08 ? _WD_PS3_MASK : 0x00) | (aWatchdogPrescaler & 0x07); // handles that the WDP3 bit is in bit 5 of the WDTCSR register,
    WDTCSR = _BV(WDCE) | _BV(WDE); // clear lock bit for 4 cycles by writing 1 to WDCE AND WDE
    WDTCSR = tWDTCSR; // set final Value
}

/*
 * @param aWatchdogPrescaler (see wdt.h) can be one of WDTO_15MS, 30, 60, 120, 250, WDTO_500MS, WDTO_1S to WDTO_8S
 *                           0 (15 ms) to 3(120 ms), 4 (250 ms) up to 9 (8000 ms)
 */
uint16_t computeSleepMillis(uint8_t aWatchdogPrescaler) {
    uint16_t tResultMillis = 8000;
    for (uint8_t i = 0; i < (9 - aWatchdogPrescaler); ++i) {
        tResultMillis = tResultMillis / 2;
    }
    return tResultMillis + DEFAULT_MILLIS_FOR_WAKEUP_AFTER_POWER_DOWN; // + for the (default) startup time. !!! But this depends from Clock Source and sleep mode !!!
}

/*
 * @param aWatchdogPrescaler (see wdt.h) can be one of WDTO_15MS, 30, 60, 120, 250, WDTO_500MS, WDTO_1S to WDTO_8S
 *                           0 (15 ms) to 3(120 ms), 4 (250 ms) up to 9 (8000 ms)
 *                           ! I have see + 30 % deviation from nominal WDT clock!
 * @param aAdjustMillis if true, adjust the Arduino internal millis counter the get quite correct millis()
 * results even after sleep, since the periodic 1 ms timer interrupt is disabled while sleeping.
 * Interrupts are enabled before sleep!
 * !!! Do not forget to call e.g. noTone() or  Serial.flush(); to wait for the last character to be sent, and/or disable interrupt sources before !!!
 */
void sleepWithWatchdog(uint8_t aWatchdogPrescaler, bool aAdjustMillis) {
    MCUSR = 0; // Clear MCUSR to enable a correct interpretation of MCUSR after reset
    ADCSRA &= ~ADEN; // disable ADC just before sleep -> saves 200 uA

    // use wdt_enable() since it handles that the WDP3 bit is in bit 5 of the WDTCSR register
    wdt_enable(aWatchdogPrescaler);

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
#  if !defined(timer0_millis)
#define timer0_millis millis_timer_millis // The ATTinyCore + Digispark libraries use millis_timer_millis in wiring.c
#  endif
#define WDTCSR  WDTCR
#endif
    WDTCSR |= _BV(WDIE) | _BV(WDIF); // Watchdog interrupt enable + reset interrupt flag -> requires ISR(WDT_vect)
    sei();         // Enable interrupts, to get the watchdog interrupt, which will wake us up
    sleep_cpu();   // The watchdog interrupt will wake us up from sleep

    // We wake up here :-)
    wdt_disable(); // Because next interrupt will otherwise lead to a reset, since wdt_enable() sets WDE / Watchdog System Reset Enable
    ADCSRA |= ADEN;

    /*
     * Since timer clock may be disabled adjust millis only if not slept in IDLE mode (SM2...0 bits are 000)
     */
#if defined(SM2)
    if (aAdjustMillis && (SMCR & ((_BV(SM2) | _BV(SM1) | _BV(SM0)))) != 0) {
#elif ! defined(SMCR)
    if (aAdjustMillis && (MCUCR & ((_BV(SM1) | _BV(SM0)))) != 0) {
#else
    if (aAdjustMillis && (SMCR & ((_BV(SM1) | _BV(SM0)))) != 0) {
#endif
        timer0_millis += computeSleepMillis(aWatchdogPrescaler);
    }
}

/*
 * 0 -> %1
 * _BV(CLKPS0) -> %2
 * _BV(CLKPS1) -> %4
 * _BV(CLKPS1) | _BV(CLKPS0) -> 8 etc. up to 256
 */
void setclockDivisionFactor(uint8_t aDivisionBits) {
    CLKPR = _BV(CLKPCE);
    CLKPR = aDivisionBits;
}

#endif // defined(__AVR__)
