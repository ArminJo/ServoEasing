/*
 * EasyButtonAtInt01.hpp
 *
 * This file can be directly configured and included in one source.
 * Include EasyButtonAtInt01.h file if you need the declarations in a second source file.
 *
 *  Arduino library for handling push buttons connected between ground and INT0 and / or INT1 pin.
 *  INT0 and INT1 are connected to Pin 2 / 3 on most Arduinos (ATmega328), to PB6 / PA3 on ATtiny167 and on ATtinyX5 we have only INT0 at PB2.
 *  The library is totally based on interrupt.
 *  Debouncing is implemented in a not blocking way! It is merely done by ignoring a button change within the debouncing time.
 *  So button state is instantly available without debouncing delay!
 *
 *  Usage:
 *  #define USE_BUTTON_0
 *  #include "EasyButtonAtInt01.hpp"
 *  EasyButton Button0AtPin2(true);
 *
 *  Copyright (C) 2018  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of EasyButtonAtInt01 https://github.com/ArminJo/EasyButtonAtInt01.
 *
 *  EasyButtonAtInt01 is free software: you can redistribute it and/or modify
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

#if defined(__AVR__)
#include <Arduino.h>
#include "EasyButtonAtInt01.h"

/*
 * Usage:
 * #define USE_BUTTON_0  // Enables code for button at INT0 (pin2 on 328P, PB6 on ATtiny167, PB2 on ATtinyX5)
 * #define USE_BUTTON_1  // Enables code for button at INT1 (pin3 on 328P, PA3 on ATtiny167, PCINT0 / PCx for ATtinyX5)
 * #include "EasyButtonAtInt01.hpp"
 * EasyButton Button0AtPin2(true);  // true  -> Button is connected to INT0
 * EasyButton Button0AtPin3(false, &Button3CallbackHandler); // false -> button is not connected to INT0 => connected to INT1
 * ...
 * digitalWrite(LED_BUILTIN, Button0AtPin2.ButtonToggleState); // The value at the first call after first press is true
 * ...
 *
 */

// For external measurement of code timing
//#define MEASURE_EASY_BUTTON_INTERRUPT_TIMING
#if defined(MEASURE_EASY_BUTTON_INTERRUPT_TIMING) || defined(BUTTON_LED_FEEDBACK)
#include "digitalWriteFast.h"
#endif

#if defined(USE_BUTTON_0)
EasyButton *EasyButton::sPointerToButton0ForISR;
#endif
#if defined(USE_BUTTON_1)
EasyButton *EasyButton::sPointerToButton1ForISR;
#endif

// @formatter:off // the eclipse formatter has problems with // comments in undefined code blocks

/*
 * These constructors are deterministic if only one button is enabled
 * If two buttons are enabled they can be taken for the 1. button at INT0
 */
EasyButton::EasyButton() {
#if defined(USE_BUTTON_0)
    init(true);  // 1. button
#else
    init(false); // 2. button
#endif
}
/*
 * The same with aButtonPressCallback
 */
EasyButton::EasyButton(void (*aButtonPressCallback)(bool aButtonToggleState)) {
    ButtonPressCallback = aButtonPressCallback;
#if defined(USE_BUTTON_0)
    init(true);  // 1. button
#else
    init(false); // 2. button
#endif
}

#if !defined(NO_BUTTON_RELEASE_CALLBACK)
EasyButton::EasyButton(void (*aButtonPressCallback)(bool aButtonToggleState),
        void (*aButtonReleaseCallback)(bool aButtonToggleState, uint16_t aButtonPressDurationMillis)) {
    ButtonPressCallback = aButtonPressCallback;
    ButtonReleaseCallback = aButtonReleaseCallback;
#  if defined(USE_BUTTON_0)
    init(true); // 1. button
#  else
    init(false); // 2. button
#  endif
}
#endif // NO_BUTTON_RELEASE_CALLBACK

/*
 * These constructors use the first (bool) parameter to decide which button to take.
 */
#if defined(USE_BUTTON_0) && defined(USE_BUTTON_1)
EasyButton::EasyButton(bool aIsButtonAtINT0)
#else
// Constructor with unused attribute to avoid warnings
EasyButton::EasyButton(bool aIsButtonAtINT0 __attribute__((unused)))
#endif
        {
#if defined(USE_BUTTON_0) && not defined(USE_BUTTON_1)
    init(true); // 1. button
#elif defined(USE_BUTTON_1) && not defined(USE_BUTTON_0)
    init(false); // 2. button
#else
    init(aIsButtonAtINT0);
#endif
}

#if defined(USE_BUTTON_0) && defined(USE_BUTTON_1)
EasyButton::EasyButton(bool aIsButtonAtINT0, void (*aButtonPressCallback)(bool aButtonToggleState))
#else
// Constructor with unused attribute to avoid warnings
EasyButton::EasyButton(bool aIsButtonAtINT0 __attribute__((unused)), void (*aButtonPressCallback)(bool aButtonToggleState))
#endif
        {
    ButtonPressCallback = aButtonPressCallback;
#if defined(USE_BUTTON_0) && not defined(USE_BUTTON_1)
    init(true); // 1. button
#elif defined(USE_BUTTON_1) && not defined(USE_BUTTON_0)
    init(false); // 2. button
#else
    init(aIsButtonAtINT0);
#endif
}

#if !defined(NO_BUTTON_RELEASE_CALLBACK)
#  if defined(USE_BUTTON_0) && defined(USE_BUTTON_1)
EasyButton::EasyButton(bool aIsButtonAtINT0, void (*aButtonPressCallback)(bool aButtonToggleState),
        void (*aButtonReleaseCallback)(bool aButtonToggleState, uint16_t aButtonPressDurationMillis))
#  else
// Constructor with unused attribute to avoid warnings
EasyButton::EasyButton(bool aIsButtonAtINT0 __attribute__((unused)), void (*aButtonPressCallback)(bool aButtonToggleState),
        void (*aButtonReleaseCallback)(bool aButtonToggleState, uint16_t aButtonPressDurationMillis))
#  endif
                {
    ButtonPressCallback = aButtonPressCallback;
    ButtonReleaseCallback = aButtonReleaseCallback;
#  if defined(USE_BUTTON_0) && not defined(USE_BUTTON_1)
init(true); // 1. button
#  elif defined(USE_BUTTON_1) && not defined(USE_BUTTON_0)
init(false); // 2. button
#  else
    init(aIsButtonAtINT0);
#  endif
}
#endif // NO_BUTTON_RELEASE_CALLBACK

/*
 * Sets pin mode to INPUT_PULLUP if not defined(BUTTON_IS_ACTIVE_HIGH) and enables INT0 Interrupt on any logical change.
 */
void EasyButton::init(bool aIsButtonAtINT0) {
    isButtonAtINT0 = aIsButtonAtINT0;
#if defined(MEASURE_EASY_BUTTON_INTERRUPT_TIMING)
    pinModeFast(INTERRUPT_TIMING_OUTPUT_PIN, OUTPUT);
#endif

#if defined(BUTTON_LED_FEEDBACK)
    pinModeFast(BUTTON_LED_FEEDBACK_PIN, OUTPUT);
#endif

#if defined(USE_BUTTON_0) && not defined(USE_BUTTON_1)
    /*
     * Only button 0 requested
     */
    INT0_DDR_PORT &= ~(_BV(INT0_BIT)); // pinModeFast(2, INPUT_PULLUP);
#  if !defined(BUTTON_IS_ACTIVE_HIGH)
    INT0_OUT_PORT |= _BV(INT0_BIT); // enable pullup
#  endif
    sPointerToButton0ForISR = this;
#  if defined(USE_ATTACH_INTERRUPT)
    attachInterrupt(digitalPinToInterrupt(INT0_PIN), &handleINT0Interrupt, CHANGE);
#  else
    EICRA |= (1 << ISC00);  // interrupt on any logical change
    EIFR |= 1 << INTF0;// clear interrupt bit
    EIMSK |= 1 << INT0;// enable interrupt on next change
#  endif //USE_ATTACH_INTERRUPT

#elif defined(USE_BUTTON_1) && not defined(USE_BUTTON_0)
    /*
     * Only button 1 requested
     */
    INT1_DDR_PORT &= ~(_BV(INT1_BIT));
#  if !defined(BUTTON_IS_ACTIVE_HIGH)
    INT1_OUT_PORT |= _BV(INT1_BIT); // enable pullup
#  endif
    sPointerToButton1ForISR = this;

#  if (!defined(ISC10)) || ((defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)) && INT1_PIN != 3)
#    if defined(PCICR)
    PCICR |= 1 << PCIE0; // Enable pin change interrupt for port PA0 to PA7
    PCMSK0 = digitalPinToBitMask(INT1_PIN);
#    else
    // ATtinyX5 no ISC10 flag existent
    GIMSK |= 1 << PCIE;//PCINT enable, we have only one
    PCMSK = digitalPinToBitMask(INT1_PIN);
#    endif
#  elif (INT1_PIN != 3)
    /*
     * ATmega328 (Uno, Nano ) etc. Enable pin change interrupt for port PD0 to PD7 (Arduino pin 0 to 7)
     */
    PCICR |= 1 << PCIE2;
    PCMSK2 = digitalPinToBitMask(INT1_PIN);
#  else
#    if defined(USE_ATTACH_INTERRUPT)
    attachInterrupt(digitalPinToInterrupt(INT1_PIN), &handleINT1Interrupt, CHANGE);
#    else
    EICRA |= (1 << ISC10);  // interrupt on any logical change
    EIFR |= 1 << INTF1;     // clear interrupt bit
    EIMSK |= 1 << INT1;     // enable interrupt on next change
#    endif //USE_ATTACH_INTERRUPT
#  endif // !defined(ISC10)

#elif defined(USE_BUTTON_0) && defined(USE_BUTTON_1)
    /*
     * Both buttons 0 + 1 requested
     */
    if (isButtonAtINT0) {
        /*
         * Button 0
         */
        INT0_DDR_PORT &= ~(_BV(INT0_BIT)); // pinModeFast(2, INPUT);
#  if !defined(BUTTON_IS_ACTIVE_HIGH)
        INT0_OUT_PORT |= _BV(INT0_BIT); // enable pullup
#  endif
        sPointerToButton0ForISR = this;
#  if defined(USE_ATTACH_INTERRUPT)
        attachInterrupt(digitalPinToInterrupt(INT0_PIN), &handleINT0Interrupt, CHANGE);
#  else
        EICRA |= (1 << ISC00);  // interrupt on any logical change
        EIFR |= 1 << INTF0;     // clear interrupt bit
        EIMSK |= 1 << INT0;     // enable interrupt on next change
#  endif //USE_ATTACH_INTERRUPT
    } else {
        /*
         * Button 1
         * Allow PinChangeInterrupt
         */
        INT1_DDR_PORT &= ~(_BV(INT1_BIT));  // pinModeFast(INT1_BIT, INPUT_PULLUP);
#  if !defined(BUTTON_IS_ACTIVE_HIGH)
        INT1_OUT_PORT |= _BV(INT1_BIT); // enable pullup
#  endif
        sPointerToButton1ForISR = this;

#  if (!defined(ISC10)) || ((defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)) && INT1_PIN != 3)
#    if defined(PCICR)
        /*
         * ATtiny167 + 87. Enable pin change interrupt for port PA0 to PA7
         */
        PCICR |= 1 << PCIE0;
        PCMSK0 = digitalPinToBitMask(INT1_PIN);
#    else
        /*
         *ATtinyX5. Enable pin change interrupt for port PB0 to PB5
         */
        GIMSK |= 1 << PCIE; // PCINT enable, we have only one
        PCMSK = digitalPinToBitMask(INT1_PIN);
#    endif
#  elif INT1_PIN == 4 || INT1_PIN == 5 || INT1_PIN == 6 || INT1_PIN == 7
    //ATmega328 (Uno, Nano ) etc. Enable pin change interrupt for port PD0 to PD7 (Arduino pin 0 to 7)
        PCICR |= 1 << PCIE2;
        PCMSK2 = digitalPinToBitMask(INT1_PIN);
#    elif INT1_PIN == 8 || INT1_PIN == 9 || INT1_PIN == 10 || INT1_PIN == 11 || INT1_PIN == 12 || INT1_PIN == 13
    //ATmega328 (Uno, Nano ) etc. Enable pin change interrupt 0 to 5 for port PB0 to PB5 (Arduino pin 8 to 13)
        PCICR |= _BV(PCIE0);
        PCMSK0 = digitalPinToBitMask(INT1_PIN);
#    elif INT1_PIN == A0 || INT1_PIN == A1 || INT1_PIN == A2 || INT1_PIN == A3 || INT1_PIN == A4 || INT1_PIN == A5
    //ATmega328 (Uno, Nano ) etc. Enable pin change interrupt 8 to 13 for port PC0 to PC5 (Arduino pin A0 to A5)
        PCICR |= _BV(PCIE1);
        PCMSK1 = digitalPinToBitMask(INT1_PIN);
#  else
#    if defined(USE_ATTACH_INTERRUPT)
        attachInterrupt(digitalPinToInterrupt(INT1_PIN), &handleINT1Interrupt, CHANGE);
#    else
        EICRA |= (1 << ISC10);  // interrupt on any logical change
        EIFR |= 1 << INTF1;     // clear interrupt bit
        EIMSK |= 1 << INT1;     // enable interrupt on next change
#    endif //USE_ATTACH_INTERRUPT
#  endif // !defined(ISC10)
    }
#endif
    ButtonStateIsActive = false; // negative logic for ButtonStateIsActive! true means button pin is LOW
    ButtonToggleState = false;
}

/*
 * Negative logic for readButtonState() true means button pin is LOW, if button is active low (default)
 */
bool EasyButton::readButtonState() {
#if defined(USE_BUTTON_0) && not defined(USE_BUTTON_1)
#  if defined(BUTTON_IS_ACTIVE_HIGH)
    return (INT0_IN_PORT & _BV(INT0_BIT));  //  = digitalReadFast(2);
#  else
    return !(INT0_IN_PORT & _BV(INT0_BIT));  //  = digitalReadFast(2);
#  endif

#elif defined(USE_BUTTON_1) && not defined(USE_BUTTON_0)
#  if defined(BUTTON_IS_ACTIVE_HIGH)
    return (INT1_IN_PORT & _BV(INT1_BIT));  //  = digitalReadFast(3);
#  else
    return !(INT1_IN_PORT & _BV(INT1_BIT));  //  = digitalReadFast(3);
#  endif

#elif defined(USE_BUTTON_0) && defined(USE_BUTTON_1)
#  if defined(BUTTON_IS_ACTIVE_HIGH)
    if (isButtonAtINT0) {
        return (INT0_IN_PORT & _BV(INT0_BIT));  //  = digitalReadFast(2);
    } else {
        return (INT1_IN_PORT & _BV(INT1_BIT));  //  = digitalReadFast(3);
    }
#  else
    if (isButtonAtINT0) {
        return !(INT0_IN_PORT & _BV(INT0_BIT));  //  = digitalReadFast(2);
    } else {
        return !(INT1_IN_PORT & _BV(INT1_BIT));  //  = digitalReadFast(3);
    }
#  endif
#endif
}

// @formatter:on // the eclipse formatter has problems with // comments in undefined code blocks

/*
 * Returns stored state if in debouncing period otherwise current state of button
 */
bool EasyButton::readDebouncedButtonState() {
    // Check for bouncing period
    if (millis() - ButtonLastChangeMillis <= BUTTON_DEBOUNCING_MILLIS) {
        return ButtonStateIsActive;
    }
    return readButtonState();
}

/*
 * Update button state if state change was not captured by the ISR
 * @return true if state was changed and updated
 */
bool EasyButton::updateButtonState() {
    noInterrupts();
    if (readDebouncedButtonState() != ButtonStateIsActive) {
#ifdef TRACE
        if (LastBounceWasChangeToInactive) {
            Serial.print(F("Updated button state, assume last button press was shorter than debouncing period of "));
            Serial.print(BUTTON_DEBOUNCING_MILLIS);
            Serial.print(F(" ms"));
#ifdef ANALYZE_MAX_BOUNCING_PERIOD
            Serial.print(F(" MaxBouncingPeriod was="));
            Serial.print(MaxBouncingPeriodMillis);
            MaxBouncingPeriodMillis = 0;
#endif
        } else {
            // It can happen, that we just catch the release of the button here, so no worry!
            Serial.print(F("Update button state to "));
            Serial.print(!ButtonStateIsActive);
            Serial.print(F(", current state was not yet caught by ISR"));
        }
        Serial.println();
#endif
        handleINT01Interrupts();
        interrupts();
        return true;
    }
    interrupts();
    return false;
}

/*
 * Updates the ButtonPressDurationMillis by polling, since this cannot be done by interrupt.
 */
uint16_t EasyButton::updateButtonPressDuration() {
    if (readDebouncedButtonState()) {
        // Button still active -> update ButtonPressDurationMillis
        ButtonPressDurationMillis = millis() - ButtonLastChangeMillis;
    }
    return ButtonPressDurationMillis;
}

/*
 * Used for long button press recognition, while button is still pressed!
 * !!! Consider to use button release callback handler and check the ButtonPressDurationMillis
 * returns EASY_BUTTON_LONG_PRESS_DETECTED, EASY_BUTTON_LONG_PRESS_STILL_POSSIBLE and EASY_BUTTON_LONG_PRESS_ABORT
 */
uint8_t EasyButton::checkForLongPress(uint16_t aLongPressThresholdMillis) {
    uint8_t tRetvale = EASY_BUTTON_LONG_PRESS_ABORT;
    if (readDebouncedButtonState()) {
        // Button still active -> update current ButtonPressDurationMillis
        // noInterrupts() is required, since otherwise we may get wrong results if interrupted during load of long value by button ISR
        noInterrupts();
        ButtonPressDurationMillis = millis() - ButtonLastChangeMillis;
        interrupts();
        tRetvale = EASY_BUTTON_LONG_PRESS_STILL_POSSIBLE; // if not detected, you may try again
    }
    if (ButtonPressDurationMillis >= aLongPressThresholdMillis) {
        // long press detected
        return EASY_BUTTON_LONG_PRESS_DETECTED;
    }

    return tRetvale;
}

/*
 * Checks for long press of button
 * Blocks until long press threshold is reached or button was released.
 * !!! Consider to use button release callback handler and check the ButtonPressDurationMillis
 * @return true if long press was detected - only once for each long press
 */
bool EasyButton::checkForLongPressBlocking(uint16_t aLongPressThresholdMillis) {
    /*
     * wait as long as button is pressed shorter than threshold millis.
     */
    while (checkForLongPress(aLongPressThresholdMillis) == EASY_BUTTON_LONG_PRESS_STILL_POSSIBLE) {
        delay(1);
    }
    /*
     * Here button was not pressed or time was greater than threshold.
     * ButtonPressDurationMillis was updated by call to checkForLongPress before
     */
    return (ButtonPressDurationMillis >= aLongPressThresholdMillis);
}

/*
 * Double press detection by computing difference between current (active) timestamp ButtonLastChangeMillis
 * and last release timestamp ButtonReleaseMillis.
 * !!!Works only reliable if called early in ButtonPress callback function!!!
 * Be aware, that the first press after booting may be detected as double press!
 * This is because ButtonReleaseMillis is initialized with 0 milliseconds, which is interpreted as the first press happened at the beginning of boot.
 * @return true if double press detected.
 */
bool EasyButton::checkForDoublePress(uint16_t aDoublePressDelayMillis) {
    unsigned long tReleaseToPressTimeMillis = ButtonLastChangeMillis - ButtonReleaseMillis;
    return (tReleaseToPressTimeMillis <= aDoublePressDelayMillis);
}

/*
 * Checks if button was not pressed in the last aTimeoutMillis
 * Can be used to recognize timeout for user button actions
 * @return true if timeout reached: false if last button release was before aTimeoutMillis
 */
bool EasyButton::checkForForButtonNotPressedTime(uint16_t aTimeoutMillis) {
    // noInterrupts() is required, since otherwise we may get wrong results if interrupted during load of long value by button ISR
    noInterrupts();
    unsigned long tButtonReleaseMillis = ButtonReleaseMillis;
    interrupts();
    return (millis() - tButtonReleaseMillis >= aTimeoutMillis);
}

/*
 * 1. Read button pin level and invert logic level since we have negative logic because of using pullups.
 * 2. Check for bouncing - state change during debounce period. We need millis() to be enabled to run in the background.
 * 3. Check for spikes - interrupts but no level change.
 * 4. Process valid button state change. If callback requested, call callback routine, get button pin level again and handle if button was released in the meantime.
 */
void EasyButton::handleINT01Interrupts() {
    // Read button value
    bool tCurrentButtonStateIsActive;

    /*
     * This is faster than readButtonState();
     */
#if defined(USE_BUTTON_0) && not defined(USE_BUTTON_1)
    tCurrentButtonStateIsActive = INT0_IN_PORT & _BV(INT0_BIT);  //  = digitalReadFast(2);

#elif defined(USE_BUTTON_1) && not defined(USE_BUTTON_0)
    tCurrentButtonStateIsActive = INT1_IN_PORT & _BV(INT1_BIT);  //  = digitalReadFast(3);

#elif defined(USE_BUTTON_0) && defined(USE_BUTTON_1)
    if (isButtonAtINT0) {
        tCurrentButtonStateIsActive = INT0_IN_PORT & _BV(INT0_BIT);  //  = digitalReadFast(2);
    } else {
        tCurrentButtonStateIsActive = INT1_IN_PORT & _BV(INT1_BIT);  //  = digitalReadFast(3);
    }
#endif
#if !defined(BUTTON_IS_ACTIVE_HIGH)
    tCurrentButtonStateIsActive = !tCurrentButtonStateIsActive; // negative logic for tCurrentButtonStateIsActive! true means button pin is LOW
#endif

#ifdef TRACE
    Serial.print(tCurrentButtonStateIsActive);
    Serial.print('-');
#endif

    unsigned long tMillis = millis();
    unsigned int tDeltaMillis = tMillis - ButtonLastChangeMillis;
    // Check for bouncing - state change during debounce period
    if (tDeltaMillis <= BUTTON_DEBOUNCING_MILLIS) {
        /*
         * Button is bouncing, signal is ringing - do nothing, ignore and wait for next interrupt
         */
#ifdef ANALYZE_MAX_BOUNCING_PERIOD
        if (MaxBouncingPeriodMillis < tDeltaMillis) {
            MaxBouncingPeriodMillis = tDeltaMillis;
            Serial.print(F("Bouncing, MBP="));
            Serial.println(MaxBouncingPeriodMillis);
            //        Serial.print(F("ms="));
            //        Serial.print(tMillis);
            //        Serial.print(F(" D="));
            //        Serial.println(tDeltaMillis);
        }
#endif
        if (tCurrentButtonStateIsActive) {
            LastBounceWasChangeToInactive = false;
        } else {
            /*
             * Store, that switch goes inactive during debouncing period.
             * This may be a bouncing issue (fine) but it can also be a very short button press.
             * In this case we do not set the ButtonStateIsActive to false because we are in debouncing period.
             * On the next press this will be detected as a spike, if not considered.
             */
            LastBounceWasChangeToInactive = true;
        }

    } else {
        /*
         * Here we are after debouncing period
         */
        if (tCurrentButtonStateIsActive == ButtonStateIsActive) {
            /*
             * No valid change detected - current is equals last state
             */
            if (tCurrentButtonStateIsActive && LastBounceWasChangeToInactive) {
                // We assume we had a very short press before (or a strange spike), which was handled as a bounce. -> must adjust last button state
                ButtonStateIsActive = false;
#ifdef TRACE
                Serial.println(F("Preceding short press detected, which was handled as bounce"));
#endif

            } else {
                /*
                 * tCurrentButtonStateIsActive == OldButtonStateIsActive. We had an interrupt, but nothing seems to have changed -> spike
                 * Do nothing, ignore and wait for next interrupt
                 */
#ifdef TRACE
                Serial.println(F("Spike"));
#endif
            }
        }

        // do not use else since we may have changed ButtonStateIsActive
        if (tCurrentButtonStateIsActive != ButtonStateIsActive) {
            /*
             * Valid change detected
             */
            ButtonLastChangeMillis = tMillis;
            LastBounceWasChangeToInactive = false;
#ifdef TRACE
            Serial.println(F("Change"));
#endif
            ButtonStateIsActive = tCurrentButtonStateIsActive;
            ButtonStateHasJustChanged = true;
            if (tCurrentButtonStateIsActive) {
                /*
                 * Button pressed
                 */
#ifdef BUTTON_LED_FEEDBACK
                digitalWriteFast(BUTTON_LED_FEEDBACK_PIN, HIGH);
#endif
                ButtonToggleState = !ButtonToggleState;
                if (ButtonPressCallback != NULL) {
                    /*
                     * Call callback function.
                     * interrupts() is required if callback function needs more time to allow millis() to proceed.
                     * Otherwise we may see bouncing instead of button release followed by spike instead of button press
                     */
                    interrupts();
                    ButtonPressCallback(ButtonToggleState);
                    /*
                     * Check button again since it may changed back while processing callback function
                     */
                    if (!readButtonState()) {
                        // button released now, so maintain status
#ifdef TRACE
                        Serial.println(F("Button release during callback processing detected."));
#endif
                        ButtonStateIsActive = false;
                        tMillis = millis();
                        ButtonPressDurationMillis = tMillis - ButtonLastChangeMillis;
                        ButtonLastChangeMillis = tMillis;
                        ButtonStateHasJustChanged = true;
                        ButtonReleaseMillis = tMillis;
                    }
                }
            } else {
                /*
                 * Button release
                 */
                ButtonPressDurationMillis = tDeltaMillis;
                ButtonReleaseMillis = tMillis;
#if !defined(NO_BUTTON_RELEASE_CALLBACK)
                if (ButtonReleaseCallback != NULL) {
                    /*
                     * Call callback function.
                     * interrupts() is required if callback function needs more time to allow millis() to proceed.
                     */
                    interrupts();
                    ButtonReleaseCallback(ButtonToggleState, ButtonPressDurationMillis);
                    /*
                     * Check button again since it may be activated while processing callback function
                     */
                    if (readButtonState()) {
                        // button activated now, so maintain status
#  ifdef TRACE
                        Serial.println(F("Button active after callback processing detected."));
#  endif
                        ButtonStateIsActive = true;
                        ButtonLastChangeMillis = millis();
                        ButtonStateHasJustChanged = true;
                        LastBounceWasChangeToInactive = false;
                    }
                }
#endif
#ifdef BUTTON_LED_FEEDBACK
                digitalWriteFast(BUTTON_LED_FEEDBACK_PIN, LOW);
#endif
            }
        }
    }
}

// end of class definitions

/*
 * This functions are weak and can be replaced by your own code
 */
#if defined(USE_BUTTON_0)
void __attribute__ ((weak)) handleINT0Interrupt() {
    EasyButton::sPointerToButton0ForISR->handleINT01Interrupts();
}
#endif

#if defined(USE_BUTTON_1)
void __attribute__ ((weak)) handleINT1Interrupt() {
    EasyButton::sPointerToButton1ForISR->handleINT01Interrupts();
}
#endif

#if not defined(USE_ATTACH_INTERRUPT)
// ISR for PIN PD2
// Cannot make the vector itself weak, since the vector table is already filled by weak vectors resulting in ignoring my weak one:-(
//ISR(INT0_vect, __attribute__ ((weak))) {
#  if defined(USE_BUTTON_0)
ISR(INT0_vect) {
#    ifdef MEASURE_EASY_BUTTON_INTERRUPT_TIMING
    digitalWriteFast(INTERRUPT_TIMING_OUTPUT_PIN, HIGH);
#    endif
    handleINT0Interrupt();
#    ifdef MEASURE_EASY_BUTTON_INTERRUPT_TIMING
    digitalWriteFast(INTERRUPT_TIMING_OUTPUT_PIN, LOW);
#    endif
}
#  endif

#  if defined(USE_BUTTON_1)
#    if (!defined(ISC10)) || ((defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)) && INT1_PIN != 3)
// on ATtinyX5 we do not have a INT1_vect but we can use the PCINT0_vect
ISR(PCINT0_vect)
#    elif INT1_PIN == 4 || INT1_PIN == 5 || INT1_PIN == 6 || INT1_PIN == 7
// PCINT for ATmega328 Arduino pins 0 to 7
ISR(PCINT2_vect)
#    elif INT1_PIN == 8 || INT1_PIN == 9 || INT1_PIN == 10 || INT1_PIN == 11 || INT1_PIN == 12 || INT1_PIN == 13
// PCINT for ATmega328 Arduino pins 8 (PB0) to 13 (PB5) - (PCINT 0 to 5)
ISR(PCINT0_vect)
#    elif INT1_PIN == A0 || INT1_PIN == A1 || INT1_PIN == A2 || INT1_PIN == A3 || INT1_PIN == A4 || INT1_PIN == A5
// PCINT for ATmega328 Arduino pins A1 (PC0) to A5 (PC5) - (PCINT 8 to 13)
ISR(PCINT1_vect)
#    else
ISR(INT1_vect)
#    endif
{
#    ifdef MEASURE_EASY_BUTTON_INTERRUPT_TIMING
    digitalWriteFast(INTERRUPT_TIMING_OUTPUT_PIN, HIGH);
#    endif
    handleINT1Interrupt();
#    ifdef MEASURE_EASY_BUTTON_INTERRUPT_TIMING
    digitalWriteFast(INTERRUPT_TIMING_OUTPUT_PIN, LOW);
#    endif
}
#  endif
#endif // not defined(USE_ATTACH_INTERRUPT)

#endif // defined(__AVR__)
