/*
 * QuadrupedNeoPixelHelper.cpp
 *
 * The NeopPixel updates must be synchronized with the ServoEasing updates in order not to interfere with the servo pulse generation.
 * Therefore we have a function handleServoTimerInterrupt() which replaces the disabled function of ServoEasing.
 *
 *  Copyright (C) 2022  Armin Joachimsmeyer
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

#include "QuadrupedConfiguration.h"
//#define QUADRUPED_HAS_NEOPIXEL        // Requires additionally 6300 to 6600 bytes
#if defined(QUADRUPED_HAS_NEOPIXEL)

#define SUPPRESS_HPP_WARNING
#include "ServoEasing.h"

#if defined(QUADRUPED_ENABLE_RTTTL)
#include <PlayRtttl.h>
#endif

/*
 * Called every 20 ms.
 * The modified overwritten ServoEasing ISR handling function extended for NeoPixel handling.
 * We have 4 ms for our processing until servo interrupt starts again by call of setTimer1InterruptMarginMicros(4000) above.
 *
 * NeoPixels are handled here, since their show() function blocks interrupts
 * and must therefore be synchronized with the servo pulse generation.
 * The interrupt is not disabled after all servos are stopped like in the original function.
 * This enables to call the NeoPixel update function continuously here.
 * Calling of updateAllServos() is controlled by the misused ICNC1 / Input Capture Noise Canceler flag, which is set by ServoEasing :-).
 *
 * Update all servos from list and check if all servos have stopped.
 */
void handleServoTimerInterrupt() {
#if defined(USE_PCA9685_SERVO_EXPANDER)
    // Otherwise it will hang forever in I2C transfer
    interrupts(); // Enable interrupts
#endif
// Check the (misused) ICNC1 flag, which signals that ServoEasing interrupts were enabled again.
    if (TCCR1B & _BV(ICNC1)) {
        // Flag was set -> call update
        if (updateAllServos()) {
            // All servos have stopped here
            // Do not disable interrupt (we need it for NeoPixels), only reset the flag
            TCCR1B &= ~_BV(ICNC1);    // Reset flag
#if defined(INFO)
            Serial.println(F("All servos stopped, using interrupts now only for NeoPixels update"));
#endif
        }
    }
    handleQuadrupedNeoPixelUpdate();
}

#endif // #if defined(QUADRUPED_HAS_NEOPIXEL)

