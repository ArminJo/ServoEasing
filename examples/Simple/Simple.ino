/*
 * Simple.cpp
 *
 *  Shows smooth linear movement from one servo position to another.
 *  Do not use interrupts, therefore it can run on any platform where the Arduino Servo library is available.
 *
 *  Copyright (C) 2019  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ServoEasing.
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
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

#include "ServoEasing.h"

#define VERSION_EXAMPLE "1.1"

#if defined(ESP8266)
const int SERVO1_PIN = 14; // D5
#elif defined(ESP32)
const int SERVO1_PIN = 5;
#else
const int SERVO1_PIN = 9;
#endif

// for ESP32 LED_BUILTIN is defined as: static const uint8_t LED_BUILTIN = 2;
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif

ServoEasing Servo1;

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial)
        ; //delay for Leonardo
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // Attach servo to pin
    Serial.print(F("Attach servo at pin "));
    Serial.println(SERVO1_PIN);
    if (Servo1.attach(SERVO1_PIN) == false) {
        Serial.println(F("Error attaching servo"));
    }

    // Set servo to start position.
    Servo1.write(0);

    // Just wait for servo to reach position
    delay(500);

    // Move slow
    Serial.println(F("Move to 90 degree with 10 degree per second blocking"));
    Servo1.easeTo(90, 10);

    delay(1000);

    // Now move faster
    Servo1.setSpeed(10);  // This speed is taken if no speed argument is given.
    Serial.println(F("Move to 180 degree with 30 degree per second blocking"));
    Servo1.easeTo(180);

    delay(2000);

    Serial.println(F("Move back to 0 degree using cubic easing"));
    Servo1.setEasingType(EASE_CUBIC_IN_OUT);
    Servo1.startEaseTo(0, 30, false);
    /*
     * Now do the updates and wait until servo finished
     */
    do {
        // First do the delay, then check for update, since we are likely called directly after start and there is nothing to move yet
        delay(REFRESH_INTERVAL / 1000); // 20 ms - REFRESH_INTERVAL is in Microseconds
    } while (!Servo1.update());
}

void loop() {
    delay(1000);
}
