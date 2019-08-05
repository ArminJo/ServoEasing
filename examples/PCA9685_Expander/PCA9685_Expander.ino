/*
 * PCA9685_Expander.cpp
 *
 *  Shows smooth linear movement from one servo position to another using PCA9685 Expander Board.
 *
 *  *****************************************************************************************************************************
 *  Comment out line 37 in ServoEasing.h to make the expander example work !!!
 *  Otherwise you will see errors like: "PCA9685_Expander:44:46: error: 'Wire' was not declared in this scope"
 *
 *  To access the library files from your sketch, you have to first use `Sketch/Show Sketch Folder (Ctrl+K)` in the Arduino IDE.
 *  Then navigate to the parallel `libraries` folder and select the library you want to access.
 *  The library files itself are located in the `src` sub-directory.
 *  If you did not yet store the example as your own sketch, then with Ctrl+K you are instantly in the right library folder.
 *  *****************************************************************************************************************************
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

/*
 * !!! Comment out line 37 in ServoEasing.h to make the expander example work !!!
 * Otherwise you will see errors like: "PCA9685_Expander:44:46: error: 'Wire' was not declared in this scope"
 */
#include "ServoEasing.h"

#define VERSION_EXAMPLE "1.0"

const int SERVO1_PIN = 9;

// for ESP32 LED_BUILTIN is defined as static const uint8_t LED_BUILTIN = 2;
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif

/*
 * Constructor to specify the expander address (needed at least if you use more than one expander board)
 * and to specify the I2C implementation library.
 * This can be done for each servo separately, but you can not (yet) mix the 3 different
 * Servo implementation libraries (Arduino Servo, Lightweight Servo and I2C Expansion Board)
 */
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS, &Wire);
/*
 * If you have only one Expander at default address and want to use the Arduino Wire library,
 * you can use the short constructor below instead, so that there is no difference to the OneServo example!
 */
//ServoEasing Servo1;
void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial)
        ; //delay for Leonardo
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    Serial.println(F("Attach servo to port 9 of PCA9685 expander"));
    Servo1.attach(SERVO1_PIN);

    // Set servo to start position.
    Serial.println(F("Try to communicate with PCA9685 Expander by TWI / I2C"));
    Servo1.write(0);
    Serial.println(F("Communication with with PCA9685 Expander was successful"));

    // Just wait for servos to reach position
    delay(500);
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

void loop() {
    // Move slow
    Serial.println(F("Move to 90 degree with 10 degree per second blocking"));
    Servo1.easeTo(90, 10);

    // Now move faster without any delay between the moves
    Serial.println(F("Move to 180 degree with 30 degree per second using interrupts of Timer1"));
    Servo1.startEaseTo(180, 30);
    /*
     * Now you can run your program while the servo is moving.
     * Just let the LED blink for 3 seconds (90 degrees moving by 30 degrees per second).
     */
    for (int i = 0; i < 15; ++i) {
        blinkLED();
    }

    delay(1000);

    Serial.println(F("Move to 45 degree in one second NOT using interrupts of Timer1"));
    Servo1.startEaseToD(45, 1000, false);
    // Blink until servo stops
    uint32_t tLastLEDChangeMillis = 0;
    uint32_t tLastServoUpdateMillis = Servo1.mMillisAtStartMove;
    while (Servo1.isMoving()) {
        /*
         * Update servo position each 20ms
         */
        if (millis() - tLastServoUpdateMillis > (REFRESH_INTERVAL / 1000)) { // 20ms - REFRESH_INTERVAL is in Microseconds
            Servo1.update();
            tLastServoUpdateMillis = millis();
        }

        /*
         * Toggle LED every 50 ms
         */
        if (millis() - tLastLEDChangeMillis > 50) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            tLastLEDChangeMillis = millis();
        }
    }

    delay(1000);

    Serial.println(F("Move to 135 degree and back nonlinear in one second each using interrupts of Timer1"));
    Servo1.setEasingType(EASE_CUBIC_IN_OUT);

    for (int i = 0; i < 2; ++i) {
        Servo1.startEaseToD(135, 1000);
        while (Servo1.isMovingAndCallYield()) {
            ; // no delays here to avoid break between forth and back movement
        }
        Servo1.startEaseToD(45, 1000);
        while (Servo1.isMovingAndCallYield()) {
            ; // no delays here to avoid break between forth and back movement
        }
    }
    Servo1.setEasingType(EASE_LINEAR);

    delay(1000);

    /*
     * The LED goes on if servo reaches 120 degree
     */
    Serial.println(F("Move to 180 degree with 50 degree per second blocking"));
    Servo1.startEaseTo(180, 50);
    while (Servo1.getCurrentAngle() < 120) {
        delay(20); // just wait until angle is above 120 degree
    }
    digitalWrite(LED_BUILTIN, HIGH);
    while (Servo1.isMovingAndCallYield()) {
        ; // wait for servo to stop
    }
    delay(1000);

    /*
     * Very fast move. The LED goes off when servo theoretical reaches 90 degree
     */
    Serial.println(F("Move from 180 to 0 degree with 360 degree per second using interrupts of Timer1"));
    Servo1.startEaseTo(0, 360, true);
    // Wait for 250 ms. The servo should have moved 90 degree.
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);

    delay(1000);
}
