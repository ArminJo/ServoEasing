/*
 * ThreeServos.cpp
 *
 *  Shows smooth movement from one servo position to another for 3 servos synchronously.
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

#include "ServoEasing.h"

#define VERSION_EXAMPLE "1.0"

const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;
const int SERVO3_PIN = 11;

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // Attach servos to pins
    Serial.println(F("Attach servos"));
    Servo1.attach(SERVO1_PIN);
    Servo2.attach(SERVO2_PIN);
    Servo3.attach(SERVO3_PIN);

    // Set servos to start position.
    Servo1.write(0);
    Servo2.write(0);
    Servo3.write(0);

    // Just wait for servos to reach position
    delay(500);
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

/*
 * For more than 2 servos you need your own interrupt handler, but it is quite simple :-)
 */
void handleTimer_COMPB_Interrupt() {
    bool tServosStopped = Servo1.update();
    tServosStopped = Servo2.update() && tServosStopped;
    tServosStopped = Servo3.update() && tServosStopped;
    if (tServosStopped) {
        // disable only if all servos stopped. This enables independent movements of all servos with this interrupt handler.
        disableServoEasingInterrupt();
    }
}

void loop() {

    /*
     * Move three servos synchronously without interrupt handler
     */
    Serial.println(F("Move to 90/90/180 degree with 20 degree per second with own update loop"));
    Servo1.startEaseTo(90, 20);
    Servo2.startEaseTo(90, 20);
    Servo3.startEaseTo(180, 20);
    /*
     * Copy the longest duration in order to move all servos synchronously.
     */
    //  In this case I know that Servo3.millisForCompleteMove is the maximum
    Servo1.millisForCompleteMove = Servo2.millisForCompleteMove = Servo3.millisForCompleteMove = max(
            max(Servo1.millisForCompleteMove,Servo2.millisForCompleteMove), Servo3.millisForCompleteMove);

    do {
        // here you can call your own program
        delay(REFRESH_INTERVAL / 1000); // optional 20ms delay - REFRESH_INTERVAL is in Microseconds
        Servo1.update();
        Servo2.update();
        /*
         * Since all servos stops at the same time I have to check only one for while()
         * ATTENTION You can not use while (Servo1.update() || Servo2.update() || Servo3.update())
         * since then the second & third will NOT be evaluated if the first one returns true
         */
    } while (!Servo3.update());

    delay(1000);

    /*
     * Move three servos synchronously with interrupt handler
     */
    Serial.println(F("Move to 180/180/0 degree with 30 degree per second using interrupts"));
    Servo3.startEaseTo(0, 30);
    Servo1.startEaseToD(180, Servo3.millisForCompleteMove);
    Servo2.startEaseToD(180, Servo3.millisForCompleteMove, true);
    // In this case I know that all durations are the same

    /*
     * Now you can run your program while the servos are moving.
     * Just let the LED blink until servos stop.
     * Since all servos stops at the same time I have to check only one
     */
    while (Servo1.isMoving()) {
        blinkLED();
    }

    delay(1000);

    /*
     * Move first and second servo synchronously with interrupt handler
     */
    Serial.println(F("Move to 90/90 degree with 40 degree per second using interrupts"));
    Servo1.startEaseTo(90, 80);
    Servo2.startEaseToD(90, Servo1.millisForCompleteMove, true);
    // No timing synchronization needed :-)
    // blink until servo stops
    while (Servo1.isMoving()) {
        blinkLED();
    }

    delay(1000);

    // Move only third servo
    Serial.println(F("Move third to 90 degree with 80 degree per second blocking"));
    Servo3.easeTo(90, 80);

    delay(1000);

    /*
     * Move all 3 servos independently
     */
    Serial.println(F("Move independently to 0/0/0 degree with 80/40/20 degree per second using interrupts"));
    Servo1.startEaseTo(0, 80);
    Servo2.startEaseTo(0, 40);
    Servo3.startEaseTo(0, 20, true);
    // Blink until servos stops
    while (Servo1.isMoving() || Servo2.isMoving() || Servo3.isMoving()) {
        blinkLED();
    }

    delay(2000);

}
