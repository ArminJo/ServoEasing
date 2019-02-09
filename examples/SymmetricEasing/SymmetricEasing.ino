/*
 * SymmetricEasing.cpp
 *
 *  Shows symmetric (end movement is mirror of start movement) linear, quadratic and cubic movements for 3 servos synchronously.
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

const int SERVO1_PIN = 10;
const int SERVO2_PIN = 9;
const int SERVO3_PIN = 7;

const int SPEED_IN_PIN = A0;

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial); //delay for Leonardo
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // Attach servos to pins
    Serial.println(F("Attach servos"));
    Servo1.attach(SERVO1_PIN, SG_90_MICROSECONDS_FOR_0_DEGREE, SG_90_MICROSECONDS_FOR_180_DEGREE);
    Servo2.attach(SERVO2_PIN, SG_90_MICROSECONDS_FOR_0_DEGREE, SG_90_MICROSECONDS_FOR_180_DEGREE);
    Servo3.attach(SERVO3_PIN, SG_90_MICROSECONDS_FOR_0_DEGREE, SG_90_MICROSECONDS_FOR_180_DEGREE);

    // Set servos to start position.
    Servo1.write(90);
    Servo2.write(90);
    Servo3.write(90);
    delay(1000);

    Servo1.startEaseToD(45, 1000);
    Servo2.startEaseToD(45, 1000);
    Servo3.startEaseToD(45, 1000);

    Servo1.setEasingType(EASE_LINEAR);
    Servo2.setEasingType(EASE_QUADRATIC);
    Servo3.setEasingType(EASE_CUBIC);

    delay(500);
}

/*
 * For more than 2 servos you need your own interrupt handler, but it is quite simple :-)
 * For 3 nonlinear servos it needs 260 us
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

    uint16_t tSpeed = analogRead(SPEED_IN_PIN);
    tSpeed = map(tSpeed, 0, 1023, 5, 150);

    /*
     * Move three servos synchronously without interrupt handler
     */
    Serial.print(F("Move to 135 degree with "));
    Serial.print(tSpeed);
    Serial.println(F(" degree per second with own update loop"));
    Servo1.startEaseTo(135, tSpeed);
    Servo2.startEaseToD(135, Servo1.millisForCompleteMove);
    Servo3.startEaseToD(135, Servo1.millisForCompleteMove);
    // In this case I know that all durations are the same

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

    /*
     * Move three servos synchronously with interrupt handler
     */
    Serial.print(F("Move to 45 degree with "));
    Serial.print(tSpeed);
    Serial.println(F(" degree per second using interrupts"));
    Servo1.startEaseTo(45, tSpeed);
    Servo2.startEaseToD(45, Servo1.millisForCompleteMove);
    Servo3.startEaseToD(45, Servo1.millisForCompleteMove, true);
    // In this case I know that all durations are the same
    // Since all servos stops at the same time I have to check only one
    while (Servo1.isMoving()) {
        ; // no delays here to avoid break between forth and back movement
    }

}
