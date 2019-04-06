/*
 * AsymmetricEasing.cpp
 *
 *  Shows asymmetric (end movement is different from start movement) user defined non linear movements for 3 servos synchronously.
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

// forward declarations
float EaseQuarticInAndBackOut(float aPercentageOfCompletion);
float EaseQuarticInAndElasticOut(float aPercentageOfCompletion);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial)
        ; //delay for Leonardo
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // Attach servos to pins
    Serial.println(F("Attach servos"));
    Servo1.attach(SERVO1_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);
    Servo2.attach(SERVO2_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);
    Servo3.attach(SERVO3_PIN, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);

    // Set servos to start position.
    Servo1.write(90);
    Servo2.write(90);
    Servo3.write(90);
    delay(1000);

    Servo1.startEaseToD(45, 1000);
    Servo2.startEaseToD(45, 1000);
    Servo3.startEaseToD(45, 1000);

    Servo1.setEasingType(EASE_USER_DIRECT);
    Servo2.setEasingType(EASE_USER_DIRECT);
    Servo1.registerUserEaseInFunction(EaseQuarticInAndBackOut);
    Servo2.registerUserEaseInFunction(EaseQuarticInAndElasticOut);
#ifndef KEEP_LIBRARY_SMALL
    Servo3.setEasingType(EASE_BOUNCE_OUT);
#else
    Servo3.setEasingType(EASE_USER_DIRECT);
    Servo3.registerUserEaseInOutFunction(EaseOutBounce);
#endif

    delay(500);
}

void loop() {

    uint16_t tSpeed = analogRead(SPEED_IN_PIN);
    setSpeedForAllServos(map(tSpeed, 0, 1023, 5, 150));


    /*
     * Move three servos synchronously without interrupt handler
     */
    Serial.print(F("Move to 135 degree with "));
    Serial.print(Servo1.getSpeed());
    Serial.println(F(" degree per second with own update loop"));
    Servo1.setEaseTo(135, tSpeed);
    Servo2.setEaseTo(135, Servo1.mMillisForCompleteMove);
    Servo3.setEaseTo(135, Servo1.mMillisForCompleteMove);
    // No need to call synchronizeAllServosAndStartInterrupt(false), since in this case I know that all durations are the same

    do {
        // here you can call your own program
        delay(REFRESH_INTERVAL / 1000); // optional 20ms delay - REFRESH_INTERVAL is in Microseconds
    } while (!updateAllServos());

    /*
     * Move three servos synchronously with interrupt handler
     */
    Serial.print(F("Move to 45 degree with "));
    Serial.print(Servo1.getSpeed());
    Serial.println(F(" degree per second using interrupts"));
    Servo1.setEaseTo(45, tSpeed);
    Servo2.setEaseTo(45, Servo1.mMillisForCompleteMove);
    Servo3.startEaseToD(45, Servo1.mMillisForCompleteMove);
    // No need to call synchronizeAllServosAndStartInterrupt(), since in this case I know that all durations are the same
    // Since all servos stops at the same time I have to check only one
    while (Servo3.isMoving()) {
        ; // no delays here to avoid break between forth and back movement
    }
}

/*
 * User defined combined movement
 */
float EaseQuarticInAndBackOut(float aPercentageOfCompletion) {
    if (aPercentageOfCompletion <= 0.5) {
        // Quartic IN
        return (8 * aPercentageOfCompletion * aPercentageOfCompletion * aPercentageOfCompletion * aPercentageOfCompletion);
    } else {
        // Back OUT
        return (1.0 - BackEaseIn(1.0 - aPercentageOfCompletion));
    }
}

float EaseQuarticInAndElasticOut(float aPercentageOfCompletion) {
    if (aPercentageOfCompletion <= 0.5) {
        // Quartic IN
        return (8 * aPercentageOfCompletion * aPercentageOfCompletion * aPercentageOfCompletion * aPercentageOfCompletion);
    } else {
        // Elastic OUT
        return (1.0 - ElasticEaseIn(1.0 - aPercentageOfCompletion));
    }
}
