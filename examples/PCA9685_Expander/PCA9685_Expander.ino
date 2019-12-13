/*
 * PCA9685_Expander.cpp
 *
 *  Shows smooth linear movement from one servo position to another using PCA9685 expander board.
 *  The PCA9685 library was successfully tested with 3 expander boards.
 *
 *  *****************************************************************************************************************************
 *  !!! Comment out line 40 "#define USE_PCA9685_SERVO_EXPANDER" in ServoEasing.h to make the expander example work !!!
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

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!! Comment out line 40 "#define USE_PCA9685_SERVO_EXPANDER" in ServoEasing.h to make the expander example work !!!
 * Otherwise you will see errors like: "PCA9685_Expander:44:46: error: 'Wire' was not declared in this scope"
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#include "ServoEasing.h"

#define VERSION_EXAMPLE "1.0"

#define INFO // to see serial output of loop

/*
 * This example is the OneServo example with only one modification to figure out that there is almost no difference between using the PCA9685 expander or the default Arduino Servo interface.
 * The PCA9685 library was successfully tested with 3 expander boards :-)
 */
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
#if defined(ARDUINO_SAM_DUE)
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS, &Wire1); // If you use more than one PCA9685 you should modify MAX_EASING_SERVOS at line 88 in ServoEasing.h
#else
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS, &Wire); // If you use more than one PCA9685 you should modify MAX_EASING_SERVOS at line 88 in ServoEasing.h
#endif
/*
 * If you have only one Expander at default address and want to use the Arduino Wire library,
 * you can use the short constructor "ServoEasing Servo1();" instead, so that there is no difference to the OneServo example!
 */
//ServoEasing Servo1;
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__)
    while (!Serial); //delay for Leonardo, but this loops forever for Maple Serial
#endif
#if defined(SERIAL_USB)
    delay(2000); // To be able to connect Serial monitor after reset and before first printout
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    /*
     * Check if I2C communication is possible. If not, we will wait forever at endTransmission.
     */
    Serial.println(F("Try to communicate with PCA9685 Expander by TWI / I2C"));
    Serial.flush();
    Wire.beginTransmission(PCA9685_DEFAULT_ADDRESS);
    if (Wire.endTransmission(true) == 0) {
        Serial.print(F("Found"));
    } else {
        Serial.print(F("Error: Communication with I2C was successful, but found no"));
    }
    Serial.print(F(" I2C device attached at address: 0x"));
    Serial.println(PCA9685_DEFAULT_ADDRESS, HEX);


    Serial.println(F("Attach servo to port 9 of PCA9685 expander"));
    /*
     * Check at least the last call to attach()
     */
    if (Servo1.attach(SERVO1_PIN) == INVALID_SERVO) {
        Serial.print(F("Error attaching servo - maybe MAX_EASING_SERVOS="));
        Serial.print(MAX_EASING_SERVOS);
        Serial.println(F(" is to small to hold all servos"));
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }

    /**************************************************
     * Set servos to start position.
     * This is the position where the movement starts.
     *************************************************/
    Servo1.write(0);


    // Wait for servos to reach start position.
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
#ifdef INFO
    Serial.println(F("Move to 90 degree with 10 degree per second blocking"));
#endif
    Servo1.setSpeed(10);  // This speed is taken if no further speed argument is given.
    Servo1.easeTo(90);

    // Now move faster without any delay between the moves
#ifdef INFO
    Serial.println(F("Move to 180 degree with 30 degree per second using interrupts"));
#endif
    Servo1.startEaseTo(180, 30);
    /*
     * Now you can run your program while the servo is moving.
     * Just let the LED blink for 3 seconds (90 degrees moving by 30 degrees per second).
     */
    for (int i = 0; i < 15; ++i) {
        blinkLED();
    }

    delay(1000);

#ifdef INFO
    Serial.println(F("Move to 45 degree in one second using interrupts"));
#endif
    Servo1.startEaseToD(45, 1000);
    // Blink until servo stops
    while (Servo1.isMoving()) {
        blinkLED();
    }

    delay(1000);

#ifdef INFO
    Serial.println(F("Move to 135 degree and back nonlinear in one second each using interrupts"));
#endif
    Servo1.setEasingType(EASE_CUBIC_IN_OUT);

    for (int i = 0; i < 2; ++i) {
        Servo1.startEaseToD(135, 1000);
        // Must call yield here for the ESP boards, since we have no delay called
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
#ifdef INFO
    Serial.println(F("Move to 180 degree with 50 degree per second blocking"));
#endif
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
#ifdef INFO
    Serial.println(F("Move from 180 to 0 degree with 360 degree per second using interrupts of Timer1"));
#endif
    Servo1.startEaseTo(0, 360, true);
    // Wait for 250 ms. The servo should have moved 90 degree.
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);

    delay(1000);
}
