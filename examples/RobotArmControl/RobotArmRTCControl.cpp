/*
 * RobotArmRTCControl.cpp
 *
 * RTC related code for Program for RobotArm.
 *
 * To run this example need to install the "ServoEasing", "IRLremote" and "PinChangeInterrupt" libraries under "Tools -> Manage Libraries..." or "Ctrl+Shift+I"
 * Use "ServoEasing", "IRLremote" and "PinChangeInterrupt" as filter string.
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

#include "RobotArmRTCControl.h"

uRTCLib RTC_DS3231;

void initRTC() {
    Wire.begin();
    RTC_DS3231.set_model(URTCLIB_MODEL_DS3231);
    Serial.println("Set date");
    RTC_DS3231.set(0, 58, 18, 4, 14, 8, 19);
//  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
}

/*
 * Only URTCLIB_SQWG_OFF_1 and URTCLIB_SQWG_1H work on my china module
 */
void testRTC() {
    Serial.println("Testing SQWG/INT output:");

    Serial.println("fixed 1:");
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_OFF_1);
    delay(2000);

    Serial.println("1 hertz:");
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_1H);
    delay(3000);

    Serial.println("1024 hertz:");
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_1024H);
    delay(2000);

    Serial.println("4096 hertz:");
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_4096H);
    delay(2000);

    Serial.println("8192 hertz:");
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_8192H);
    delay(2000);

}

void printRTC() {
    static long sLastMillisOfRTCRead;

    if (millis() - sLastMillisOfRTCRead >= 1000) {
        sLastMillisOfRTCRead = millis();
        RTC_DS3231.refresh();

        Serial.print("RTC DateTime: ");
        Serial.print(RTC_DS3231.day());
        Serial.print('.');
        Serial.print(RTC_DS3231.month());
        Serial.print('.');
        Serial.print(RTC_DS3231.year());

        Serial.print(' ');

        Serial.print(RTC_DS3231.hour());
        Serial.print(':');
        Serial.print(RTC_DS3231.minute());
        Serial.print(':');
        Serial.print(RTC_DS3231.second());

        Serial.print(" DOW: ");
        Serial.print(RTC_DS3231.dayOfWeek());

        Serial.print(" - Temp: ");
        Serial.print(RTC_DS3231.temp());

        Serial.println();
    }
}
