/*
 * RobotArmRTCControl.hpp
 *
 * RTC related code for RobotArmControl.
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef _ROBOT_ARM_RTC_CONTROL_HPP
#define _ROBOT_ARM_RTC_CONTROL_HPP

#include <Arduino.h>

#include "RobotArmRTCControl.h"

uRTCLib RTC_DS3231;

bool requestDateBySerialAndSet(void) {

    uint8_t tSecond, tMinute, tHour, tDayOfMonth, tMonth, tYear;
    char tInputBuffer[17];
    Serial.println();
    Serial.println(F("Enter the time in format: \"HH MM SS DD MM YY\" - Timeout is 30 seconds"));
    Serial.setTimeout(30000);
    // read exactly 17 characters
    size_t tReadLength = Serial.readBytes(tInputBuffer, 17);
    if (tReadLength == 17 && tInputBuffer[14] == ' ') {
        sscanf_P(tInputBuffer, PSTR("%2hhu %2hhu %2hhu %2hhu %2hhu %2hhu"), &tHour, &tMinute, &tSecond, &tDayOfMonth, &tMonth,
                &tYear);
        // read newline etc.
        while (Serial.available()) {
            Serial.read();
        }
        Serial.setTimeout(10000);
        Serial.println();
        Serial.println(F("Enter the day of week as number between 1 and 7 (Sunday-Saturday) - Timeout is 10 seconds"));
        Serial.readBytes(tInputBuffer, 1);
        if (tInputBuffer[0] > '0' && tInputBuffer[0] < '8') {
            RTC_DS3231.set(tSecond, tMinute, tHour, tInputBuffer[0] - '0', tDayOfMonth, tMonth, tYear);
            Serial.print(F("Time set to: "));
            printRTC();
            return true;
        }
    }
    Serial.println(F("Clock has not been changed, press reset for next try."));
    return false;
}

void initRTC() {
    Wire.begin();
    RTC_DS3231.set_model(URTCLIB_MODEL_DS3231);

    if (RTC_DS3231.lostPower() || RTC_DS3231.year() < 19) {
        if (requestDateBySerialAndSet()) {
            RTC_DS3231.lostPowerClear();
        }
    }
}

#if !defined(__AVR__) && ! defined(PROGMEM)
#define PROGMEM
#endif

//#define GERMAN_NAMES_FOR_DATE

#if defined(GERMAN_NAMES_FOR_DATE)
const char Sunday[] PROGMEM= "Sonntag";
const char Monday[] PROGMEM= "Montag";
const char Tuesday[] PROGMEM= "Dienstag";
const char Wednesday[] PROGMEM= "Mittwoch";
const char Thursday[] PROGMEM= "Donnerstag";
const char Friday[] PROGMEM= "Freitag";
const char Saturday[] PROGMEM= "Samstag";

const char January[] PROGMEM= "Januar";
const char February[] PROGMEM= "Februar";
const char March[] PROGMEM= "März";
const char April[] PROGMEM= "April";
const char May[] PROGMEM= "Mai";
const char June[] PROGMEM= "Juni";
const char July[] PROGMEM= "July";
const char August[] PROGMEM= "August";
const char September[] PROGMEM= "September";
const char October[] PROGMEM= "Oktober";
const char November[] PROGMEM= "November";
const char December[] PROGMEM= "Dezember";
#else
const char Sunday[] PROGMEM ="Sunday";
const char Monday[] PROGMEM ="Monday";
const char Tuesday[] PROGMEM ="Tuesday";
const char Wednesday[] PROGMEM ="Wednesday";
const char Thursday[] PROGMEM ="Thursday";
const char Friday[] PROGMEM ="Friday";
const char Saturday[] PROGMEM ="Saturday";

const char January[] PROGMEM ="January";
const char February[] PROGMEM = "February";
const char March[] PROGMEM = "March";
const char April[] PROGMEM = "April";
const char May[] PROGMEM = "May";
const char June[] PROGMEM = "June";
const char July[] PROGMEM = "July";
const char August[] PROGMEM = "August";
const char September[] PROGMEM = "September";
const char October[] PROGMEM = "October";
const char November[] PROGMEM ="November";
const char December[] PROGMEM = "December";
#endif

const char* const sDayStrings[] PROGMEM = { Sunday, Monday, Tuesday, Wednesday, Thursday, Friday, Saturday };
#define DOW_MAX_STRING_SIZE 11 // excluding trailing NUL (Donnerstag)

const char* const sMonthStrings[] PROGMEM = { January, February, March, April, May, June, July, August, September, October,
        November, December };
#define MONTH_MAX_STRING_SIZE 10 // excluding trailing NUL (September)

/*
 * @return true if update/refresh happened
 */
bool printRTCEveryPeriod(uint16_t aPeriodSeconds, uint8_t aDateFormatSpecifier) {
    static long sLastMillisOfRTCRead;
    if (millis() - sLastMillisOfRTCRead >= (1000 * aPeriodSeconds)) {
        sLastMillisOfRTCRead = millis();
        RTC_DS3231.refresh();
        printRTC(aDateFormatSpecifier);
        return true;
    }
    return false;
}

void printRTC(uint8_t aDateFormatSpecifier) {
    RTC_DS3231.refresh();
    printRTCDate(aDateFormatSpecifier);
    Serial.print(' ');
    printRTCTime((aDateFormatSpecifier & DATE_FORMAT_LONG_MASK), false);
    Serial.println();
}

void printRTCDateAmericanFormat(bool aPrintLongFormat) {
#if defined(__AVR__)
    char tDateString[(2 * DOW_MAX_STRING_SIZE) + 1 + (2 * MONTH_MAX_STRING_SIZE) + 1 + 12];
#else
    char tDateString[DOW_MAX_STRING_SIZE + MONTH_MAX_STRING_SIZE + 12];
#endif

    if (aPrintLongFormat) {
#if defined(__AVR__)
        // fist copy day of week
        strcpy_P(&tDateString[sizeof(tDateString) - (DOW_MAX_STRING_SIZE + MONTH_MAX_STRING_SIZE) - 3],
                (PGM_P) pgm_read_word(&sDayStrings[RTC_DS3231.dayOfWeek() - 1]));
        strcpy_P(&tDateString[sizeof(tDateString) - (MONTH_MAX_STRING_SIZE) - 2],
                (PGM_P) pgm_read_word(&sMonthStrings[RTC_DS3231.month() - 1]));
        sprintf_P(tDateString, PSTR("%s, %s %2hhu, 20%2hhu"),
                &tDateString[sizeof(tDateString) - (DOW_MAX_STRING_SIZE + MONTH_MAX_STRING_SIZE) - 3],
                &tDateString[sizeof(tDateString) - (MONTH_MAX_STRING_SIZE) - 2], RTC_DS3231.day(), RTC_DS3231.year());
#else
        sprintf(tDateString, "%s, %s %2hhu, 20%2hhu", sDayStrings[RTC_DS3231.dayOfWeek() - 1],
                sMonthStrings[RTC_DS3231.month() - 1], RTC_DS3231.day(), RTC_DS3231.year());
#endif
    } else {
        sprintf_P(tDateString, PSTR("%02hhu/%20hhu/20%2hhu"), RTC_DS3231.month(), RTC_DS3231.day(), RTC_DS3231.year());
    }
    Serial.print(tDateString);
}

void printRTCDateEuropeanFormat(bool aPrintLongFormat) {
#if defined(__AVR__)
    char tDateString[(2 * DOW_MAX_STRING_SIZE) + 1 + (2 * MONTH_MAX_STRING_SIZE) + 1 + 12];
#else
    char tDateString[DOW_MAX_STRING_SIZE + MONTH_MAX_STRING_SIZE + 12];
#endif

    if (aPrintLongFormat) {
#if defined(__AVR__)
        // fist copy day of week
        strcpy_P(&tDateString[sizeof(tDateString) - (DOW_MAX_STRING_SIZE + MONTH_MAX_STRING_SIZE) - 3],
                (PGM_P) pgm_read_word(&sDayStrings[RTC_DS3231.dayOfWeek() - 1]));
        strcpy_P(&tDateString[sizeof(tDateString) - (MONTH_MAX_STRING_SIZE) - 2],
                (PGM_P) pgm_read_word(&sMonthStrings[RTC_DS3231.month() - 1]));
        sprintf_P(tDateString, PSTR("%s, %2hhu. %s 20%2hhu"),
                &tDateString[sizeof(tDateString) - (DOW_MAX_STRING_SIZE + MONTH_MAX_STRING_SIZE) - 3], RTC_DS3231.day(),
                &tDateString[sizeof(tDateString) - (MONTH_MAX_STRING_SIZE) - 2], RTC_DS3231.year());
#else
        sprintf(tDateString, "%s, %2hhu. %s 20%2hhu", sDayStrings[RTC_DS3231.dayOfWeek() - 1], RTC_DS3231.day(),
                sMonthStrings[RTC_DS3231.month() - 1], RTC_DS3231.year());
#endif
    } else {
        sprintf_P(tDateString, PSTR("%02hhu.%02hhu.20%2hhu"), RTC_DS3231.day(), RTC_DS3231.month(), RTC_DS3231.year());
    }
    Serial.print(tDateString);
}

void printRTCDateISOFormat() {
    char tDateString[11];
#if defined(__AVR__)
    sprintf_P(tDateString, PSTR("20%2hhu-%02hhu-%02hhu"), RTC_DS3231.year(), RTC_DS3231.month(), RTC_DS3231.day());
#else
    sprintf(tDateString, "20%2hhu-%02hhu-%02hhu", RTC_DS3231.year(), RTC_DS3231.month(), RTC_DS3231.day());
#endif
    Serial.print(tDateString);
}

void printRTCDate(uint8_t aDateFormatSpecifier) {

    if (aDateFormatSpecifier & DATE_FORMAT_AMERICAN) {
        printRTCDateAmericanFormat((aDateFormatSpecifier & DATE_FORMAT_LONG_MASK));
    } else if (aDateFormatSpecifier & DATE_FORMAT_EUROPEAN) {
        printRTCDateEuropeanFormat((aDateFormatSpecifier & DATE_FORMAT_LONG_MASK));
    } else {
// ISO Format
        printRTCDateISOFormat();
    }
}

void printRTCTemperature() {
    Serial.print(RTC_DS3231.temp() / 100);
    Serial.print('.');
    Serial.print(RTC_DS3231.temp() - ((RTC_DS3231.temp() / 100) * 100));
}

void printRTCTime(bool aPrintLongFormat, bool aDoRefresh) {
    if (aDoRefresh) {
        RTC_DS3231.refresh();
    }
    char tTimeString[9]; // 8 + trailing NUL character
#if defined(__AVR__)
    if (aPrintLongFormat) {
        sprintf_P(tTimeString, PSTR("%02hhu:%02hhu:%02hhu"), RTC_DS3231.hour(), RTC_DS3231.minute(), RTC_DS3231.second());
    } else {
        sprintf_P(tTimeString, PSTR("%02hhu:%02hhu"), RTC_DS3231.hour(), RTC_DS3231.minute());
    }
#else
    if (aPrintLongFormat) {
        sprintf(tTimeString, "%02hhu:%02hhu:%02hhu", RTC_DS3231.hour(), RTC_DS3231.minute(), RTC_DS3231.second());
    } else {
        sprintf(tTimeString, "%02hhu:%02hhu", RTC_DS3231.hour(), RTC_DS3231.minute());
    }
#endif
    Serial.print(tTimeString);
}

/*
 * Only URTCLIB_SQWG_OFF_1 and URTCLIB_SQWG_1H work on my china module
 */
void testRTC() {
    Serial.println(F("Testing SQWG/INT output:"));

    Serial.println(F("fixed 1:"));
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_OFF_1);
    delay(2000);

    Serial.println(F("1 hertz:"));
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_1H);
    delay(3000);

    Serial.println(F("1024 hertz:"));
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_1024H);
    delay(2000);

    Serial.println(F("4096 hertz:"));
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_4096H);
    delay(2000);

    Serial.println(F("8192 hertz:"));
    RTC_DS3231.sqwgSetMode(URTCLIB_SQWG_8192H);
    delay(2000);

}

#endif // _ROBOT_ARM_RTC_CONTROL_HPP
