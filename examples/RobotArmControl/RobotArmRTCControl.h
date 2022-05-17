/*
 * RobotArmRTCControl.h
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef _ROBOT_ARM_RTC_CONTROL_H
#define _ROBOT_ARM_RTC_CONTROL_H

#include "uRTCLib.h"

extern uRTCLib RTC_DS3231;

void initRTC();
void testRTC();

#define DATE_FORMAT_LONG_MASK 0x01
#define DATE_FORMAT_AMERICAN 0x02
#define DATE_FORMAT_AMERICAN_LONG 0x03
#define DATE_FORMAT_EUROPEAN 0x04
#define DATE_FORMAT_EUROPEAN_LONG 0x05

void printRTC(uint8_t aDateFormatSpecifier = 0);
bool printRTCEveryPeriod(uint16_t aPeriodSeconds, uint8_t aDateFormatSpecifier = 0);

void printRTCDate(uint8_t aDateFormatSpecifier = 0);
void printRTCDateAmericanFormat(bool aPrintLongFormat);
void printRTCDateEuropeanFormat(bool aPrintLongFormat);
void printRTCDateISOFormat();

void printRTCTime(bool aPrintLongFormat = true, bool aDoRefresh = false);

void printRTCTemperature();
#endif // _ROBOT_ARM_RTC_CONTROL_H
