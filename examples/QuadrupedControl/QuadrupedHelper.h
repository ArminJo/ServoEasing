/*
 * QuadrupedHelper.h
 *
 *   Contains miscellaneous function for the quadruped
 *
 *  Copyright (C) 2019-2026  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of QuadrupedControl https://github.com/ArminJo/QuadrupedControl.
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  QuadrupedControl and ServoEasing are free software: you can redistribute it and/or modify
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

#ifndef _QUADRUPED_HELPER_H
#define _QUADRUPED_HELPER_H

void checkForVCCUnderVoltageAndShutdown();
bool checkForVCCUnderVoltage();
void playShutdownMelody();
void doBeep();

#if defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
void doUSRight();
void doUSLeft();
void doUSScan();
#endif

#if defined(QUADRUPED_ENABLE_RTTTL)
void doRandomMelody();
#endif

void doCalibration();

extern bool isShutDownByUndervoltage;

#endif // _QUADRUPED_HELPER_H
