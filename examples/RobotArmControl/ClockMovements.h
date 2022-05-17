/*
 * ClockMovements.h
 *
 * Contains layout of the pen and the digits.
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

#include "RobotArmServoConfiguration.h"

#if defined(ROBOT_ARM_HAS_RTC_CONTROL)
#include "uRTCLib.h"
#endif

#include <stdint.h>

#ifndef _CLOCK_MOVEMENTS_H
#define _CLOCK_MOVEMENTS_H

/*
 * All values in millimeter
 */

#define DRAW_SURFACE_Z           (-20) // 2 cm below zero plane / height of shoulder

#define LIFT_HEIGHT              (15)

#define CLOCK_DIGIT_WIDTH        (25)
#define CLOCK_DIGIT_MARGIN_X     (10)
#define CLOCK_DIGITS_Y          (120) // see MAX_X_VALUE_FOR_GROUND
#define CLOCK_DIGIT_HEIGHT       (50)
#define CLOCK_DIGITS_Z           (DRAW_SURFACE_Z - 5)

// Position left side of number
#define CLOCK_HOUR_ONES_X       (-(CLOCK_DIGIT_WIDTH + CLOCK_DIGIT_MARGIN_X))
#define CLOCK_HOUR_TENS_X       (CLOCK_HOUR_ONES_X -(CLOCK_DIGIT_WIDTH + CLOCK_DIGIT_MARGIN_X))
#define CLOCK_MINUTES_TENS_X    (CLOCK_DIGIT_MARGIN_X)
#define CLOCK_MINUTES_ONES_X    (CLOCK_MINUTES_TENS_X + (CLOCK_DIGIT_WIDTH + CLOCK_DIGIT_MARGIN_X))

#define DIGIT_MINUTES_ONES      0
#define DIGIT_MINUTES_TENS      1
#define DIGIT_HOUR_ONES         2
#define DIGIT_HOUR_TENS         3

#define CLAW_HEIGHT             (10) // the height of the claw used to be safe above the pen before grabbing

#define PEN_POSITION_X          (CLOCK_MINUTES_ONES_X  + CLOCK_DIGIT_WIDTH)
#define PEN_POSITION_Y          (100)

#define PEN_TIP_POSITION_Z      (DRAW_SURFACE_Z - 15) // the position of tip if stored
#define PEN_LENGTH              (60) // To move above the pen
#define PEN_GRIP_OFFSET         (30) // the position above pen tip, where to grip the pen
#define PEN_GRIP_Z              (PEN_TIP_POSITION_Z + PEN_GRIP_OFFSET )
#define PEN_HOLDER_DEPTH        (20) // the depth of the hole in the holder
#if (PEN_GRIP_OFFSET < PEN_HOLDER_DEPTH)
#error PEN_GRIP_OFFSET must be bigger than PEN_HOLDER_DEPTH, otherwise pen cannot be stored correctly
#endif

#define ERASER_POSITION_X       CLOCK_HOUR_TENS_X
#define ERASER_POSITION_Y       90
#define ERASER_POSITION_Z       DRAW_SURFACE_Z
#define ERASER_LENGTH           (50) // To move above the eraser
#define ERASER_GRIP_LENGTH      (35) // the position from eraser base, where to grip the eraser
#define ERASER_GRIP_Z           (ERASER_POSITION_Z + ERASER_GRIP_LENGTH) // the position from eraser base, where to grip the eraser

void doStartClock();

void doDrawTime();
void doGetPen();
void doGetPenSpecial();
void doStorePen();
void doGetEraser();
void doStoreEraser();
void doSetModeForClockMovement();
#if defined(ROBOT_ARM_HAS_RTC_CONTROL)
void checkTimeAndDraw(uRTCLib * aRTC_DS3231);
#endif

void doErase(uint8_t aDigitPosition);
void drawNumber(uint8_t aNumber);
void drawNumber(uint8_t aDigitPosition, uint8_t aNumber);
void drawTime(uint_fast8_t aHour, uint_fast8_t aMinute, bool aForceDrawing);

#endif // _CLOCK_MOVEMENTS_H
