/*
 * ClockMovements.h
 *
 * layout of the pen and the digits
 *
 *  Created on: 21.05.2019
 *      Author: Armin
 */

#include "RobotArmServoConfiguration.h"
#include "uRTCLib.h"

#include <stdint.h>

#ifndef SRC_CLOCK_MOVEMENTS_H_
#define SRC_CLOCK_MOVEMENTS_H_

/*
 * All values in millimeter
 * All Z values have ORIGIN_HEIGHT_OVER_GROUND_PLANE subtracted
 * The numbers have
 */

#define DRAW_SURFACE_Z           (-20)

#define LIFT_HEIGHT              (15)

#define CLOCK_DIGIT_WIDTH        (25)
#define CLOCK_DIGIT_MARGIN_X     (10)
#define CLOCK_DIGITS_Y          (120) // see FRONT_VALUE_FOR_GROUND
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

#define PEN_POSITION_Z          (DRAW_SURFACE_Z - 15)
#define PEN_LENGTH              (60) // To move above the pen
#define PEN_GRIP_LENGTH         (30) // the position from pen tip, where to grip the pen
#define PEN_GRIP_Z              (PEN_POSITION_Z + PEN_GRIP_LENGTH ) // the position from pen tip, where to grip the pen
#define PEN_HOLDER_DEPTH        (20) // the depth of the hole in the holder
#if (PEN_GRIP_LENGTH < PEN_HOLDER_DEPTH)
#error PEN_GRIP_LENGTH must be bigger than PEN_HOLDER_DEPTH, otherwise pen cannot be stored correctly
#endif

#define ERASER_POSITION_X       CLOCK_HOUR_TENS_X
#define ERASER_POSITION_Y       90
#define ERASER_POSITION_Z       DRAW_SURFACE_Z
#define ERASER_LENGTH           (50) // To move above the eraser
#define ERASER_GRIP_LENGTH      (35) // the position from eraser base, where to grip the eraser
#define ERASER_GRIP_Z           (ERASER_POSITION_Z + ERASER_GRIP_LENGTH) // the position from eraser base, where to grip the eraser

void doGetPen();
void doStorePen();
void doGetEraser();
void doStoreEraser();
void doSetModeForClockMovement();
void checkTimeAndDraw(uRTCLib * aRTC_DS3231);

void doErase(uint8_t aDigitPosition);
void drawNumber(uint8_t aNumber);
void drawNumber(uint8_t aDigitPosition, uint8_t aNumber);

#endif /* SRC_CLOCK_MOVEMENTS_H_ */

#pragma once
