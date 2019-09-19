/*
 * QuadrupedNeoPixel.h
 *
 *  Created on: 18.09.2019
 *      Author: Armin
 */

#ifndef QUADRUPED_NEOPIXEL_H_
#define QUADRUPED_NEOPIXEL_H_

#include "NeoPatterns.h"

// How many NeoPixels are mounted?
#define NUM_PIXELS      24
#define PIXEL_OFFSET_RIGHT_BAR 0
#define PIXEL_OFFSET_FRONT_BAR 8
#define PIXEL_OFFSET_LEFT_BAR 16

void initNeoPatterns();
void triggerNeoPatterns();
void handleMovementPattern();
void handleServoTimerInterrupt();

extern NeoPatterns QuadrupedNeoPixelBar;

#endif /* QUADRUPED_NEOPIXEL_H_ */

#pragma once
