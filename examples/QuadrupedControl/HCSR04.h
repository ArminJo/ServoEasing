/*
 * HCSR04.h
 *
 *  Created on: 08.11.2018
 *  Copyright (C) 2018  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 */

#include <stdint.h>

#ifndef HCSR04_H_
#define HCSR04_H_

#define US_DISTANCE_DEFAULT_TIMEOUT 20000
// Timeout of 20000L is 3.4 meter
void initUSDistancePins(uint8_t aTriggerOutPin, uint8_t aEchoInPin);
unsigned int getUSDistance(unsigned int aTimeoutMicros = US_DISTANCE_DEFAULT_TIMEOUT);
unsigned int getCentimeterFromUSMicroSeconds(unsigned int aDistanceMicros);
unsigned int getUSDistanceAsCentiMeter(unsigned int aTimeoutMicros = US_DISTANCE_DEFAULT_TIMEOUT);
unsigned int getUSDistanceAsCentiMeterWithCentimeterTimeout(unsigned int aTimeoutCentimeter);
#if (defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7) | defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13) | defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5))
/*
 * Non blocking version
 */
void startUSDistanceAsCentiMeterWithCentimeterTimeoutNonBlocking(unsigned int aTimeoutCentimeter);
bool isUSDistanceMeasureFinished();
extern unsigned int sUSDistanceCentimeter;
extern volatile unsigned long sUSPulseMicros;
#endif

#endif // HCSR04_H_

#pragma once
