/*
 * ADCUtils.h
 *
 *  Created on: 23.02.2018
 *  Copyright (C) 2018  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 */

#ifndef SRC_ADCUTILS_H_
#define SRC_ADCUTILS_H_

#include <Arduino.h>

// PRESCALE4 => 13 * 4 = 52 microseconds per ADC conversion at 1 Mhz Clock => 19,2 kHz
#define ADC_PRESCALE2    1 // 26 microseconds per ADC conversion at 1 MHz
#define ADC_PRESCALE4    2 // 52 microseconds per ADC conversion at 1 MHz
// PRESCALE8 => 13 * 8 = 104 microseconds per ADC sample at 1 Mhz Clock => 9,6 kHz
#define ADC_PRESCALE8    3 // 104 microseconds per ADC conversion at 1 MHz
#define ADC_PRESCALE16   4 // 208 microseconds per ADC conversion at 1 MHz
#define ADC_PRESCALE32   5 // 416 microseconds per ADC conversion at 1 MHz
#define ADC_PRESCALE64   6 // 52 microseconds per ADC conversion at 16 MHz
#define ADC_PRESCALE128  7 // 104 microseconds per ADC conversion at 16 MHz

// definitions for 0.1 ms conversion time
#if (F_CPU == 1000000)
#define ADC_PRESCALE ADC_PRESCALE8
#elif (F_CPU == 8000000)
#define ADC_PRESCALE ADC_PRESCALE64
#elif (F_CPU == 16000000)
#define ADC_PRESCALE ADC_PRESCALE128
#endif

// Temperature channel definitions - 1 LSB / 1 degree Celsius
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define SHIFT_VALUE_FOR_REFERENCE REFS2
#define ADC_1_1_VOLT_CHANNEL_MUX 12
#else
#define SHIFT_VALUE_FOR_REFERENCE REFS0
#endif

// Temperature channel definitions - 1 LSB / 1 degree Celsius
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define ADC_TEMPERATURE_CHANNEL_MUX 15
#elif defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
#define ADC_ISCR_CHANNEL_MUX 3
#define ADC_TEMPERATURE_CHANNEL_MUX 11
#define ADC_1_1_VOLT_CHANNEL_MUX 12
#elif defined (__AVR_ATmega328P__)
#define ADC_TEMPERATURE_CHANNEL_MUX 8
#define ADC_1_1_VOLT_CHANNEL_MUX 14
#elif defined(__AVR_ATmega32U4__)
#define ADC_TEMPERATURE_CHANNEL_MUX 0x27
#define ADC_1_1_VOLT_CHANNEL_MUX 0x1E
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
#define ADC_1_1_VOLT_CHANNEL_MUX 0x1E
#define INTERNAL INTERNAL1V1
#endif

uint16_t readADCChannel(uint8_t aChannelNumber);
uint16_t readADCChannelWithReference(uint8_t aChannelNumber, uint8_t aReference);
uint16_t readADCChannelWithOversample(uint8_t aChannelNumber, uint8_t aOversampleExponent);
uint16_t readADCChannelWithReferenceOversample(uint8_t aChannelNumber, uint8_t aReference, uint8_t aOversampleExponent);
uint16_t readADCChannelWithReferenceMultiSamples(uint8_t aChannelNumber, uint8_t aReference, uint8_t aNumberOfSamples);
uint16_t readADCChannelWithReferenceMax(uint8_t aChannelNumber, uint8_t aReference, uint16_t aNumberOfSamples);
uint16_t readADCChannelWithReferenceMaxMicros(uint8_t aChannelNumber, uint8_t aReference, uint16_t aMicrosecondsToAquire);

float getVCCVoltageSimple(void);
uint16_t getVCCVoltageMillivoltSimple(void);
float getTemperatureSimple(void);
float getVCCVoltage(void);
uint16_t getVCCVoltageMillivolt(void);
float getTemperature(void);

#endif /* SRC_ADCUTILS_H_ */

#pragma once
