/*
 * ADCUtils.cpp
 *
 * ADC utility functions. Conversion time is defined as 0.104 milliseconds for 16 MHz Arduinos in ADCUtils.h.
 *
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Utils https://github.com/ArminJo/Arduino-Utils.
 *
 *  ArduinoUtils is free software: you can redistribute it and/or modify
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

#if defined(AVR)
#include "ADCUtils.h"

// Union to speed up the combination of low and high bytes to a word
// it is not optimal since the compiler still generates 2 unnecessary moves
// but using  -- value = (high << 8) | low -- gives 5 unnecessary instructions
union Myword {
    struct {
        uint8_t LowByte;
        uint8_t HighByte;
    } byte;
    uint16_t UWord;
    int16_t Word;
    uint8_t * BytePointer;
};

uint16_t readADCChannel(uint8_t aChannelNumber) {
    Myword tUValue;
    ADMUX = aChannelNumber | (DEFAULT << SHIFT_VALUE_FOR_REFERENCE);

//  ADCSRB = 0; // free running mode  - is default
// ADSC-StartConversion ADIF-Reset Interrupt Flag - NOT free running mode
    ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADIF) | ADC_PRESCALE);

// wait for single conversion to finish
    loop_until_bit_is_clear(ADCSRA, ADSC);

// Get value
    tUValue.byte.LowByte = ADCL;
    tUValue.byte.HighByte = ADCH;
    return tUValue.UWord;
//    return ADCL | (ADCH <<8); // needs 4 bytes more
}

uint16_t readADCChannelWithReference(uint8_t aChannelNumber, uint8_t aReference) {
    Myword tUValue;
    ADMUX = aChannelNumber | (aReference << SHIFT_VALUE_FOR_REFERENCE);

// ADCSRB = 0; // free running mode if ADATE is 1 - is default
    // ADSC-StartConversion ADIF-Reset Interrupt Flag - NOT free running mode
    ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADIF) | ADC_PRESCALE);

// wait for single conversion to finish
    loop_until_bit_is_clear(ADCSRA, ADSC);

// Get value
    tUValue.byte.LowByte = ADCL;
    tUValue.byte.HighByte = ADCH;
    return tUValue.UWord;
}

uint16_t readADCChannelWithOversample(uint8_t aChannelNumber, uint8_t aOversampleExponent) {
    return readADCChannelWithReferenceOversample(aChannelNumber, DEFAULT, aOversampleExponent);
}

uint16_t readADCChannelWithReferenceOversample(uint8_t aChannelNumber, uint8_t aReference, uint8_t aOversampleExponent) {
    uint16_t tSumValue = 0;
    ADMUX = aChannelNumber | (aReference << SHIFT_VALUE_FOR_REFERENCE);

// ADCSRB = 0; // free running mode if ADATE is 1 - is default
// ADSC-StartConversion ADATE-AutoTriggerEnable ADIF-Reset Interrupt Flag
    ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIF) | ADC_PRESCALE);

    for (uint8_t i = 0; i < _BV(aOversampleExponent); i++) {
        /*
         * wait for free running conversion to finish.
         * Do not wait for ADSC here, since ADSC is only low for 1 ADC Clock cycle on free running conversion.
         */
        loop_until_bit_is_set(ADCSRA, ADIF);

        ADCSRA |= _BV(ADIF); // clear bit to recognize next conversion has finished
        // Add value
        tSumValue += ADCL | (ADCH << 8); // using myWord does not save space here
        // tSumValue += (ADCH << 8) | ADCL; // this does NOT work!
    }
    ADCSRA &= ~_BV(ADATE); // Disable auto-triggering (free running mode)
    return (tSumValue >> aOversampleExponent);
}

/*
 * returns sum of all sample values
 */
uint16_t readADCChannelWithReferenceMultiSamples(uint8_t aChannelNumber, uint8_t aReference, uint8_t aNumberOfSamples) {
    uint16_t tSumValue = 0;
    ADMUX = aChannelNumber | (aReference << SHIFT_VALUE_FOR_REFERENCE);

// ADCSRB = 0; // free running mode if ADATE is 1 - is default
// ADSC-StartConversion ADATE-AutoTriggerEnable ADIF-Reset Interrupt Flag
    ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIF) | ADC_PRESCALE);

    for (uint8_t i = 0; i < aNumberOfSamples; i++) {
        /*
         * wait for free running conversion to finish.
         * Do not wait for ADSC here, since ADSC is only low for 1 ADC Clock cycle on free running conversion.
         */
        loop_until_bit_is_set(ADCSRA, ADIF);

        ADCSRA |= _BV(ADIF); // clear bit to recognize next conversion has finished
        // Add value
        tSumValue += ADCL | (ADCH << 8); // using myWord does not save space here
        // tSumValue += (ADCH << 8) | ADCL; // this does NOT work!
    }
    ADCSRA &= ~_BV(ADATE); // Disable auto-triggering (free running mode)
    return tSumValue;
}

/*
 * use ADC_PRESCALE16 which gives 13 us conversion and good linearity
 */
uint16_t readADCChannelWithReferenceMax(uint8_t aChannelNumber, uint8_t aReference, uint16_t aNumberOfSamples) {
    uint16_t tADCValue = 0;
    uint16_t tMaximum = 0;
    ADMUX = aChannelNumber | (aReference << SHIFT_VALUE_FOR_REFERENCE);

// ADCSRB = 0; // free running mode if ADATE is 1 - is default
// ADSC-StartConversion ADATE-AutoTriggerEnable ADIF-Reset Interrupt Flag
    ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIF) | ADC_PRESCALE16);

    for (uint16_t i = 0; i < aNumberOfSamples; i++) {
        /*
         * wait for free running conversion to finish.
         * Do not wait for ADSC here, since ADSC is only low for 1 ADC Clock cycle on free running conversion.
         */
        loop_until_bit_is_set(ADCSRA, ADIF);

        ADCSRA |= _BV(ADIF); // clear bit to recognize next conversion has finished
        // check value
        tADCValue = ADCL | (ADCH << 8);
        if (tADCValue > tMaximum) {
            tMaximum = tADCValue;
        }
    }
    ADCSRA &= ~_BV(ADATE); // Disable auto-triggering (free running mode)
    return tMaximum;
}

uint16_t readADCChannelWithReferenceMaxMicros(uint8_t aChannelNumber, uint8_t aReference, uint16_t aMicrosecondsToAquire) {
    uint16_t tNumberOfSamples = aMicrosecondsToAquire / 13;
    return readADCChannelWithReferenceMax(aChannelNumber, aReference, tNumberOfSamples);
}

/*
 * aMaxRetries = 255 -> try forever
 */
int readUntil4ConsecutiveValuesAreEqual(uint8_t aChannelNumber, uint8_t aDelay, uint8_t aAllowedDifference, uint8_t aMaxRetries) {
    int tValues[4];
    int tMin;
    int tMax;

    tValues[0] = readADCChannel(aChannelNumber);
    for (int i = 1; i < 4; ++i) {
        delay(aDelay); // only 3 delays!
        tValues[i] = readADCChannel(aChannelNumber);
    }

    do {
        // find min and max
        tMin = 1024;
        tMax = 0;
        for (int i = 0; i < 4; ++i) {
            if (tValues[i] < tMin) {
                tMin = tValues[i];
            }
            if (tValues[i] > tMax) {
                tMax = tValues[i];
            }
        }
        /*
         * check for terminating condition
         */
        if ((tMax - tMin) <= aAllowedDifference) {
            break;
        } else {
//            Serial.print("Difference=");
//            Serial.println(tMax - tMin);

            // move values
            for (int i = 0; i < 3; ++i) {
                tValues[i] = tValues[i + 1];
            }
            // and wait
            delay(aDelay);
            tValues[3] = readADCChannel(aChannelNumber);
        }
        if (aMaxRetries != 255) {
            aMaxRetries--;
        }
    } while (aMaxRetries > 0);

    return (tMax + tMin) / 2;
}

/*
 * Versions without handling of switched reference and channel.
 * Use only if reference (DEFAULT, INTERNAL) is known to be at the right value (DEFAULT for VCC and INTERNAL for temperature)
 * and register ADMUX may be overwritten.
 * Use it for example if you only call getVCCVoltageSimple() or getTemperatureSimple() in your program.
 * Calling both will lead to wrong values since of reference and channel switching.
 */
float getVCCVoltageSimple(void) {
// use AVCC with external capacitor at AREF pin as reference
    float tVCC = readADCChannelWithReferenceOversample(ADC_1_1_VOLT_CHANNEL_MUX, DEFAULT, 2);
    return ((1023 * 1.1) / tVCC);
}

uint16_t getVCCVoltageMillivoltSimple(void) {
// use AVCC with external capacitor at AREF pin as reference
    uint16_t tVCC = readADCChannelWithReferenceOversample(ADC_1_1_VOLT_CHANNEL_MUX, DEFAULT, 2);
    return ((1023L * 1100) / tVCC);
}

float getTemperatureSimple(void) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    return 0.0;
#else
// use internal 1.1 volt as reference
    float tTemp = (readADCChannelWithReferenceOversample(ADC_TEMPERATURE_CHANNEL_MUX, INTERNAL, 2) - 317);
    return (tTemp / 1.22);
#endif
}

float getVCCVoltage(void) {
    return (getVCCVoltageMillivolt() / 1000.0);
}

/*
 * Read value of 1.1 volt internal channel using VCC as reference.
 * Waits for reference and channel switching.
 */
uint16_t getVCCVoltageMillivolt(void) {
// use AVCC with external capacitor at AREF pin as reference
    uint8_t tOldADMUX = ADMUX;
    /*
     * Must wait >= 200 us if reference has to be switched to VSS
     * Must wait >= 400 us if channel has to be switched to 1.1 volt internal channel from channel with 5 volt input
     */
    if ((ADMUX & (INTERNAL << SHIFT_VALUE_FOR_REFERENCE)) || ((ADMUX & 0x0F) != ADC_1_1_VOLT_CHANNEL_MUX)) {
        // Switch to 1.1 volt channel and AREF to VCC
        ADMUX = ADC_1_1_VOLT_CHANNEL_MUX | (DEFAULT << SHIFT_VALUE_FOR_REFERENCE);
        // and wait for settling
        delayMicroseconds(400); // experimental value is >= 400 us
    }
    uint16_t tVCC = readADCChannelWithReferenceOversample(ADC_1_1_VOLT_CHANNEL_MUX, DEFAULT, 2);
    ADMUX = tOldADMUX;
    /*
     * Do not wait for reference to settle here, since it may not be necessary
     */
    return ((1023L * 1100) / tVCC);
}

void printVCCVoltageMillivolt(Print* aSerial) {
    aSerial->print(F("VCC="));
    aSerial->print(getVCCVoltageMillivolt());
    aSerial->println(" mV");
}

/*
 * Version which restore the ADC Channel and handle reference switching.
 */
float getTemperature(void) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    return 0.0;
#else
// use internal 1.1 volt as reference
    uint8_t tOldADMUX;

    bool tReferenceMustBeChanged = (ADMUX & (DEFAULT << SHIFT_VALUE_FOR_REFERENCE));
    if (tReferenceMustBeChanged) {
        tOldADMUX = ADMUX;
        // set AREF  to 1.1 volt and wait for settling
        ADMUX = ADC_TEMPERATURE_CHANNEL_MUX | (INTERNAL << SHIFT_VALUE_FOR_REFERENCE);
        delayMicroseconds(4000); // measured value is 3500 us
    }

    float tTemp = (readADCChannelWithReferenceOversample(ADC_TEMPERATURE_CHANNEL_MUX, INTERNAL, 2) - 317);

    if (tReferenceMustBeChanged) {
        ADMUX = tOldADMUX;
        // wait for settling back to VCC
        delayMicroseconds(400); // experimental value is > 200 us
    }
    return (tTemp / 1.22);
#endif
}
#endif // defined(AVR)
