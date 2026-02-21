/**
 * \class uRTCLib
 * \brief DS1307, DS3231 and DS3232 RTCs basic library
 *
 * Really tiny library to basic RTC functionality on Arduino.
 *
 * Supported features:
 *     * SQuare Wave Generator
 *     * Fixed output pin for DS1307
 *     * RAM for DS1307 and DS3232
 *     * temperature sensor for DS3231 and DS3232
 *     * Alarms (1 and 2) for DS3231 and DS3232
 *     * Power failure check and clear
 *
 * See uEEPROMLib for EEPROM support, https://github.com/Naguissa/uEEPROMLib
 *
 * Note: For AtTiny you need TinyWireM library from Adafruit installed (available on library manager).
 *
 *
 * I2C locked in unknown state
 *
 * If uC crashes and I2C communication is locked in a unknown state you have a procedure to unlock it.
 * It's not implemented in this library but you can find an explanation and a PIC implementation thanks
 * to @rtek1000 in #42 : https://github.com/Naguissa/uRTCLib/issues/42
 *
 * @file uRTCLib.cpp
 * @copyright Naguissa
 * @author Naguissa
 * @see <a href="https://github.com/Naguissa/uRTCLib">https://github.com/Naguissa/uRTCLib</a>
 * @see <a href="https://www.foroelectro.net/librerias-arduino-ide-f29/rtclib-arduino-libreria-simple-y-eficaz-para-rtc-y-t95.html">https://www.foroelectro.net/librerias-arduino-ide-f29/rtclib-arduino-libreria-simple-y-eficaz-para-rtc-y-t95.html</a>
 * @see <a href="mailto:naguissa@foroelectro.net">naguissa@foroelectro.net</a>
 * @see <a href="https://github.com/Naguissa/uEEPROMLib">See uEEPROMLib for EEPROM support.</a>
 * @version 6.9.9
 */

#include <Arduino.h>
#ifndef URTCLIB_WIRE
	#if defined(ARDUINO_attiny) || defined(ARDUINO_AVR_ATTINYX4) || defined(ARDUINO_AVR_ATTINYX5) || defined(ARDUINO_AVR_ATTINYX7) || defined(ARDUINO_AVR_ATTINYX8) || defined(ARDUINO_AVR_ATTINYX61) || defined(ARDUINO_AVR_ATTINY43) || defined(ARDUINO_AVR_ATTINY828) || defined(ARDUINO_AVR_ATTINY1634) || defined(ARDUINO_AVR_ATTINYX313)
		#include <TinyWireM.h>                  // I2C Master lib for ATTinys which use USI
		#define URTCLIB_WIRE TinyWireM
	#else
		#include <Wire.h>
		#define URTCLIB_WIRE Wire
	#endif
#endif
#include "uRTCLib.h"

/**
 * \brief Constructor
 */
uRTCLib::uRTCLib() { }

/**
 * \brief Constructor
 *
 * @param rtc_address I2C address of RTC
 */
uRTCLib::uRTCLib(const int rtc_address) {
	_rtc_address = rtc_address;
}

/**
 * \brief Constructor
 *
 * @param rtc_address I2C address of RTC
 * @param model RTC model:
 *	 - #URTCLIB_MODEL_DS1307
 *	 - #URTCLIB_MODEL_DS3231
 *	 - #URTCLIB_MODEL_DS3232
 */
uRTCLib::uRTCLib(const int rtc_address, const uint8_t model) {
	_rtc_address = rtc_address;
	_model = model;
}

/**
 * \brief Refresh data from HW RTC
 *
 * @return False on error
 */
bool uRTCLib::refresh() {
	uRTCLIB_YIELD
	URTCLIB_WIRE.beginTransmission(_rtc_address);
	URTCLIB_WIRE.write(0); // set DS3231 register pointer to 00h
	URTCLIB_WIRE.endTransmission();
	uRTCLIB_YIELD

	// Adjust requested bytes to selected model:
	uRTCLIB_SIZE_T bytesRequested = 0;
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			bytesRequested = 8;
			_controlStatus = 0;
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
	        #if defined(ARDUINO_attiny) || defined(ARDUINO_AVR_ATTINYX4) || defined(ARDUINO_AVR_ATTINYX5) || defined(ARDUINO_AVR_ATTINYX7) || defined(ARDUINO_AVR_ATTINYX8) || defined(ARDUINO_AVR_ATTINYX61) || defined(ARDUINO_AVR_ATTINY43) || defined(ARDUINO_AVR_ATTINY828) || defined(ARDUINO_AVR_ATTINY1634) || defined(ARDUINO_AVR_ATTINYX313)
		        bytesRequested = 8;
	        #else
		        bytesRequested = 19;
	        #endif
	        break;
    }

	uint8_t bytesReceived = URTCLIB_WIRE.requestFrom(_rtc_address, bytesRequested);
	#if defined(ARDUINO_attiny) || defined(ARDUINO_AVR_ATTINYX4) || defined(ARDUINO_AVR_ATTINYX5) || defined(ARDUINO_AVR_ATTINYX7) || defined(ARDUINO_AVR_ATTINYX8) || defined(ARDUINO_AVR_ATTINYX61) || defined(ARDUINO_AVR_ATTINY43) || defined(ARDUINO_AVR_ATTINY828) || defined(ARDUINO_AVR_ATTINY1634) || defined(ARDUINO_AVR_ATTINYX313)
	    if (bytesReceived != 0) {
		    // Error
		    return false;
	    }
	#else
	    if (bytesReceived < bytesRequested) {
		    // Error
		    return false;
	    }
	#endif
	// 0x00h
	uint8_t tempByte = URTCLIB_WIRE.read();
	// On DS1307 EOSC and lost_power functions are combined in CH (Clock Halt).
	// It is placed on 1st bit of 1st byte.
	// So use that flag to mark both
	if (_model == URTCLIB_MODEL_DS1307) {
    	_controlStatus |= (((tempByte >> 1) & 0b01000000) | (tempByte & 0b10000000));
	}
	uRTCLIB_YIELD
	// Parentheses reqired on bitwise operation for correct uRTCLIB_bcdToDec operation
	// Serial.print("byte_00h "); Serial.println(tempByte, BIN);
	// Serial.print("byte_00h_bcdToDec_with_parentheses "); Serial.println(uRTCLIB_bcdToDec((tempByte & 0b01111111)));
	// Serial.print("byte_00h_bcdToDec_without_parentheses "); Serial.println(uRTCLIB_bcdToDec(tempByte & 0b01111111));
	_second = uRTCLIB_bcdToDec((tempByte & 0b01111111));
	// Serial.print("_second "); Serial.println(_second);

	// 0x01h
	_minute = URTCLIB_WIRE.read() & 0b01111111;
	uRTCLIB_YIELD
	_minute = uRTCLIB_bcdToDec(_minute);

	// 0x02h
	_hour = URTCLIB_WIRE.read() & 0b01111111;
	uRTCLIB_YIELD
	// Serial.print("0x02h "); Serial.println(_hour, BIN);
	bool _12hrMode = (bool) (_hour & 0b01000000);
	bool _pmNotAm = (bool) (_hour & 0b00100000);
	if(_12hrMode)
		_hour = _hour & 0b00011111;
	else
		_hour = _hour & 0b00111111;
	_hour = uRTCLIB_bcdToDec(_hour);

	// 0x03h
	_dayOfWeek = URTCLIB_WIRE.read();
	uRTCLIB_YIELD
	_dayOfWeek = uRTCLIB_bcdToDec(_dayOfWeek);

	// 0x04h
	_day = URTCLIB_WIRE.read();
	uRTCLIB_YIELD
	_day = uRTCLIB_bcdToDec(_day);

	// 0x05h
	_month = URTCLIB_WIRE.read() & 0b00011111;
	uRTCLIB_YIELD
	_month = uRTCLIB_bcdToDec(_month);

	// 0x06h
	_year = URTCLIB_WIRE.read();
	uRTCLIB_YIELD
	_year = uRTCLIB_bcdToDec(_year);

	_temp = URTCLIB_TEMP_ERROR; // Some obvious error value

	// Now we need to read extra requested bytes depending on the RTC model again:
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			uint8_t status;
			// 0x07h
			status = URTCLIB_WIRE.read();
			if (status & 0b00010000) {
				_sqwg_mode = status & 0b10000000 ? URTCLIB_SQWG_OFF_1 : URTCLIB_SQWG_OFF_0;
			} else {
				switch (status & 0b00000011) {
					case 0x00000011:
						_sqwg_mode = URTCLIB_SQWG_32768H;
						// Emulate 32K switch with 32K SQWG option on DS1307
	                    _controlStatus |= 0b00001000;
						break;

					case 0x00000010:
						_sqwg_mode = URTCLIB_SQWG_8192H;
						break;

					case 0x00000001:
						_sqwg_mode = URTCLIB_SQWG_4096H;
						break;

					// case 0x00000000:
					default:
						_sqwg_mode = URTCLIB_SQWG_1H;
						break;
				}
			}
			if(_12hrMode) _controlStatus |= 0b00100000;
			if(_pmNotAm) _controlStatus |= 0b00010000;
			// Serial.print("_controlStatus "); Serial.println(_controlStatus, BIN);
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
            #if defined(ARDUINO_attiny) || defined(ARDUINO_AVR_ATTINYX4) || defined(ARDUINO_AVR_ATTINYX5) || defined(ARDUINO_AVR_ATTINYX7) || defined(ARDUINO_AVR_ATTINYX8) || defined(ARDUINO_AVR_ATTINYX61) || defined(ARDUINO_AVR_ATTINY43) || defined(ARDUINO_AVR_ATTINY828) || defined(ARDUINO_AVR_ATTINY1634) || defined(ARDUINO_AVR_ATTINYX313)
            	bytesRequested = 11;
                bytesReceived = URTCLIB_WIRE.requestFrom(_rtc_address, bytesRequested);
                if (bytesReceived != 0) {
                    // Error
                    return false;
                }
            #endif
			uint8_t MSB, LSB; // LSB is also used as tmp  variable

			_a1_mode = URTCLIB_ALARM_TYPE_1_NONE;
			_a2_mode = URTCLIB_ALARM_TYPE_2_NONE;

			// 0x07h
			_a1_second = URTCLIB_WIRE.read();
			uRTCLIB_YIELD
			_a1_mode = _a1_mode | ((_a1_second & 0b10000000) >> 7);
			_a1_second = uRTCLIB_bcdToDec((_a1_second & 0b01111111));   //parentheses for bitwise operation as argument for uRTCLIB_bcdToDec is required
																		//otherwise wrong result will be returned by function

			// 0x08h
			_a1_minute = URTCLIB_WIRE.read();
			uRTCLIB_YIELD
			_a1_mode = _a1_mode | ((_a1_minute & 0b10000000) >> 6);
			_a1_minute = uRTCLIB_bcdToDec((_a1_minute & 0b01111111));

			// 0x09h
			_a1_hour = URTCLIB_WIRE.read();
			uRTCLIB_YIELD
			_a1_mode = _a1_mode | ((_a1_hour & 0b10000000) >> 5);
			_a1_hour = uRTCLIB_bcdToDec((_a1_hour & 0b00111111));

			// 0x0Ah
			_a1_day_dow = URTCLIB_WIRE.read();
			uRTCLIB_YIELD
			_a1_mode = _a1_mode | ((_a1_day_dow & 0b10000000) >> 4);
			if (!(_a1_mode & 0b00001111)) {
				_a1_mode = _a1_mode | ((_a1_day_dow & 0b01000000) >> 2);
			}
			_a1_day_dow = _a1_day_dow & 0b00111111;
			_a1_day_dow = uRTCLIB_bcdToDec(_a1_day_dow);

			// 0x0Bh
			_a2_minute = URTCLIB_WIRE.read();
			uRTCLIB_YIELD
			_a2_mode = _a2_mode | ((_a2_minute & 0b10000000) >> 6);
			_a2_minute = _a2_minute & 0b01111111;
			_a2_minute = uRTCLIB_bcdToDec(_a2_minute);

			// 0x0Ch
			_a2_hour = URTCLIB_WIRE.read();
			uRTCLIB_YIELD
			_a2_mode = _a2_mode | ((_a2_hour & 0b10000000) >> 5);
			_a2_hour = _a2_hour & 0b00111111;
			_a2_hour = uRTCLIB_bcdToDec(_a2_hour);

			// 0x0Dh
			_a2_day_dow = URTCLIB_WIRE.read();
			uRTCLIB_YIELD
			_a2_mode = _a2_mode | ((_a2_day_dow & 0b10000000) >> 4);
			if (!(_a2_mode & 0b00001110)) { // M4-M2 is 0, check DT/DY
				_a2_mode = _a2_mode | ((_a2_day_dow & 0b01000000) >> 2);
			}
			_a2_day_dow = uRTCLIB_bcdToDec(_a2_day_dow & 0b00111111);


			// Control registers
			// 0x0Eh
			LSB = URTCLIB_WIRE.read();
			uRTCLIB_YIELD
			// Serial.print("0x0Eh "); Serial.println(LSB, BIN);

			bool _eosc = (bool) (LSB & 0b10000000);
			// Serial.print("_eosc "); Serial.println(_eosc);
			if (LSB & 0b00000100) {
				_sqwg_mode = URTCLIB_SQWG_OFF_1;
				// Alarms disabled?
				if (LSB & 0b00000001) {
					_a1_mode |= 0b00100000;
				} else {
					_a1_mode = URTCLIB_ALARM_TYPE_1_NONE;
				}
				if (LSB & 0b00000010) {
					_a2_mode |= 0b00100000;
				} else {
					_a2_mode = URTCLIB_ALARM_TYPE_2_NONE;
				}
			} else {
				_sqwg_mode = LSB & 0b00011000;
				// Mark alarms as disabled because the SQWG:
				_a1_mode = URTCLIB_ALARM_TYPE_1_NONE;
				_a2_mode = URTCLIB_ALARM_TYPE_2_NONE;
			}


			// 0x0Fh
			LSB = URTCLIB_WIRE.read(); //Control
			uRTCLIB_YIELD
			// Serial.print("0x0Fh "); Serial.println(LSB, BIN);
			_controlStatus = LSB;
			if(_eosc) _controlStatus |= 0b01000000;
			if(_12hrMode) _controlStatus |= 0b00100000;
			if(_pmNotAm) _controlStatus |= 0b00010000;
			// Serial.print("_controlStatus "); Serial.println(_controlStatus, BIN);
			// _lost_power = (bool) (_controlStatus & 0b10000000);
			// _eosc = (bool) (_controlStatus & 0b01000000);
			// _12hrMode = (bool) (_controlStatus & 0b00100000);
			// _pmNotAm = (bool) (_controlStatus & 0b00010000);
			// _32k = (bool) (_controlStatus & 0b00001000);
			// _a2_triggered_flag = (bool) (_controlStatus & 0b00000010);
			// _a1_triggered_flag = (bool) (_controlStatus & 0b00000001);


			// 0x10h
			_aging = URTCLIB_WIRE.read(); //Aging
			uRTCLIB_YIELD
			if (_aging & 0b10000000) {
				_aging--;
			}


			// Temperature registers (11h-12h) get updated automatically every 64s

			// 0x11h
			MSB = URTCLIB_WIRE.read(); //2's complement int portion
			uRTCLIB_YIELD
			// 0x12h
			LSB = URTCLIB_WIRE.read(); //fraction portion
			uRTCLIB_YIELD
			_temp = 0b0000000000000000 | (MSB  << 2) | (LSB >> 6); // 8+2 bits, *25 is the same as number + 2bitdecimals * 100 in base 10
			if (MSB & 0b10000000) {
				_temp = (_temp | 0b1111110000000000);
				_temp--;
			}
			_temp = _temp * 25; // *25 is the same as number + 2bit (decimals) * 100 in base 10
			break;
	}
	return true;
}

/**
 * \brief Returns Enable Oscillator Flah
 *
 * DS3231 Control Register (0Eh) Bit 7: Enable Oscillator (EOSC)
 * When set to logic 0, the oscillator is started. When set to logic 1, the oscillator
 * is stopped when the DS3231 switches to VBAT. This bit is clear (logic 0) when power 
 * is first applied. When the DS3231 is powered by VCC, the oscillator is always on
 * regardless of the status of the EOSC bit. When EOSC is disabled, all register data 
 * is static.
 *
 * @return _eosc flag - 0 if set to enable OSC with VBAT if VCC is stopped
 */
bool uRTCLib::getEOSCFlag() {
	return (bool) (_controlStatus & 0b01000000);
}

/**
 * \brief Returns lost power VBAT staus
 *
 * DS1307 has a 'CH' Clock Halt Bit in Register 00h.
 *
 * On first application of power to the device the time and date registers are typically reset to 01/01/00  01  00:00:00  (MM/DD/YY  DOW  HH:MM:SS).
 *
 * The CH bit in the seconds register will be set to a 1.
 *
 * Others have a 'OSF' Oscillator Stop Flag in Register 0Fh
 *
 * @return True if power was lost (both power sources, VCC and VBAT)
 */
bool uRTCLib::lostPower() {
	return (bool) (_controlStatus & 0b10000000);
}

/**
 * \brief Clears lost power VBAT staus
 *
 * DS1307 has a 'CH' Clock Halt Bit in Register 00h ->  When cleared to 0, the oscillator is enabled and the time starts to increase
 *
 * Others have a 'OSF' Oscillator Stop Flag in Register 0Fh
 */
void uRTCLib::lostPowerClear() {
	uint8_t status;
	// _lost_power = (bool) (_controlStatus & 0b10000000);
	_controlStatus &= 0b01111111;	// clear lost power status
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x00);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			URTCLIB_WIRE.requestFrom(_rtc_address, 1);
			status = URTCLIB_WIRE.read();
			status &= 0b01111111;
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x00);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(status);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x0F);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			URTCLIB_WIRE.requestFrom(_rtc_address, 1);
			status = URTCLIB_WIRE.read();
			status &= 0b01111111;
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x0F);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(status);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			break;
	}
}



/**
  *\brief Enable VBAT operation when VCC power is lost.
  *
  * DS3231/DS3232 should enable the battery by default on first power-up using VCC, however this sometimes
  * won't happen automatically, and therefore the Control Register needs to be forcefully overwritten
  * to set EOSC to 0. The devices are usually shipped from China with EOSC set to 1 to save battery
  * (even though they come with no battery included).
  *
  * Cause of frustration for a lot of first time users of the device.
  *   i.e. Time is lost even though battery present.
  *
  * Reference: https://forum.arduino.cc/index.php?topic=586520.msg3990086#msg3990086
  *
  * @return True on success
  */
bool uRTCLib::enableBattery() {
	switch (_model) {

		case URTCLIB_MODEL_DS1307: // Not available
			// No EOSC register here, always connected, so we return true
			return true;
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			uint8_t status;
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x0E);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			URTCLIB_WIRE.requestFrom(_rtc_address, 1);
			status = URTCLIB_WIRE.read();
			status &= 0b01111111;
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x0E);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(status);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD

			// Return the status bit as a bool, to check against values of Control Register (0Eh)
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x0E);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			URTCLIB_WIRE.requestFrom(_rtc_address, 1);
			uRTCLIB_YIELD
			status =  URTCLIB_WIRE.read();
			bool _eosc = (bool) (status & 0b10000000);
			if(_eosc) _controlStatus |= 0b01000000;
			else _controlStatus &= 0b10111111;
			return !_eosc;
			break;
	}

	// Never should get here, so we return false
	return false;
}

/**
  *\brief Disable VBAT operation when VCC power is lost.
  *
  *
  * @return True on success
  */
bool uRTCLib::disableBattery() {
	switch (_model) {

		case URTCLIB_MODEL_DS1307: // Not available
			// No EOSC register here, always connected, so we return false (at the end)
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			uint8_t status;
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x0E);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			URTCLIB_WIRE.requestFrom(_rtc_address, 1);
			status = URTCLIB_WIRE.read();
			status |= 0b10000000;	// set eosc bit high to disable battery
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x0E);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(status);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD

			// Return the status bit as a bool, to check against values of Control Register (0Eh)
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x0E);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			URTCLIB_WIRE.requestFrom(_rtc_address, 1);
			status =  URTCLIB_WIRE.read();
			bool _eosc = (bool) (status & 0b10000000);
			if(_eosc) _controlStatus |= 0b01000000;
			else _controlStatus &= 0b10111111;
			return _eosc;
			break;
	}

	return false;
}

/**
 * \brief Returns actual temperature
 *
 * Temperature is returned as degrees * 100; i.e.: 3050 is 30.50º
 *
 * WARNING: DS1307 has no temperature register, so it always returns #URTCLIB_TEMP_ERROR
 *
 * @return Current stored temperature
 */
int16_t uRTCLib::temp() {
	if (_model == URTCLIB_MODEL_DS1307) {
		return URTCLIB_TEMP_ERROR;
	}
	return _temp;
}

/**
 * \brief Returns actual second
 *
 * @return Current stored second
 */
uint8_t uRTCLib::second() {
	return _second;
}

/**
 * \brief Returns actual minute
 *
 * @return Current stored minute
 */
uint8_t uRTCLib::minute() {
	return _minute;
}


/**
 * \brief Returns actual hour
 *
 * @return Current stored hour
 */
uint8_t uRTCLib::hour() {
	return _hour;
}

/**
 * \brief Returns whether clock is in 12 or 24 hour mode
 * with AM or PM if in 12 hour mode
 * 0 = 24 hour mode (0-23 hours)
 * 1 = 12 hour mode AM hours (1-12 hours)
 * 2 = 12 hour mode PM hours (1-12 hours)
 *
 * @return byte with value 0, 1 or 2
 */
uint8_t uRTCLib::hourModeAndAmPm() {
	if((bool) (_controlStatus & 0b00100000)){		// _12hrMode = (bool) (_controlStatus & 0b00100000);
		if((bool) (_controlStatus & 0b00010000))	// _pmNotAm = (bool) (_controlStatus & 0b00010000);
			return 2;
		else
			return 1;
	}
	else
		return 0;
}

/**
 * \brief Returns actual day
 *
 * @return Current stored day
 */
uint8_t uRTCLib::day() {
	return _day;
}

/**
 * \brief Returns actual month
 *
 * @return Current stored month
 */
uint8_t uRTCLib::month() {
	return _month;
}

/**
 * \brief Returns actual year
 *
 * @return Current stored year
 */
uint8_t uRTCLib::year() {
	return _year;
}

/**
 * \brief Returns actual Day Of Week
 *
 * @return Current stored Day Of Week
 *   - #URTCLIB_WEEKDAY_SUNDAY
 *   - #URTCLIB_WEEKDAY_MONDAY
 *   - #URTCLIB_WEEKDAY_TUESDAY
 *   - #URTCLIB_WEEKDAY_WEDNESDAY
 *   - #URTCLIB_WEEKDAY_THURSDAY
 *   - #URTCLIB_WEEKDAY_FRIDAY
 *   - #URTCLIB_WEEKDAY_SATURDAY
 */
uint8_t uRTCLib::dayOfWeek() {
	return _dayOfWeek;
}


/**
 * \brief Sets RTC i2 addres
 *
 * @param addr RTC i2C address
 */
void uRTCLib::set_rtc_address(const int addr) {
	_rtc_address = addr;
}


/**
 * \brief Sets RTC Model
 *
 * @param model RTC Model
 *	 - #URTCLIB_MODEL_DS1307
 *	 - #URTCLIB_MODEL_DS3231
 *	 - #URTCLIB_MODEL_DS3232
 */
void uRTCLib::set_model(const uint8_t model) {
	_model = model;
}

/**
 * \brief Gets RTC Model
 *
 * @return RTC Model
 *	 - #URTCLIB_MODEL_DS1307
 *	 - #URTCLIB_MODEL_DS3231
 *	 - #URTCLIB_MODEL_DS3232
 */
uint8_t uRTCLib::model() {
	return _model;
}

/**
 * \brief Sets RTC datetime data
 *
 * @param second second to set to HW RTC
 * @param minute minute to set to HW RTC
 * @param hour hour to set to HW RTC
 * @param dayOfWeek day of week to set to HW RTC
 *   - #URTCLIB_WEEKDAY_SUNDAY
 *   - #URTCLIB_WEEKDAY_MONDAY
 *   - #URTCLIB_WEEKDAY_TUESDAY
 *   - #URTCLIB_WEEKDAY_WEDNESDAY
 *   - #URTCLIB_WEEKDAY_THURSDAY
 *   - #URTCLIB_WEEKDAY_FRIDAY
 *   - #URTCLIB_WEEKDAY_SATURDAY
 * @param dayOfMonth day of month to set to HW RTC
 * @param month month to set to HW RTC
 * @param year year to set to HW RTC in last 2 digits mode. As RTCs only support 19xx and 20xx years (see datasheets), it's harcoded to 20xx.
 */
void uRTCLib::set(const uint8_t second, const uint8_t minute, const uint8_t hour, const uint8_t dayOfWeek, const uint8_t dayOfMonth, const uint8_t month, const uint8_t year) {
	uRTCLIB_YIELD
	URTCLIB_WIRE.beginTransmission(_rtc_address);
	URTCLIB_WIRE.write(0); // set next input to start at the seconds register
	URTCLIB_WIRE.write(uRTCLIB_decToBcd(second)); // set seconds
	URTCLIB_WIRE.write(uRTCLIB_decToBcd(minute)); // set minutes
	URTCLIB_WIRE.write(uRTCLIB_decToBcd(hour)); // set hours
	URTCLIB_WIRE.write(uRTCLIB_decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
	URTCLIB_WIRE.write(uRTCLIB_decToBcd(dayOfMonth)); // set date (1 to 31)
	URTCLIB_WIRE.write(0B10000000 | uRTCLIB_decToBcd(month)); // set month
	URTCLIB_WIRE.write(uRTCLIB_decToBcd(year)); // set year (0 to 99)
	URTCLIB_WIRE.endTransmission();
	uRTCLIB_YIELD
	//
	URTCLIB_WIRE.beginTransmission(_rtc_address);
	URTCLIB_WIRE.write(0X0F);
	URTCLIB_WIRE.endTransmission();
	uRTCLIB_YIELD
	/* flip OSF bit --> Disabled, use lostPowerClear instead.
	URTCLIB_WIRE.requestFrom(_rtc_address, 1);
	uint8_t statreg = URTCLIB_WIRE.read();
	statreg &= ~0x80;
	uRTCLIB_YIELD
	URTCLIB_WIRE.beginTransmission(_rtc_address);
	URTCLIB_WIRE.write(0X0F);
	URTCLIB_WIRE.write((byte)statreg);
	URTCLIB_WIRE.endTransmission();
	*/
}

/**
 * \brief Set clock in 12 or 24 hour mode
 * 12 hour mode has 1-12 hours and AM or PM flag
 * 24 hour mode has 0-23 hours
 * get current clock mode and AM or PM flag using hourModeAndAmPm()
 *
 * @param twelveHrMode true or false
 */
void uRTCLib::set_12hour_mode(const bool twelveHrMode) {
	bool currentMode12Hr = (bool) (_controlStatus & 0b00100000);
	if((currentMode12Hr && twelveHrMode) || (!currentMode12Hr && !twelveHrMode))	// already in same mode, return
		return;
	bool _pmNotAm = (bool) (_controlStatus & 0b00010000);
	if(twelveHrMode && !currentMode12Hr) {
		// current Mode is 24 hour
		// requested Mode is 12 hour
		if(_hour == 0) {		// 0Hr = 12AM
			_hour = 12;
			_pmNotAm = false;
		}
		else if(_hour < 12) {	// 1Hr-11Hr = 1AM-11AM
			_pmNotAm = false;
		}
		else if(_hour == 12) {	// 12Hr = 12PM
			_pmNotAm = true;
		}
		else {					// 13Hr-23Hr = 1PM-11PM
			_hour -= 12;
			_pmNotAm = true;
		}
	}
	else {
		// current Mode is 12 hour
		// requested Mode is 24 hour
		if(!_pmNotAm) {	// AM time 1AM-11AM = 1-11Hr, 12AM = 0Hr
			if(_hour == 12)
				_hour = 0;
			// else _hour = 1-11, will remain same in 24 hour mode as well
		}
		else {	// PM time 12PM = 12Hr, 1PM-11PM = 13-23Hr
			if(_hour < 12)
				_hour += 12;
			// else _hour = 12 PM, will remain same in 24 hour mode as well
		}
	}
	// prepare hour register byte
	byte hour_bcd = uRTCLIB_decToBcd(_hour);
	if(twelveHrMode) {
		// _12hrMode = true;
		_controlStatus |= 0b00100000;               // _12hrMode = (bool) (_controlStatus & 0b00100000);
		hour_bcd |= 0B01000000;
		// set AM or PM
		if(_pmNotAm) {
			_controlStatus |= 0b00010000;           // _pmNotAm = (bool) (_controlStatus & 0b00010000);
			hour_bcd |= 0B00100000;
		}
	}
	else {
		// _12hrMode = false;
		_controlStatus &= 0b11011111;
	}
	// set hour register byte
	uRTCLIB_YIELD
	URTCLIB_WIRE.beginTransmission(_rtc_address);
	URTCLIB_WIRE.write(0x02); // set next input to start at the hours register
	URTCLIB_WIRE.write(hour_bcd); // set hours
	URTCLIB_WIRE.endTransmission();
	uRTCLIB_YIELD
	//
	URTCLIB_WIRE.beginTransmission(_rtc_address);
	URTCLIB_WIRE.write(0X0F);
	URTCLIB_WIRE.endTransmission();
	uRTCLIB_YIELD
}



/*************  Alarms: ****************/


/**
 * \brief Sets any alarm
 *
 * This method can also be used to disable an alarm, but it's better to use alarmDisable(const uint8_t alarm) to do so.
 *
 * @param type Alarm type:
 *	 - #URTCLIB_ALARM_TYPE_1_NONE
 *	 - #URTCLIB_ALARM_TYPE_1_ALL_S
 *	 - #URTCLIB_ALARM_TYPE_1_FIXED_S
 *	 - #URTCLIB_ALARM_TYPE_1_FIXED_MS
 *	 - #URTCLIB_ALARM_TYPE_1_FIXED_HMS
 *	 - #URTCLIB_ALARM_TYPE_1_FIXED_DHMS
 *	 - #URTCLIB_ALARM_TYPE_1_FIXED_DOWHMS
 *	 - #URTCLIB_ALARM_TYPE_2_NONE
 *	 - #URTCLIB_ALARM_TYPE_2_ALL_M
 *	 - #URTCLIB_ALARM_TYPE_2_FIXED_M
 *	 - #URTCLIB_ALARM_TYPE_2_FIXED_HM
 *	 - #URTCLIB_ALARM_TYPE_2_FIXED_DHM
 *	 - #URTCLIB_ALARM_TYPE_2_FIXED_DOWHM
 * @param second second to set Alarm (ignored in Alarm 2)
 * @param minute minute to set Alarm
 * @param hour hour to set Alarm
 * @param day_dow Day of the month or DOW to set Alarm, depending on alarm type
 *
 * @return false in case of not supported (DS1307) or wrong parameters
 */
bool uRTCLib::alarmSet(const uint8_t type, const uint8_t second, const uint8_t minute, const uint8_t hour, const uint8_t day_dow) {
	bool ret = false;
	uint8_t status;
	if (_model == URTCLIB_MODEL_DS1307) {
		return false;
	}
	uRTCLIB_YIELD

	if (type == URTCLIB_ALARM_TYPE_1_NONE) {
		ret = true;

		// Disable Alarm:
		URTCLIB_WIRE.beginTransmission(_rtc_address);
		uRTCLIB_YIELD
		URTCLIB_WIRE.write(0x0E);
		uRTCLIB_YIELD
		URTCLIB_WIRE.endTransmission();
		uRTCLIB_YIELD
		URTCLIB_WIRE.requestFrom(_rtc_address, 1);
		status = URTCLIB_WIRE.read();
		status &= 0b11111110;
		URTCLIB_WIRE.beginTransmission(_rtc_address);
		uRTCLIB_YIELD
		URTCLIB_WIRE.write(0x0E);
		uRTCLIB_YIELD
		URTCLIB_WIRE.write(status);
		uRTCLIB_YIELD
		URTCLIB_WIRE.endTransmission();
		uRTCLIB_YIELD
		_a1_mode = type;
	} else if (type == URTCLIB_ALARM_TYPE_2_NONE) {
		ret = true;

		// Disable Alarm:
		URTCLIB_WIRE.beginTransmission(_rtc_address);
		uRTCLIB_YIELD
		URTCLIB_WIRE.write(0x0E);
		uRTCLIB_YIELD
		URTCLIB_WIRE.endTransmission();
		uRTCLIB_YIELD
		URTCLIB_WIRE.requestFrom(_rtc_address, 1);
		status = URTCLIB_WIRE.read();
		status &= 0b11111101;
		URTCLIB_WIRE.beginTransmission(_rtc_address);
		uRTCLIB_YIELD
		URTCLIB_WIRE.write(0x0E);
		uRTCLIB_YIELD
		URTCLIB_WIRE.write(status);
		uRTCLIB_YIELD
		URTCLIB_WIRE.endTransmission();
		uRTCLIB_YIELD
		_a2_mode = type;
	} else {
		switch (type & 0b10000000) {
			case 0b00000000: // Alarm 1
				ret = true;
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x07); // set next input to start at the seconds register
				uRTCLIB_YIELD
				URTCLIB_WIRE.write((uRTCLIB_decToBcd(second) & 0b01111111) | ((type & 0b00000001) << 7)); // set seconds & mode/bit1
				uRTCLIB_YIELD
				URTCLIB_WIRE.write((uRTCLIB_decToBcd(minute) & 0b01111111) | ((type & 0b00000010) << 6)); // set minutes & mode/bit2
				uRTCLIB_YIELD
				URTCLIB_WIRE.write((uRTCLIB_decToBcd(hour) & 0b00111111) | ((type & 0b00000100) << 5)); // set hours & mode/bit3
				uRTCLIB_YIELD
				URTCLIB_WIRE.write((uRTCLIB_decToBcd(day_dow) & 0b00111111) | ((type & 0b00001000) << 4) | ((type & 0b00010000) << 2)); // set date / day of week (1=Sunday, 7=Saturday)  & mode/bit4 & mode/DY-DT
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD

				// Enable Alarm:
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x0E);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD
				URTCLIB_WIRE.requestFrom(_rtc_address, 1);
				uRTCLIB_YIELD
				status = URTCLIB_WIRE.read();
				status = status | 0b00000101;  // INTCN and A1IE bits
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x0E);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(status);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD

				_a1_mode = type;
				_a1_second = second;
				_a1_minute = minute;
				_a1_hour = hour;
				_a1_day_dow = day_dow;
				_sqwg_mode = URTCLIB_SQWG_OFF_1;

				break;

			case 0b10000000: // Alarm 2
				ret = true;
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x0B); // set next input to start at the minutes register
				uRTCLIB_YIELD
				URTCLIB_WIRE.write((uRTCLIB_decToBcd(minute) & 0b01111111) | ((type & 0b00000010) << 6)); // set minutes & mode/bit2
				uRTCLIB_YIELD
				URTCLIB_WIRE.write((uRTCLIB_decToBcd(hour) & 0b00111111) | ((type & 0b00000100) << 5)); // set hours & mode/bit3
				uRTCLIB_YIELD
				URTCLIB_WIRE.write((uRTCLIB_decToBcd(day_dow) & 0b00111111) | ((type & 0b00001000) << 4) | ((type & 0b00010000) << 2)); // set date / day of week (1=Sunday, 7=Saturday)  & mode/bit4 & mode/DY-DT (bit3)
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD

				// Enable Alarm:
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x0E);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD
				URTCLIB_WIRE.requestFrom(_rtc_address, 1);
				uRTCLIB_YIELD
				status = URTCLIB_WIRE.read();
				status = status | 0b00000110;  // INTCN and A2IE bits
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x0E);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(status);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD

				_a2_mode = type;
				_a2_minute = minute;
				_a2_hour = hour;
				_a2_day_dow = day_dow;
				_sqwg_mode = URTCLIB_SQWG_OFF_1;

				break;
		} // Alarm type switch
		uRTCLIB_YIELD
	} // if..else
	return ret;
}



/**
 * \brief Disables an alarm
 *
 * @param alarm Alarm number:
 *	 - #URTCLIB_ALARM_1
 *	 - #URTCLIB_ALARM_2
 *
 * @return false in case of not supported (DS1307) or wrong parameters
 */
bool uRTCLib::alarmDisable(const uint8_t alarm) {
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			return false;
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			uint8_t status, mask = 0;
			switch (alarm) {
				case URTCLIB_ALARM_1: // Alarm 1
					mask = 0b11111110;  // A1IE bit
					_a1_mode = URTCLIB_ALARM_TYPE_1_NONE;
					break;

				case URTCLIB_ALARM_2: // Alarm 2
					mask = 0b11111101;  // A2IE bit
					_a2_mode = URTCLIB_ALARM_TYPE_2_NONE;
					break;
			} // Alarm type switch
			if (mask) {
				// Disable Alarm:
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x0E);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD
				URTCLIB_WIRE.requestFrom(_rtc_address, 1);
				status = URTCLIB_WIRE.read();
				status &= mask;  // A1IE or A2IE bit
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x0E);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(status);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD
				return true;
			}
			break;
	} // model switch
	return false;
}

/**
 * \brief Clears an alarm flag
 *
 * @param alarm Alarm number:
 *	 - #URTCLIB_ALARM_1
 *	 - #URTCLIB_ALARM_2
 *
 * @return false in case of not supported (DS1307) or wrong parameters
 */
bool uRTCLib::alarmClearFlag(const uint8_t alarm) {
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			return false;
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			uint8_t status, mask = 0;
			switch (alarm) {
				case URTCLIB_ALARM_1: // Alarm 1
					mask = 0b11111110;
					break;

				case URTCLIB_ALARM_2: // Alarm 2
					mask = 0b11111101;
					break;

			} // Alarm type switch
			if (mask) {
				// Clear Alarm Flag:
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x0F);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD
				URTCLIB_WIRE.requestFrom(_rtc_address, 1);
				status = URTCLIB_WIRE.read();
				status &= mask;  // A?F bit
				_controlStatus &= mask;	// clear alarm triggered flags on _controlStatus as well
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x0F);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(status);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD
				return true;
			}
			break;
	} // model switch
	return false;
}



/**
 * \brief Returns actual alarm mode.
 *
 * See URTCLIB_ALARM_TYPE_X_YYYYY defines to see modes
 *
 * @param alarm Alarm number:
 *	 - #URTCLIB_ALARM_1
 *	 - #URTCLIB_ALARM_2
 *
 * @return Current stored mode. 0b11111111 means error.
 *	 - #URTCLIB_ALARM_TYPE_1_NONE
 *	 - #URTCLIB_ALARM_TYPE_1_ALL_S
 *	 - #URTCLIB_ALARM_TYPE_1_FIXED_S
 *	 - #URTCLIB_ALARM_TYPE_1_FIXED_MS
 *	 - #URTCLIB_ALARM_TYPE_1_FIXED_HMS
 *	 - #URTCLIB_ALARM_TYPE_1_FIXED_DHMS
 *	 - #URTCLIB_ALARM_TYPE_1_FIXED_DOWHMS
 *	 -	...or...
 *	 - #URTCLIB_ALARM_TYPE_2_NONE
 *	 - #URTCLIB_ALARM_TYPE_2_ALL_M
 *	 - #URTCLIB_ALARM_TYPE_2_FIXED_M
 *	 - #URTCLIB_ALARM_TYPE_2_FIXED_HM
 *	 - #URTCLIB_ALARM_TYPE_2_FIXED_DHM
 *	 - #URTCLIB_ALARM_TYPE_2_FIXED_DOWHM
 */
uint8_t uRTCLib::alarmMode(const uint8_t alarm) {
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			return 0b11111111;
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			switch (alarm) {
				case URTCLIB_ALARM_1: // Alarm 1
					return _a1_mode;
					break;

				case URTCLIB_ALARM_2: // Alarm 2
					return _a2_mode;
					break;
			} // Alarm type switch
			break;
	} // model switch
	return 0b11111111;
}

/**
 * \brief Returns actual alarm second
 *
 * @param alarm Alarm number:
 *	 - #URTCLIB_ALARM_1
 *	 - #URTCLIB_ALARM_2
 *
 * @return Current stored second. 0b11111111 means error.
 */
uint8_t uRTCLib::alarmSecond(const uint8_t alarm) {
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			return 0b11111111;
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			switch (alarm) {
				case URTCLIB_ALARM_1: // Alarm 1
					return _a1_second;
					break;

				case URTCLIB_ALARM_2: // Alarm 2
					return 0;
					break;
			} // Alarm type switch
			break;
	} // model switch
	return 0b11111111;
}

/**
 * \brief Returns actual alarm minute
 *
 * @param alarm Alarm number:
 *	 - #URTCLIB_ALARM_1
 *	 - #URTCLIB_ALARM_2
 *
 * @return Current stored minute. 0b11111111 means error.
 */
uint8_t uRTCLib::alarmMinute(const uint8_t alarm) {
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			return 0b11111111;
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			switch (alarm) {
				case URTCLIB_ALARM_1: // Alarm 1
					return _a1_minute;
					break;

				case URTCLIB_ALARM_2: // Alarm 2
					return _a2_minute;
					break;
			} // Alarm type switch
			break;
	} // model switch
	return 0b11111111;
}


/**
 * \brief Returns actual alarm hour
 *
 * @param alarm Alarm number:
 *	 - #URTCLIB_ALARM_1
 *	 - #URTCLIB_ALARM_2
 *
 * @return Current stored hour. 0b11111111 means error.
 */
uint8_t uRTCLib::alarmHour(const uint8_t alarm) {
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			return 0b11111111;
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			switch (alarm) {
				case URTCLIB_ALARM_1: // Alarm 1
					return _a1_hour;
					break;

				case URTCLIB_ALARM_2: // Alarm 2
					return _a2_hour;
					break;
			} // Alarm type switch
			break;
	} // model switch
	return 0b11111111;
}

/**
 * \brief Returns actual alarm day or DOW
 *
 * @param alarm Alarm number:
 *	 - #URTCLIB_ALARM_1
 *	 - #URTCLIB_ALARM_2
 *
 * @return Current stored day or dow. 0b11111111 means error.
 */
uint8_t uRTCLib::alarmDayDow(const uint8_t alarm) {
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			return 0b11111111;
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			switch (alarm) {
				case URTCLIB_ALARM_1: // Alarm 1
					return _a1_day_dow;
					break;

				case URTCLIB_ALARM_2: // Alarm 2
					return _a2_day_dow;
					break;
			} // Alarm type switch
			break;
	} // model switch
	return 0b11111111;
}


/**
* \brief Checks if any alarm has been triggered
*
* NOTE: Alarm Flags A1F and A2F will be triggered whether or not Alarm Interrupt is Enabled A1IE and A2IE
* When the RTC register values match alarm register settings, the corresponding Alarm Flag ‘A1F’ or ‘A2F’ bit is set to logic 1.
* If using alarmTriggered function to check for alarm trigger, be sure to alarmMode function to see if alarm is enabled or not.
*
* @param alarm Alarm number:
*	 - #URTCLIB_ALARM_1
*	 - #URTCLIB_ALARM_2
*	 - #URTCLIB_ALARM_ANY
*
* @return bool true or false
*/
bool uRTCLib::alarmTriggered(const uint8_t alarm) {
	if ((alarm == URTCLIB_ALARM_1 || alarm == URTCLIB_ALARM_ANY) && (bool) (_controlStatus & 0b00000001)) {
		// _a1_triggered_flag = (bool) (_controlStatus & 0b00000001)
		return true;
	}
	if ((alarm == URTCLIB_ALARM_2 || alarm == URTCLIB_ALARM_ANY) && (bool) (_controlStatus & 0b00000010)) {
		// _a2_triggered_flag = (bool) (_controlStatus & 0b00000010)
		return true;
	}
	return false;
}





/************** SQuare Wave Generator ****************/

/**
 * \brief Changes SQWG mode, including turning it off
 *
 * @param mode SQWG mode:
 *	 - #URTCLIB_SQWG_OFF_0
 *	 - #URTCLIB_SQWG_OFF_1
 *	 - #URTCLIB_SQWG_1H
 *	 - #URTCLIB_SQWG_1024H
 *	 - #URTCLIB_SQWG_4096H
 *	 - #URTCLIB_SQWG_8192H
 *	 - #URTCLIB_SQWG_32768H
 *
 * @return false in case of not supported (DS1307) or wrong parameters
 */
bool uRTCLib::sqwgSetMode(const uint8_t mode) {
	uint8_t status, processAnd = 0b00000000, processOr = 0b00000000;
	uRTCLIB_YIELD
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			switch (mode) {
				case URTCLIB_SQWG_OFF_0:
					processAnd = 0b01101111;  // OUT and SQWE bits
					break;

				case URTCLIB_SQWG_OFF_1:
					processAnd = 0b11101111; //  SQWE
					processOr = 0b10000000; // OUT
					break;

				case URTCLIB_SQWG_1H:
					processAnd = 0b11111100; //  RS1, RS0
					processOr = 0b0010000; // SQWE
					break;

				case URTCLIB_SQWG_4096H:
					processAnd = 0b11111101; //  RS1
					processOr = 0b0010001; // SQWE, RS0
					break;

				case URTCLIB_SQWG_8192H:
					processAnd = 0b11111110; //  RS0
					processOr = 0b0010010; // SQWE, RS1
					break;

				case URTCLIB_SQWG_32768H:
					processAnd = 0b11111111; //  nothing
					processOr = 0b0010011; // SQWE, RS1, RS0
					break;

			} // mode switch

			if (processAnd || processOr) { // Any bit change?
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x07);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD
				URTCLIB_WIRE.requestFrom(_rtc_address, 1);
				status = URTCLIB_WIRE.read();
				status = (status & processAnd) | processOr;
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x07);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(status);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD
				_sqwg_mode = mode;
				return true;
			}
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			switch (mode) {
				case URTCLIB_SQWG_OFF_1:
					processAnd = 0b11111111; //  nothing
					processOr = 0b00000100; // INTCN
					break;

				case URTCLIB_SQWG_1H:
					processAnd = 0b11100011; //  RS1, RS0, INTCN
					processOr = 0b00000000; // nothing
					break;

				case URTCLIB_SQWG_1024H:
					processAnd = 0b11101011; //  RS1, INTCN
					processOr = 0b00001000; // RS0
					break;

				case URTCLIB_SQWG_4096H:
					processAnd = 0b11110011; //  RS0, INTCN
					processOr = 0b00010000; // RS1
					break;

				case URTCLIB_SQWG_8192H:
					processAnd = 0b11111011; //  INTCN
					processOr = 0b00011000; //  RS1, RS0
					break;

			} // mode switch

			if (processAnd || processOr) { // Any bit change?
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x0E);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD
				URTCLIB_WIRE.requestFrom(_rtc_address, 1);
				status = URTCLIB_WIRE.read();
				status = (status & processAnd) | processOr;
				URTCLIB_WIRE.beginTransmission(_rtc_address);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(0x0E);
				uRTCLIB_YIELD
				URTCLIB_WIRE.write(status);
				uRTCLIB_YIELD
				URTCLIB_WIRE.endTransmission();
				uRTCLIB_YIELD
				_sqwg_mode = mode;
				if (mode == URTCLIB_SQWG_OFF_1 || mode == URTCLIB_SQWG_OFF_0) {
					_a1_mode = URTCLIB_ALARM_TYPE_1_NONE;
					_a2_mode = URTCLIB_ALARM_TYPE_2_NONE;
				}
				return true;
			}
			break;
	} // model switch
	uRTCLIB_YIELD
	return false;
}


/**
 * \brief Gets current SQWG mode
 *
 * @return SQWG mode:
 *	 - #URTCLIB_SQWG_OFF_0
 *	 - #URTCLIB_SQWG_OFF_1
 *	 - #URTCLIB_SQWG_1H
 *	 - #URTCLIB_SQWG_1024H
 *	 - #URTCLIB_SQWG_4096H
 *	 - #URTCLIB_SQWG_8192H
 *	 - #URTCLIB_SQWG_32768H
 */
uint8_t uRTCLib::sqwgMode() {
	return _sqwg_mode;
}


/*** RAM functionality (Only DS1307. Addresses 08h to 3Fh so we offset 08h positions and limit to 38h as maximum address ***/


/**
 * \brief Reads a byte from RTC RAM
 *
 * @param address RAM Address
 *
 * @return content of that position. If any error it will return always 0xFF;
 */
byte uRTCLib::ramRead(const uint8_t address) {
	uint8_t offset = 0xff;
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			if (address < 0x38) {
				offset = 0x08;
			}
			break;

		case URTCLIB_MODEL_DS3232:
			if (address < 0xec) {
				offset = 0x14;
			}
			break;
	}
	if (offset != 0xff) {
		URTCLIB_WIRE.beginTransmission(_rtc_address);
		uRTCLIB_YIELD
		URTCLIB_WIRE.write(address + offset);
		uRTCLIB_YIELD
		URTCLIB_WIRE.endTransmission();
		uRTCLIB_YIELD
		URTCLIB_WIRE.requestFrom(_rtc_address, 1);
		uRTCLIB_YIELD
		return URTCLIB_WIRE.read();
	}
	return 0xff;
}


/**
 * \brief Writes a byte to RTC RAM
 *
 * @param address RAM Address
 * @param data Content to write on that position
 *
 * @return true if correct
 */
bool uRTCLib::ramWrite(const uint8_t address, byte data) {
	uint8_t offset = 0xff;
	switch (_model) {
		case URTCLIB_MODEL_DS1307:
			if (address < 0x38) {
				offset = 0x08;
			}
			break;

		case URTCLIB_MODEL_DS3232:
			if (address < 0xec) {
				offset = 0x14;
			}
			break;
	}
	if (offset != 0xff) {
		URTCLIB_WIRE.beginTransmission(_rtc_address);
		uRTCLIB_YIELD
		URTCLIB_WIRE.write(address + offset);
		uRTCLIB_YIELD
		URTCLIB_WIRE.write(data);
		uRTCLIB_YIELD
		URTCLIB_WIRE.endTransmission();
		uRTCLIB_YIELD
		return true;
	}
	return false;
}



/**
 * \brief Reads actual aging value on the RTC
 *
 * @return Aging register value on RTC, 2-complement recalculated (use as regular int8_t)
 */
int8_t uRTCLib::agingGet() {
	return _aging;
}

/**
 * \brief Sets aging value on the RTC
 *
 * @param val new value (use as regular int8_t, 2-complement conversion is done internally)
 *
 * @return True when executed, false if RTC doesn't support it.
 */
bool uRTCLib::agingSet(int8_t val) {
	bool ret = false;
	switch (_model) {
		case URTCLIB_MODEL_DS3231:
		case URTCLIB_MODEL_DS3232:
			if (val < 0) {
				val++;
			}
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x10);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(val);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD

			ret = true;

			// Read status register 0x0E
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x0E);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			URTCLIB_WIRE.requestFrom(_rtc_address, 1);
			uRTCLIB_YIELD
			byte status = URTCLIB_WIRE.read();
			uRTCLIB_YIELD

			// Enable CONV bit on status register 0x0E to apply changes inmediately
			status |= 0b00100000;
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0x0E);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(status);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD

	}
	return ret;
}



/**
 * \brief Enables 32K pin output
 *
 * A Pull-Up resistor is required on the pin.
 * As DS1307 doen't have this functionality we map it to SqWG with 32K frequency
 */
bool uRTCLib::enable32KOut() {
	//_32k = (bool) (_controlStatus & 0b00001000);
	_controlStatus |= 0b00001000;
	switch (_model) {
		case URTCLIB_MODEL_DS1307: // As DS1307 doesn't have this pin, map it to SqWG at same frequency
			return sqwgSetMode(URTCLIB_SQWG_32768H);
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			uint8_t status;
			uRTCLIB_YIELD
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			URTCLIB_WIRE.write(0X0F);
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			URTCLIB_WIRE.requestFrom(_rtc_address, 1);
			status = URTCLIB_WIRE.read();
			status |= 0b00001000;
			uRTCLIB_YIELD
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0X0F);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(status);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			return true;
			break;
	}
}

/**
 * \brief Disables 32K pin output
 *
 * As DS1307 doen't have this functionality we map it to SqWG with 32K frequency
 */
bool uRTCLib::disable32KOut() {
	//_32k = (bool) (_controlStatus & 0b00001000);
	_controlStatus &= 0b11110111;
	switch (_model) {
		case URTCLIB_MODEL_DS1307: // As DS1307 doesn't have this pin, map it to SqWG at same frequency
			return sqwgSetMode(URTCLIB_SQWG_OFF_0);
			break;

		// case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
		// case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
		default:
			uint8_t status;
			uRTCLIB_YIELD
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			URTCLIB_WIRE.write(0X0F);
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			URTCLIB_WIRE.requestFrom(_rtc_address, 1);
			status = URTCLIB_WIRE.read();
			status &= 0b11110111;
			uRTCLIB_YIELD
			URTCLIB_WIRE.beginTransmission(_rtc_address);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(0X0F);
			uRTCLIB_YIELD
			URTCLIB_WIRE.write(status);
			uRTCLIB_YIELD
			URTCLIB_WIRE.endTransmission();
			uRTCLIB_YIELD
			return true;
			break;
	}
}


/**
 * \brief Checks 32K pin output status
 *
 * As DS1307 doen't have this functionality we map it to SqWG with 32K frequency
 */
bool uRTCLib::status32KOut() {
	return (bool) (_controlStatus & 0b00001000);
}


/*** EEPROM functionality has been moved to separate library: https://github.com/Naguissa/uEEPROMLib ***/
