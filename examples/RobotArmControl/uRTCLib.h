/**
 * \mainpage
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
 *
 * @see <a href="https://github.com/Naguissa/uRTCLib">https://github.com/Naguissa/uRTCLib</a>
 * @see <a href="https://www.foroelectro.net/librerias-arduino-ide-f29/rtclib-arduino-libreria-simple-y-eficaz-para-rtc-y-t95.html">https://www.foroelectro.net/librerias-arduino-ide-f29/rtclib-arduino-libreria-simple-y-eficaz-para-rtc-y-t95.html</a>
 * @see <a href="mailto:naguissa@foroelectro.net">naguissa@foroelectro.net</a>
 * @see <a href="https://github.com/Naguissa/uEEPROMLib">See uEEPROMLib for EEPROM support.</a>
 * @version 6.9.9
 */
/** \file uRTCLib.h
 *   \brief uRTCLib header file
 */
#ifndef URTCLIB
	/**
	 * \brief Prevent multiple inclussion
	 */
	#define URTCLIB
	#include "Arduino.h"
	#ifndef URTCLIB_WIRE
		#if defined(ARDUINO_attiny) || defined(ARDUINO_AVR_ATTINYX4) || defined(ARDUINO_AVR_ATTINYX5) || defined(ARDUINO_AVR_ATTINYX7) || defined(ARDUINO_AVR_ATTINYX8) || defined(ARDUINO_AVR_ATTINYX61) || defined(ARDUINO_AVR_ATTINY43) || defined(ARDUINO_AVR_ATTINY828) || defined(ARDUINO_AVR_ATTINY1634) || defined(ARDUINO_AVR_ATTINYX313)
			#include <TinyWireM.h>                  // I2C Master lib for ATTinys which use USI
			#define URTCLIB_WIRE TinyWireM
		#else
			#include <Wire.h>
			#define URTCLIB_WIRE Wire
		#endif
	#endif

	/**
	 * \brief Default RTC I2C address
	 *
	 * Usual address is 0x68
	 */
	#define URTCLIB_ADDRESS 0x68

	/************	MODELS ***********/
	/**
	 * \brief Model definition, DS1307
	 */
	#define URTCLIB_MODEL_DS1307 1
	/**
	 * \brief Model definition, DS3231
	 */
	#define URTCLIB_MODEL_DS3231 2
	/**
	 * \brief Model definition, DS3232
	 */
	#define URTCLIB_MODEL_DS3232 3


	/************	WEEK DAYS ***********/
	/**
	 * \brief Week day definition, [Mon..Sun] as [1..7]. Sunday
	 */
	#define URTCLIB_WEEKDAY_SUNDAY 1
	/**
	 * \brief Week day definition, [Mon..Sun] as [1..7]. Monday
	 */
	#define URTCLIB_WEEKDAY_MONDAY 2
	/**
	 * \brief Week day definition, [Mon..Sun] as [1..7]. Tuesday
	 */
	#define URTCLIB_WEEKDAY_TUESDAY 3
	/**
	 * \brief Week day definition, [Mon..Sun] as [1..7]. Wednesday
	 */
	#define URTCLIB_WEEKDAY_WEDNESDAY 4
	/**
	 * \brief Week day definition, [Mon..Sun] as [1..7]. Thursday
	 */
	#define URTCLIB_WEEKDAY_THURSDAY 5
	/**
	 * \brief Week day definition, [Mon..Sun] as [1..7]. Friday
	 */
	#define URTCLIB_WEEKDAY_FRIDAY 6
	/**
	 * \brief Week day definition, [Mon..Sun] as [1..7]. Saturday
	 */
	#define URTCLIB_WEEKDAY_SATURDAY 7



	/************	ALARM SELECTION: ***********/
	//Note: Not valid for DS1307!

	/**
	 * \brief Alarm 1 - Disabled
	 *
	 * Alarm1 structure:
	 *
	 * bit 0 - A1M1
	 * bit 1 - A1M2
	 * bit 2 - A1M3
	 * bit 3 - A1M4
	 * bit 4 - A1 DT/DY
	 * bit 5 - A1 Enabled
	 * bit 6 - Unused, always 0
	 * bit 7 - Always 0
	 */
	#define URTCLIB_ALARM_TYPE_1_NONE 0b00000000

	/**
	 * \brief Alarm 1 - Trigger every second
	 */
	#define URTCLIB_ALARM_TYPE_1_ALL_S 0b00101111

	/**
	 * \brief Alarm 1 - Trigger every minute at a fixed second
	 */
	#define URTCLIB_ALARM_TYPE_1_FIXED_S 0b00101110

	/**
	 * \brief Alarm 1 - Trigger every hour at a fixed minute and second
	 */
	#define URTCLIB_ALARM_TYPE_1_FIXED_MS 0b00101100

	/**
	 * \brief Alarm 1 - Trigger every day at a fixed hour, minute and second
	 */
	#define URTCLIB_ALARM_TYPE_1_FIXED_HMS 0b00101000

	/**
	 * \brief Alarm 1 - Trigger every month at a fixed day, hour, minute and second
	 */
	#define URTCLIB_ALARM_TYPE_1_FIXED_DHMS 0b00100000

	/**
	 * \brief Alarm 1 - Trigger every week at a fixed day-of-week, hour, minute and second
	 */
	#define URTCLIB_ALARM_TYPE_1_FIXED_DOWHMS 0b00110000

	/**
	 * \brief Alarm 2 - Disabled
	 *
	 * Alarm1 structure:
	 *
	 * bit 0 - A2M1 - Unused, always 0
	 * bit 1 - A2M2
	 * bit 2 - A2M3
	 * bit 3 - A2M4
	 * bit 4 - A2 DT/DY
	 * bit 5 - A2 Enabled
	 * bit 6 - Unused, always 0
	 * bit 7 - Always 1
	 */
	#define URTCLIB_ALARM_TYPE_2_NONE 0b10000000

	/**
	 * \brief Alarm 2 - Trigger every minute at 00 seconds
	 */
	#define URTCLIB_ALARM_TYPE_2_ALL_M 0b10101110

	/**
	 * \brief Alarm 2 - Trigger every hour at minute and 00 seconds
	 */
	#define URTCLIB_ALARM_TYPE_2_FIXED_M 0b10101100

	/**
	 * \brief Alarm 2 - Trigger every day at hour, minute and 00 seconds
	 */
	#define URTCLIB_ALARM_TYPE_2_FIXED_HM 0b10101000

	/**
	 * \brief Alarm 2 - Trigger every month at day, hour, minute and 00 seconds
	 */
	#define URTCLIB_ALARM_TYPE_2_FIXED_DHM 0b10100000

	/**
	 * \brief Alarm 2 - Trigger every week at day-of-week, hour, minute and 00 seconds
	 */
	#define URTCLIB_ALARM_TYPE_2_FIXED_DOWHM 0b10110000


	/**
	 * \brief When requesting for Alarm 1
	 */
	#define URTCLIB_ALARM_1 URTCLIB_ALARM_TYPE_1_NONE

	/**
	 * \brief When requesting for Alarm 2
	 */
	#define URTCLIB_ALARM_2 URTCLIB_ALARM_TYPE_2_NONE

	/**
	 * \brief When requesting for any of both alarms, for triggered alarms
	 */
	#define URTCLIB_ALARM_ANY 0b01010101


	/************	SQWG SELECTION: ***********/

	/**
	 * \brief SQWG OFF, keeps output low
	 *
	 * Only valid for DS1307
	 */
	#define URTCLIB_SQWG_OFF_0 0b11111111

	/**
	 * \brief SQWG OFF, keeps output hight
	 */
	#define URTCLIB_SQWG_OFF_1 0b11111110

	/**
	 * \brief SQWG running at 1Hz
	 */
	#define URTCLIB_SQWG_1H 0b00000000

	/**
	 * \brief SQWG running at 1024Hz
	 *
	 * Not  valid for DS1307
	 */
	#define URTCLIB_SQWG_1024H 0b00001000

	/**
	 * \brief SQWG running at 4096Hz
	 */
	#define URTCLIB_SQWG_4096H 0b00010000

	/**
	 * \brief SQWG running at 8192Hz
	 */
	#define URTCLIB_SQWG_8192H 0b00011000

	/**
	 * \brief SQWG running at 32768Hz
	 *
	 * Only valid for DS1307
	 */
	#define URTCLIB_SQWG_32768H 0b00000011


	/************	TEMPERATURE ***********/
	/**
	 * \brief Temperarure read error indicator return value
	 *
	 * 327.67º, obviously erroneous
	 */
	#define URTCLIB_TEMP_ERROR 32767


	/************	MISC  ***********/


	/**
	 * \brief Convert normal decimal numbers to binary coded decimal
	 */
	#define uRTCLIB_decToBcd(val) ((uint8_t) ((val / 10 * 16) + (val % 10)))

	/**
	 * \brief Convert binary coded decimal to normal decimal numbers
	 */
	#define uRTCLIB_bcdToDec(val) ((uint8_t) ((val / 16 * 10) + (val % 16)))

	// ESP yield function (ESP32 has no need for that on dual core, but it has on single core version)
	#if ARDUINO_ARCH_ESP8266
		/**
		 * \brief ESP8266, yield to don't block ESP functionality.
		 *
		 * When this library is used in other MCUs this is simply removed by the preprocessor
		 */
		#define uRTCLIB_YIELD yield();
	#else
		#if ARDUINO_ARCH_ESP32
		/**
		 * \brief ESP32, yield to don't block ESP functionality.
		 *
		 * When this library is used in other MCUs this is simply removed by the preprocessor
		 */
			#define uRTCLIB_YIELD yield();
		#else
			#define uRTCLIB_YIELD
		#endif
	#endif
	
	#ifdef ARDUINO_ARCH_MEGAAVR
		/**
		 * \brief MEGAAVR core uses int instead size_t
		 */
		#define uRTCLIB_SIZE_T int
	#else
		#define uRTCLIB_SIZE_T size_t
	#endif


	class uRTCLib {
		public:
			/******* Constructors *******/
			/**
			 * \brief Constructor
			 */
			uRTCLib();
			/**
			 * \brief Constructor
			 *
			 * @param rtc_address I2C address of RTC
			 */
			uRTCLib(const int);
			/**
			 * \brief Constructor
			 *
			 * @param rtc_address I2C address of RTC
			 * @param model RTC model:
			 *	 - #URTCLIB_MODEL_DS1307
			 *	 - #URTCLIB_MODEL_DS3231
			 *	 - #URTCLIB_MODEL_DS3232
			 */
			uRTCLib(const int, const uint8_t);

			/******* RTC functions ********/
			/**
			 * \brief Refresh data from HW RTC
			 *
			 * @return False on error
			 */
			bool refresh();
			/**
			 * \brief Returns actual second
			 *
			 * @return Current stored second
			 */
			uint8_t second();
			/**
			 * \brief Returns actual minute
			 *
			 * @return Current stored minute
			 */
			uint8_t minute();
			/**
			 * \brief Returns actual hour
			 *
			 * @return Current stored hour
			 */
			uint8_t hour();
			/**
			 * \brief Returns whether clock is in 12 or 24 hour mode
			 * and AM or PM if in 12 hour mode
			 * 0 = 24 hour mode (0-23 hours)
			 * 1 = 12 hour mode AM hours (1-12 hours)
			 * 2 = 12 hour mode PM hours (1-12 hours)
			 *
			 * @return byte with value 0, 1 or 2
			 */
			uint8_t hourModeAndAmPm();
			/**
			 * \brief Returns actual day
			 *
			 * @return Current stored day
			 */
			uint8_t day();
			/**
			 * \brief Returns actual month
			 *
			 * @return Current stored month
			 */
			uint8_t month();
			/**
			 * \brief Returns actual year
			 *
			 * @return Current stored year
			 */
			uint8_t year();
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
			uint8_t dayOfWeek();
			/**
			 * \brief Returns actual temperature
			 *
			 * Temperature is returned as degrees * 100; i.e.: 3050 is 30.50º
			 *
			 * WARNING: DS1307 has no temperature register, so it always returns #URTCLIB_TEMP_ERROR
			 *
			 * @return Current stored temperature
			 */
			int16_t temp();
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
			void set(const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t);
			/**
			 * \brief Set clock in 12 or 24 hour mode
			 * 12 hour mode has 1-12 hours and AM or PM flag
			 * 24 hour mode has 0-23 hours
			 * get current clock mode and AM or PM flag using hourModeAndAmPm()
			 *
			 * @param twelveHrMode true or false
			 */
			void set_12hour_mode(const bool);
			/**
			 * \brief Sets RTC i2 addres
			 *
			 * @param addr RTC i2C address
			 */
			void set_rtc_address(const int);
			/**
			 * \brief Sets RTC Model
			 *
			 * @param model RTC Model
			 *	 - #URTCLIB_MODEL_DS1307
			 *	 - #URTCLIB_MODEL_DS3231
			 *	 - #URTCLIB_MODEL_DS3232
			 */
			void set_model(const uint8_t);
			/**
			 * \brief Gets RTC Model
			 *
			 * @return RTC Model
			 *	 - #URTCLIB_MODEL_DS1307
			 *	 - #URTCLIB_MODEL_DS3231
			 *	 - #URTCLIB_MODEL_DS3232
			 */
			uint8_t model();

			/******* Power ********/
			/**
			 * \brief Returns Enable Oscillator Flag
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
			bool getEOSCFlag();
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
			bool lostPower();
			/**
			 * \brief Clears lost power VBAT staus
			 *
			 * DS1307 has a 'CH' Clock Halt Bit in Register 00h ->  When cleared to 0, the oscillator is enabled and time starts incermenting
			 *
			 * Others have a 'OSF' Oscillator Stop Flag in Register 0Fh
			 */
			void lostPowerClear();
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
			bool enableBattery(); // Only DS3231 and DS3232.
			/**
			  *\brief Disable VBAT operation when VCC power is lost.
			  *
			  *
			  * @return True on success
			  */
			bool disableBattery(); // Only DS3231 and DS3232.


			/******** Alarms ************/
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
			bool alarmSet(const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t); // Seconds will be ignored on Alarm 2
			/**
			 * \brief Disables an alarm
			 *
			 * @param alarm Alarm number:
			 *	 - #URTCLIB_ALARM_1
			 *	 - #URTCLIB_ALARM_2
			 *
			 * @return false in case of not supported (DS1307) or wrong parameters
			 */
			bool alarmDisable(const uint8_t);
			/**
			 * \brief Clears an alarm flag
			 *
			 * @param alarm Alarm number:
			 *	 - #URTCLIB_ALARM_1
			 *	 - #URTCLIB_ALARM_2
			 *
			 * @return false in case of not supported (DS1307) or wrong parameters
			 */
			bool alarmClearFlag(const uint8_t);
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
			uint8_t alarmMode(const uint8_t);
			/**
			 * \brief Returns actual alarm second
			 *
			 * @param alarm Alarm number:
			 *	 - #URTCLIB_ALARM_1
			 *	 - #URTCLIB_ALARM_2
			 *
			 * @return Current stored second. 0b11111111 means error.
			 */
			uint8_t alarmSecond(const uint8_t);
			/**
			 * \brief Returns actual alarm minute
			 *
			 * @param alarm Alarm number:
			 *	 - #URTCLIB_ALARM_1
			 *	 - #URTCLIB_ALARM_2
			 *
			 * @return Current stored minute. 0b11111111 means error.
			 */
			uint8_t alarmMinute(const uint8_t);
			/**
			 * \brief Returns actual alarm hour
			 *
			 * @param alarm Alarm number:
			 *	 - #URTCLIB_ALARM_1
			 *	 - #URTCLIB_ALARM_2
			 *
			 * @return Current stored hour. 0b11111111 means error.
			 */
			uint8_t alarmHour(const uint8_t);
			/**
			 * \brief Returns actual alarm day or DOW
			 *
			 * @param alarm Alarm number:
			 *	 - #URTCLIB_ALARM_1
			 *	 - #URTCLIB_ALARM_2
			 *
			 * @return Current stored day or dow. 0b11111111 means error.
			 */
			uint8_t alarmDayDow(const uint8_t);
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
			bool alarmTriggered(const uint8_t);

			/*********** SQWG ************/
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
			uint8_t sqwgMode();
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
			bool sqwgSetMode(const uint8_t);


			/************ RAM *************/
			// Only DS1307 and DS3232.
			// DS1307: Addresses 08h to 3Fh so we offset 08h positions and limit to 38h as maximum address
			// DS3232: Addresses 14h to FFh so we offset 14h positions and limit to EBh as maximum address
			/**
			 * \brief Reads a byte from RTC RAM
			 *
			 * @param address RAM Address
			 *
			 * @return content of that position. If any error it will return always 0xFF;
			 */
			byte ramRead(const uint8_t);
			/**
			 * \brief Writes a byte to RTC RAM
			 *
			 * @param address RAM Address
			 * @param data Content to write on that position
			 *
			 * @return true if correct
			 */
			bool ramWrite(const uint8_t, byte);

			/************ Aging *************/
			// Only DS3231 and DS3232. Address 0x10h
			/**
			 * \brief Reads actual aging value on the RTC
			 *
			 * @return Aging register value on RTC, 2-complement recalculated (use as regular int8_t)
			 */
			int8_t agingGet();
			/**
			 * \brief Sets aging value on the RTC
			 *
			 * @param val new value (use as regular int8_t, 2-complement conversion is done internally)
			 *
			 * @return True when executed, false if RTC doesn't support it.
			 */
			bool agingSet(int8_t);

			/************ 32K Generator pin *************/
			// Only DS3231 and DS3232. On DS1307 we map it to SqWG
			/**
			 * \brief Enables 32K pin output
			 *
			 * A Pull-Up resistor is required on the pin.
			 * As DS1307 doen't have this functionality we map it to SqWG with 32K frequency
			 */
			bool enable32KOut();
			/**
			 * \brief Disable 32K pin output
			 *
			 * As DS1307 doen't have this functionality we map it to SqWG with 32K frequency
			 */
			bool disable32KOut();
			/**
			 * \brief Checks 32K pin output status
			 *
			 * As DS1307 doen't have this functionality we map it to SqWG with 32K frequency
			 */
			bool status32KOut();


		private:
			// Address
			int _rtc_address = URTCLIB_ADDRESS;

			// RTC read data
			uint8_t _second = 0;
			uint8_t _minute = 0;
			uint8_t _hour = 0;
			uint8_t _day = 0;
			uint8_t _month = 0;
			uint8_t _year = 0;
			uint8_t _dayOfWeek = 0;
			int16_t _temp = 9999;

			// Model, for alarms and RAM
			uint8_t _model = URTCLIB_MODEL_DS3232;

			// Alarms:
			uint8_t _a1_mode = URTCLIB_ALARM_TYPE_1_NONE;
			uint8_t _a1_second = 0;
			uint8_t _a1_minute = 0;
			uint8_t _a1_hour = 0;
			uint8_t _a1_day_dow = 0;
			//bool _a1_triggered_flag = _controlStatus  LSB Bit 0

			uint8_t _a2_mode = URTCLIB_ALARM_TYPE_2_NONE;
			uint8_t _a2_minute = 0;
			uint8_t _a2_hour = 0;
			uint8_t _a2_day_dow = 0;
			// bool _a2_triggered_flag = _controlStatus  Bit 1

			// Aging
			int8_t _aging = 0;

			// SQWG
			uint8_t _sqwg_mode = URTCLIB_SQWG_OFF_1;

			// Keep record of various Flags
			// _controlStatus  MSB Bit 7    _lost_power        = (bool) (_controlStatus & 0b10000000);    // Lost power flag
			// _controlStatus  Bit 6        _eosc              = (bool) (_controlStatus & 0b01000000);    // Oscilator enabled flag (negated)
			// _controlStatus  Bit 5        _12hrMode          = (bool) (_controlStatus & 0b00100000);    // 12 or 24h mode
			// _controlStatus  Bit 4        _pmNotAm           = (bool) (_controlStatus & 0b00010000);    // am or pm if 12 hour mode
			// _controlStatus  Bit 3        32K                = (bool) (_controlStatus & 0b00001000);    // 32K
			// _controlStatus  Bit 2        ---                = (bool) (_controlStatus & 0b00000100);    // None
			// _controlStatus  Bit 1        _a2_triggered_flag = (bool) (_controlStatus & 0b00000010);    // Alarm 2 triggered flag
			// _controlStatus  LSB Bit 0    _a1_triggered_flag = (bool) (_controlStatus & 0b00000001);    // Alarm 1 triggered flag
			uint8_t _controlStatus = 0x00;

	};

#endif


