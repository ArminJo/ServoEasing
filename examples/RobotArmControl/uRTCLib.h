/**
 * DS1307, DS3231 and DS3232 RTCs basic library
 *
 * Really tiny library to basic RTC functionality on Arduino.
 *
 * Supported features:
 *     * SQuare Wave Generator
 *     * Fixed output pin for DS1307
 *     * RAM for DS1307 and DS3232
 *     * temperature sensor for DS3231 and DS3232
 *     * Alarms (1 and 2) for DS3231 and DS3232
 *
 * See uEEPROMLib for EEPROM support.
 *
 * @copyright Naguissa
 * @author Naguissa
 * @url https://github.com/Naguissa/uRTCLib
 * @url https://www.foroelectro.net/librerias-arduino-ide-f29/rtclib-arduino-libreria-simple-y-eficaz-para-rtc-y-t95.html
 * @email naguissa.com@gmail.com
 * @version 6.0.0
 * @created 2015-05-07
 */
#ifndef URTCLIB
	#define URTCLIB
	#include "Arduino.h"
	#include "Wire.h"
	/*
	RTC I2C Address:
	DS3231 RTC 0x68
	*/
	#define URTCLIB_ADDRESS 0x68

	// Model definition, to work with temperature and alarm(s)
	#define URTCLIB_MODEL_DS1307 1
	#define URTCLIB_MODEL_DS3231 2
	#define URTCLIB_MODEL_DS3232 3

	/*
	ALARM SELECTION:

	*Note: Only valid for DS3231!

	URTCLIB_ALARM_1 - When requesting for Alarm 1
	URTCLIB_ALARM_2 - When requesting for Alarm 2

	ALARM TYPES:

	Alarm 1:
	--------
	URTCLIB_ALARM_TYPE_1_ALL_S - Every second
	URTCLIB_ALARM_TYPE_1_FIXED_S - Every minute at given second
	URTCLIB_ALARM_TYPE_1_FIXED_MS - Every hour at given Minute:Second
	URTCLIB_ALARM_TYPE_1_FIXED_HMS - Every day at given Hour:Minute:Second
	URTCLIB_ALARM_TYPE_1_FIXED_DHMS - Every month at given DAY-Hour:Minute:Second
	URTCLIB_ALARM_TYPE_1_FIXED_WHMS - Every week at given DOW + Hour:Minute:Second

	Alarm 2 (triggers always at 00 seconds):
	----------------------------------------
	URTCLIB_ALARM_TYPE_2_ALL_M - Every minute at 00 Seconds
	URTCLIB_ALARM_TYPE_2_FIXED_M - Every hour at given Minute(:00)
	URTCLIB_ALARM_TYPE_2_FIXED_HM - Every day at given Hour:Minute(:00)
	URTCLIB_ALARM_TYPE_2_FIXED_DHM - Every month at given DAY-Hour:Minute(:00)
	URTCLIB_ALARM_TYPE_2_FIXED_WHM - Every week at given DOW + Hour:Minute(:00)
	*/

	#define URTCLIB_ALARM_TYPE_1_NONE 0b00100000
	#define URTCLIB_ALARM_TYPE_1_ALL_S 0b00010111
	#define URTCLIB_ALARM_TYPE_1_FIXED_S 0b00010110
	#define URTCLIB_ALARM_TYPE_1_FIXED_MS 0b00010100
	#define URTCLIB_ALARM_TYPE_1_FIXED_HMS 0b00010000
	#define URTCLIB_ALARM_TYPE_1_FIXED_DHMS 0b00000000
	#define URTCLIB_ALARM_TYPE_1_FIXED_DOWHMS 0b00001000

	#define URTCLIB_ALARM_TYPE_2_NONE 0b10100000
	#define URTCLIB_ALARM_TYPE_2_ALL_M 0b10001011
	#define URTCLIB_ALARM_TYPE_2_FIXED_M 0b10001010
	#define URTCLIB_ALARM_TYPE_2_FIXED_HM 0b10001000
	#define URTCLIB_ALARM_TYPE_2_FIXED_DHM 0b10000000
	#define URTCLIB_ALARM_TYPE_2_FIXED_DOWHM 0b10000100

	#define URTCLIB_ALARM_1 URTCLIB_ALARM_TYPE_1_NONE
	#define URTCLIB_ALARM_2 URTCLIB_ALARM_TYPE_2_NONE

	#define URTCLIB_SQWG_OFF_0 0b11111111   /* DS1307 only */
	#define URTCLIB_SQWG_OFF_1 0b11111110   /* All */
	#define URTCLIB_SQWG_1H 0b00000000      /* All */
	#define URTCLIB_SQWG_1024H 0b00001000   /* DS3231 and DS3232 only */
	#define URTCLIB_SQWG_4096H 0b00010000   /* All */
	#define URTCLIB_SQWG_8192H 0b00011000   /* All */
	#define URTCLIB_SQWG_32768H 0b00000011  /* DS1307 only */

	// Convert normal decimal numbers to binary coded decimal
	#define uRTCLIB_decToBcd(val) ((uint8_t) ((val / 10 * 16) + (val % 10)))

	// Convert binary coded decimal to normal decimal numbers
	#define uRTCLIB_bcdToDec(val) ((uint8_t) ((val / 16 * 10) + (val % 16)))


	class uRTCLib {
		public:
			// Constructors
			uRTCLib();
			uRTCLib(const int);
			uRTCLib(const int, const uint8_t);

			// RTC functions
			void refresh();
			uint8_t second();
			uint8_t minute();
			uint8_t hour();
			uint8_t day();
			uint8_t month();
			uint8_t year();
			uint8_t dayOfWeek();
			float temp();
			void set(const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t);

			void set_rtc_address(const int);
			void set_model(const uint8_t);
			uint8_t model();

			// Alarms
			bool alarmSet(const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t); // Seconds will be ignored on Alarm 2
			bool alarmDisable(const uint8_t);
			bool alarmClearFlag(const uint8_t);
			uint8_t alarmMode(const uint8_t);
			uint8_t alarmSecond(const uint8_t);
			uint8_t alarmMinute(const uint8_t);
			uint8_t alarmHour(const uint8_t);
			uint8_t alarmDayDow(const uint8_t);

			// SQWG
			uint8_t sqwgMode();
			bool sqwgSetMode(const uint8_t);


			// RAM
			// Only DS1307 and DS3232.
			// DS1307: Addresses 08h to 3Fh so we offset 08h positions and limit to 38h as maximum address
			// DS3232: Addresses 14h to FFh so we offset 14h positions and limit to EBh as maximum address
			byte ramRead(const uint8_t);
			bool ramWrite(const uint8_t, byte);

		private:
			// Address
			int _rtc_address = URTCLIB_ADDRESS;
			// RTC rad data
			uint8_t _second = 0;
			uint8_t _minute = 0;
			uint8_t _hour = 0;
			uint8_t _day = 0;
			uint8_t _month = 0;
			uint8_t _year = 0;
			uint8_t _dayOfWeek = 0;
			float _temp = 9999;

			// Model, for alarms and RAM
			uint8_t _model = URTCLIB_MODEL_DS3232;

			// Alarms:
			uint8_t _a1_mode = URTCLIB_ALARM_TYPE_1_NONE;
			uint8_t _a1_second = 0;
			uint8_t _a1_minute = 0;
			uint8_t _a1_hour = 0;
			uint8_t _a1_day_dow = 0;

			uint8_t _a2_mode = URTCLIB_ALARM_TYPE_2_NONE;
			uint8_t _a2_minute = 0;
			uint8_t _a2_hour = 0;
			uint8_t _a2_day_dow = 0;

			// SQWG
			uint8_t _sqwg_mode = URTCLIB_SQWG_OFF_1;

	};

#endif


