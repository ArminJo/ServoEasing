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
#include <Arduino.h>
#include <Wire.h>
#include "uRTCLib.h"

/**
 * Constructor
 */
uRTCLib::uRTCLib() {
}

/**
 * Constructor
 *
 * @param int rtc_address I2C address of RTC
 */
uRTCLib::uRTCLib(const int rtc_address) {
    _rtc_address = rtc_address;
}

/**
 * Constructor
 *
 * @param int rtc_address I2C address of RTC
 * @param uint8_t model RTC model, see URTCLIB_MODEL_DSXXXX defines
 */
uRTCLib::uRTCLib(const int rtc_address, const uint8_t model) {
    _rtc_address = rtc_address;
    _model = model;
}

/**
 * Refresh data from HW RTC
 */
void uRTCLib::refresh() {

    Wire.beginTransmission(_rtc_address);
    Wire.write(0); // set DS3231 register pointer to 00h
    Wire.endTransmission();

    // Adjust requested bytes to selected model:
    switch (_model) {
    case URTCLIB_MODEL_DS1307:
        Wire.requestFrom(_rtc_address, 8);
        break;

        // case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
        // case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
    default:
        Wire.requestFrom(_rtc_address, 15);
        break;
    }

    _second = Wire.read();

    _second = uRTCLIB_bcdToDec(_second);

    _minute = Wire.read();

    _minute = uRTCLIB_bcdToDec(_minute);

    _hour = Wire.read() & 0b111111;

    _hour = uRTCLIB_bcdToDec(_hour);

    _dayOfWeek = Wire.read();

    _dayOfWeek = uRTCLIB_bcdToDec(_dayOfWeek);

    _day = Wire.read();

    _day = uRTCLIB_bcdToDec(_day);

    _month = Wire.read();

    _month = uRTCLIB_bcdToDec(_month);

    _year = Wire.read();

    _year = uRTCLIB_bcdToDec(_year);

    _temp = 9999; // Some obvious error value

    // Now we need to read extra requested bytes depending on the RTC model again:
    switch (_model) {
    case URTCLIB_MODEL_DS1307:
        uint8_t status;
        status = Wire.read();
        if (status & 0b00010000) {
            _sqwg_mode = status & 0b10000000 ? URTCLIB_SQWG_OFF_1 : URTCLIB_SQWG_OFF_0;
        } else {
            switch (status & 0b00000011) {
            case 0x00000011:
                _sqwg_mode = URTCLIB_SQWG_32768H;
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
        break;

        // case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
        // case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
    default:
        byte MSB, LSB; // LSB is also used as tmp  variable

        _a1_mode = 0b00000000;
        _a2_mode = 0b10000000;

        _a1_second = Wire.read();

        _a1_mode = _a1_mode | ((_a1_second & 0b10000000) >> 7);
        _a1_second = uRTCLIB_bcdToDec(_a1_second);

        _a1_minute = Wire.read();

        _a1_mode = _a1_mode | ((_a1_minute & 0b10000000) >> 6);
        _a1_minute = uRTCLIB_bcdToDec(_a1_minute);

        _a1_hour = Wire.read() & 0b111111;

        _a1_mode = _a1_mode | ((_a1_hour & 0b10000000) >> 5);
        _a1_hour = uRTCLIB_bcdToDec(_a1_hour);

        _a1_day_dow = Wire.read();

        _a1_mode = _a1_mode | ((_a1_day_dow & 0b10000000) >> 4);
        if (!(_a1_mode & 0b00001111)) {
            _a1_mode = _a1_mode | ((_a1_day_dow & 0b01000000) >> 3);
        }
        _a1_day_dow = uRTCLIB_bcdToDec(_a2_day_dow);

        _a2_minute = Wire.read();

        _a2_mode = _a2_mode | ((_a2_minute & 0b10000000) >> 7);
        _a2_minute = uRTCLIB_bcdToDec(_a2_minute);

        _a2_hour = Wire.read() & 0b111111;

        _a2_mode = _a2_mode | ((_a2_hour & 0b10000000) >> 6);
        _a2_hour = uRTCLIB_bcdToDec(_a2_hour);

        _a2_day_dow = Wire.read();

        _a2_mode = _a2_mode | ((_a2_day_dow & 0b10000000) >> 5);
        if (!(_a2_mode & 0b00000111)) {
            _a2_mode = _a2_mode | ((_a2_day_dow & 0b01000000) >> 4);
        }
        _a2_day_dow = uRTCLIB_bcdToDec(_a2_day_dow);

        // Control registers
        LSB = Wire.read();

        if (LSB & 0b00000100) {
            _sqwg_mode = URTCLIB_SQWG_OFF_1;
            // Alarms disabled?
            if (!(LSB & 0b00000001)) {
                _a1_mode = URTCLIB_ALARM_TYPE_1_NONE;
            }
            if (!(LSB & 0b00000010)) {
                _a2_mode = URTCLIB_ALARM_TYPE_2_NONE;
            }
        } else {
            _sqwg_mode = LSB & 0b00011000;
            // Mark alarms as disabled because the SQWG:
            _a1_mode = URTCLIB_ALARM_TYPE_1_NONE;
            _a2_mode = URTCLIB_ALARM_TYPE_2_NONE;
        }

        // Temperature registers (11h-12h) get updated automatically every 64s
        Wire.beginTransmission(_rtc_address);
        Wire.write(0x11);
        Wire.endTransmission();
        Wire.requestFrom(_rtc_address, 2);

        MSB = Wire.read(); //2's complement int portion
        LSB = Wire.read(); //fraction portion
        _temp = ((((short) MSB << 8) | (short) LSB) >> 6) / 4.0;

        break;
    }
}

/**
 * Returns actual temperature
 *
 * WARNING: Currently only DS3231 is known to have it at a known address
 *
 * @return float Current stored temperature
 */
float uRTCLib::temp() {
    if (_model == URTCLIB_MODEL_DS1307) {
        return 9999;
    }
    return _temp;
}

/**
 * Returns actual second
 *
 * @return uint8_t Current stored second
 */
uint8_t uRTCLib::second() {
    return _second;
}

/**
 * Returns actual minute
 *
 * @return uint8_t Current stored minute
 */
uint8_t uRTCLib::minute() {
    return _minute;
}

/**
 * Returns actual hour
 *
 * @return uint8_t Current stored hour
 */
uint8_t uRTCLib::hour() {
    return _hour;
}

/**
 * Returns actual day
 *
 * @return uint8_t Current stored day
 */
uint8_t uRTCLib::day() {
    return _day;
}

/**
 * Returns actual month
 *
 * @return uint8_t Current stored month
 */
uint8_t uRTCLib::month() {
    return _month;
}

/**
 * Returns actual year
 *
 * @return uint8_t Current stored year
 */
uint8_t uRTCLib::year() {
    return _year;
}

/**
 * Returns actual Day Of Week
 *
 * @return uint8_t Current stored Day Of Week
 */
uint8_t uRTCLib::dayOfWeek() {
    return _dayOfWeek;
}

/**
 * Sets RTC i2 addres
 *
 * @param int RTC i2C address
 */
void uRTCLib::set_rtc_address(const int addr) {
    _rtc_address = addr;
}

/**
 * Sets RTC Model
 *
 * @param uint8_t RTC Model
 */
void uRTCLib::set_model(const uint8_t model) {
    _model = model;
}

/**
 * Gets RTC Model
 *
 * @return uint8_t RTC Model
 */
uint8_t uRTCLib::model() {
    return _model;
}

/**
 * Sets RTC datetime data
 *
 * @param uint8_t second second to set to HW RTC
 * @param uint8_t minute minute to set to HW RTC
 * @param uint8_t hour hour to set to HW RTC
 * @param uint8_t dayOfWeek day of week to set to HW RTC
 * @param uint8_t dayOfMonth day of month to set to HW RTC
 * @param uint8_t month month to set to HW RTC
 * @param uint8_t year year to set to HW RTC
 */
void uRTCLib::set(const uint8_t second, const uint8_t minute, const uint8_t hour, const uint8_t dayOfWeek, const uint8_t dayOfMonth,
        const uint8_t month, const uint8_t year) {

    Wire.beginTransmission(_rtc_address);
    Wire.write(0); // set next input to start at the seconds register
    Wire.write(uRTCLIB_decToBcd(second)); // set seconds
    Wire.write(uRTCLIB_decToBcd(minute)); // set minutes
    Wire.write(uRTCLIB_decToBcd(hour)); // set hours
    Wire.write(uRTCLIB_decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
    Wire.write(uRTCLIB_decToBcd(dayOfMonth)); // set date (1 to 31)
    Wire.write(uRTCLIB_decToBcd(month)); // set month
    Wire.write(uRTCLIB_decToBcd(year)); // set year (0 to 99)
    Wire.endTransmission();

}

/**
 * Alarms:
 */

/**
 * Sets any alarm
 *
 * This method can also be used to disable an alarm, but it's better to use alarmDisable(const uint8_t alarm) to do so.
 *
 * @param uint8_t type Alarm type, see URTCLIB_ALARM_TYPE_X_YYYY defines
 * @param uint8_t second second to set Alarm (ignored in Alarm 2)
 * @param uint8_t minute minute to set Alarm
 * @param uint8_t hour hour to set Alarm
 * @param uint8_t day_dow Day of the month or DOW to set Alarm, depending on alarm type
 *
 * @return bool false in case of not supported (DS1307) or wrong parameters
 */
bool uRTCLib::alarmSet(const uint8_t type, const uint8_t second, const uint8_t minute, const uint8_t hour, const uint8_t day_dow) {
    bool ret = false;
    uint8_t status;

    if (type == URTCLIB_ALARM_TYPE_1_NONE) {
        ret = true;

        // Disable Alarm:
        Wire.beginTransmission(_rtc_address);

        Wire.write(0x0E);

        Wire.endTransmission();

        Wire.requestFrom(_rtc_address, 1);
        status = Wire.read();
        status &= 0b11111110;
        Wire.beginTransmission(_rtc_address);

        Wire.write(0x0E);

        Wire.write(status);

        Wire.endTransmission();

        _a1_mode = type;
    } else if (type == URTCLIB_ALARM_TYPE_2_NONE) {
        ret = true;

        // Disable Alarm:
        Wire.beginTransmission(_rtc_address);

        Wire.write(0x0E);

        Wire.endTransmission();

        Wire.requestFrom(_rtc_address, 1);
        status = Wire.read();
        status &= 0b11111101;
        Wire.beginTransmission(_rtc_address);

        Wire.write(0x0E);

        Wire.write(status);

        Wire.endTransmission();

        _a2_mode = type;
    } else {
        switch (_model) {
        case URTCLIB_MODEL_DS1307:
            ret = false;
            break;

            // case URTCLIB_MODEL_DS3231: // Commented out because it's default mode
            // case URTCLIB_MODEL_DS3232: // Commented out because it's default mode
        default:
            switch (type & 0b11000000) {
            case 0b00000000: // Alarm 1
                ret = true;
                Wire.beginTransmission(_rtc_address);

                Wire.write(0x07); // set next input to start at the seconds register

                Wire.write((uRTCLIB_decToBcd(second) & 0b01111111) | ((type & 0b00000001) << 7)); // set seconds & mode/bit0

                Wire.write((uRTCLIB_decToBcd(minute) & 0b01111111) | ((type & 0b00000010) << 6)); // set minutes & mode/bit1

                Wire.write((uRTCLIB_decToBcd(hour) & 0b01111111) | ((type & 0b00000100) << 5)); // set hours & mode/bit2

                Wire.write((uRTCLIB_decToBcd(day_dow) & 0b00111111) | ((type & 0b00011000) << 3)); // set date / day of week (1=Sunday, 7=Saturday)  & mode/bit3 & mode/DY-DT (bit4)

                Wire.endTransmission();

                // Enable Alarm:
                Wire.beginTransmission(_rtc_address);

                Wire.write(0x0E);

                Wire.endTransmission();

                Wire.requestFrom(_rtc_address, 1);

                status = Wire.read();
                status = status | 0b00000101;  // INTCN and A1IE bits
                Wire.beginTransmission(_rtc_address);

                Wire.write(0x0E);

                Wire.write(status);

                Wire.endTransmission();

                _a1_mode = type;
                _a1_second = second;
                _a1_minute = minute;
                _a1_hour = hour;
                _a1_day_dow = day_dow;
                _sqwg_mode = URTCLIB_SQWG_OFF_1;

                break;

            case 0b10000000: // Alarm 2
                ret = true;
                Wire.beginTransmission(_rtc_address);

                Wire.write(0x07); // set next input to start at the seconds register

                Wire.write((uRTCLIB_decToBcd(minute) & 0b01111111) | ((type & 0b00000010) << 7)); // set minutes & mode/bit1

                Wire.write((uRTCLIB_decToBcd(hour) & 0b01111111) | ((type & 0b00000100) << 6)); // set hours & mode/bit2

                Wire.write((uRTCLIB_decToBcd(day_dow) & 0b00111111) | ((type & 0b00011000) << 4)); // set date / day of week (1=Sunday, 7=Saturday)  & mode/bit3 & mode/DY-DT (bit4)

                Wire.endTransmission();

                // Enable Alarm:
                Wire.beginTransmission(_rtc_address);

                Wire.write(0x0E);

                Wire.endTransmission();

                Wire.requestFrom(_rtc_address, 1);

                status = Wire.read();
                status = status | 0b00000110;  // INTCN and A2IE bits
                Wire.beginTransmission(_rtc_address);

                Wire.write(0x0E);

                Wire.write(status);

                Wire.endTransmission();

                _a2_mode = type;
                _a2_minute = minute;
                _a2_hour = hour;
                _a2_day_dow = day_dow;
                _sqwg_mode = URTCLIB_SQWG_OFF_1;

                break;
            } // Alarm type switch
            break;
        } // model switch

    }
    // if..else
    return ret;
}

/**
 * Disables an alarm
 *
 * @param uint8_t alarm Alarm type, see URTCLIB_ALARM_X defines
 *
 * @return bool false in case of not supported (DS1307) or wrong parameters
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
            break;

        case URTCLIB_ALARM_2: // Alarm 2
            mask = 0b11111101;  // A2IE bit
            break;
        } // Alarm type switch
        if (mask) {
            // Disable Alarm:
            Wire.beginTransmission(_rtc_address);

            Wire.write(0x0E);

            Wire.endTransmission();

            Wire.requestFrom(_rtc_address, 1);
            status = Wire.read();
            status &= 0b11111110;  // A1IE bit
            Wire.beginTransmission(_rtc_address);

            Wire.write(0x0E);

            Wire.write(status);

            Wire.endTransmission();

            _a1_mode = URTCLIB_ALARM_TYPE_1_NONE;
            return true;
        }
        break;
    } // model switch
    return false;
}

/**
 * Disables an alarm
 *
 * @param uint8_t alarm Alarm type, see URTCLIB_ALARM_X defines
 *
 * @return bool false in case of not supported (DS1307) or wrong parameters
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
            Wire.beginTransmission(_rtc_address);

            Wire.write(0x0F);

            Wire.endTransmission();

            Wire.requestFrom(_rtc_address, 1);
            status = Wire.read();
            status &= mask;  // A?F bit
            Wire.beginTransmission(_rtc_address);

            Wire.write(0x0F);

            Wire.write(status);

            Wire.endTransmission();

            return true;
        }
        break;
    } // model switch
    return false;
}

/**
 * Returns actual alarm mode. See URTCLIB_ALARM_TYPE_X_YYYYY defines to see modes
 *
 * @return uint8_t Current stored mode. 0b11111111 means error.
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
 * Returns actual alarm second
 *
 * @return uint8_t Current stored second. 0b11111111 means error.
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
 * Returns actual alarm minute
 *
 * @return uint8_t Current stored minute. 0b11111111 means error.
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
 * Returns actual alarm hour
 *
 * @return uint8_t Current stored hour. 0b11111111 means error.
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
 * Returns actual alarm day or DOW
 *
 * @return uint8_t Current stored day or dow. 0b11111111 means error.
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
 * SQuare Wave Generator
 */

/**
 * Changes SQWG mode, including turning it off
 *
 * @param uint8_t type SQWG mode, see URTCLIB_SQWG_XXXX defines
 *
 * @return bool false in case of not supported (DS1307) or wrong parameters
 */
bool uRTCLib::sqwgSetMode(const uint8_t mode) {
    uint8_t status = 0b00011100;
//    uint8_t processAnd = 0b00000000, processOr = 0b00000000;

    switch (mode) {
    case URTCLIB_SQWG_OFF_1:
        status = 0b00000100; // INTCN
        break;

    case URTCLIB_SQWG_1H:
//        processAnd = 0b11100011; //  RS1, RS0, INTCN
        status = 0b00000000; // nothing
        break;

    case URTCLIB_SQWG_1024H:
//        processAnd = 0b11101011; //  RS1, INTCN
        status = 0b00001000; // RS0
        break;

    case URTCLIB_SQWG_4096H:
//        processAnd = 0b11110011; //  RS0, INTCN
        status = 0b00010000; // RS1
        break;

    case URTCLIB_SQWG_8192H:
//        processAnd = 0b11111011; //  INTCN
        status = 0b00011000; //  RS1, RS0
        break;

    } // mode switch

    Wire.beginTransmission(_rtc_address);

    Wire.write(0x0E);

    Wire.write(status);

    Wire.endTransmission();

    _sqwg_mode = mode;
    if (mode == URTCLIB_SQWG_OFF_1 || mode == URTCLIB_SQWG_OFF_0) {
        _a1_mode = URTCLIB_ALARM_TYPE_1_NONE;
        _a2_mode = URTCLIB_ALARM_TYPE_2_NONE;
    }
    return true;

    return false;
}

/**
 * Gets current SQWG mode
 *
 * @return uint8_t SQWG mode, see URTCLIB_SQWG_XXXX defines
 */
uint8_t uRTCLib::sqwgMode() {
    return _sqwg_mode;
}

/**
 * RAM functionality (Only DS1307. Addresses 08h to 3Fh so we offset 08h positions and limit to 38h as maximum address
 */

/**
 * Reads a byte from RTC RAM
 *
 * @param uint8_t address RAM Address
 *
 * @return byte content of that position. If any error it will return always 0xFF;
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
        if (address < 0xeb) {
            offset = 0x14;
        }
        break;
    }
    if (offset != 0xff) {
        Wire.beginTransmission(_rtc_address);

        Wire.write(address + offset);

        Wire.endTransmission();

        Wire.requestFrom(_rtc_address, 1);

        return Wire.read();
    }
    return 0xff;
}

/**
 * Reads a byte from RTC RAM
 *
 * @param uint8_t address RAM Address
 * @param byte data Content to write on that position
 *
 * @return bool true if correct
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
        if (address < 0xeb) {
            offset = 0x14;
        }
        break;
    }
    if (offset != 0xff) {
        Wire.beginTransmission(_rtc_address);

        Wire.write(address + offset);

        Wire.write(data);

        Wire.endTransmission();

        return true;
    }
    return false;
}

/**
 * EEPROM functionality has been moved to separate library: https://github.com/Naguissa/uEEPROMLib
 */
