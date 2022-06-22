/* Arduino SoftI2C library.
 *
 * SoftI2CMasterConfig.h
 *
 * Copyright (C) 2022, Armin Joachimsmeyer
 *
 * This contains a sample configuration setting for SoftI2CMaster.h
 *
 * This file is part of SoftI2CMaster https://github.com/felias-fogg/SoftI2CMaster.
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino I2cMaster Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/* In order to use the library, you need to define SDA_PIN, SCL_PIN,
 * SDA_PORT and SCL_PORT before including this file.  Have a look at
 * http://www.arduino.cc/en/Reference/PortManipulation for finding out
 * which values to use. For example, if you use digital pin 3 (corresponding
 * to PD3) for SDA and digital pin 13 (corresponding to PB5)
 * for SCL on a standard Arduino,
 * you have to use the following definitions:
 * #define SDA_PIN 3
 * #define SDA_PORT PORTD
 * #define SCL_PIN 5
 * #define SCL_PORT PORTB
 *
 * Alternatively, you can define the compile time constant I2C_HARDWARE,
 * in which case the TWI hardware is used. In this case you have to use
 * the standard SDA/SCL pins (and, of course, the chip needs to support
 * this).
 *
 * You can also define the following constants (see also below):
 ' - I2C_PULLUP = 1 meaning that internal pullups should be used
 * - I2C_CPUFREQ, when changing CPU clock frequency dynamically
 * - I2C_FASTMODE = 1 meaning that the I2C bus allows speeds up to 400 kHz
 * - I2C_SLOWMODE = 1 meaning that the I2C bus will allow only up to 25 kHz
 * - I2C_NOINTERRUPT = 1 in order to prohibit interrupts while
 *   communicating (see below). This can be useful if you use the library
 *   for communicating with SMbus devices, which have timeouts.
 *   Note, however, that interrupts are disabled from issuing a start condition
 *   until issuing a stop condition. So use this option with care!
 * - I2C_TIMEOUT = 0...10000 msec in order to return from the I2C functions
 *   in case of a I2C bus lockup (i.e., SCL constantly low). 0 means no timeout.
 * - I2C_MAXWAIT = 0...32767 number of retries in i2c_start_wait. 0 means never stop.
 */
#ifndef _SOFT_I2C_MASTER_CONFIG_H
#define _SOFT_I2C_MASTER_CONFIG_H

//#define SCL_PIN 5
//#define SCL_PORT PORTC
//#define SDA_PIN 4
//#define SDA_PORT PORTC
#define I2C_HARDWARE 1 // use I2C Hardware
#define I2C_PULLUP 1
//#define I2C_TIMEOUT 5000 // costs 350 bytes
#define I2C_FASTMODE 1

#endif // _SOFT_I2C_MASTER_CONFIG_H
