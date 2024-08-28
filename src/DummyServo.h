/*
 * DummyServo.h
 *
 *  Dummy servo library as example for a user provided servo library,
 *  which is activated by #define USE_USER_PROVIDED_SERVO_LIB.
 *
 *  Copyright (C) 2024  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  ServoEasing is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

/*
 * We only require:
 *  Servo                       - Class for manipulating servo motors connected to Arduino pins.
 *  attach(pin, min, max)       - Attaches to a pin setting min and max values in microseconds
 *  writeMicroseconds(value)    - Sets the servo pulse width in microseconds
 *  detach()                    - Stops an attached servo from pulsing its i/o pin.
 */

#ifndef _DUMMY_SERVO_H
#define _DUMMY_SERVO_H

#include <inttypes.h>

class Servo
{
public:
  Servo();
  uint8_t attach(int pin, int min, int max); // as above but also sets min and max values for writes.
  void detach();
  void writeMicroseconds(int value); // Write pulse width in microseconds

private:
  /*
   * Not used in this example
   */
   uint8_t servoPin;
   int8_t min;
   int8_t max;
};

#endif // _DUMMY_SERVO_H
