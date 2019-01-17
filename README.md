# SmoothServo

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://travis-ci.org/ArminJo/SmoothServo.svg?branch=master)](https://travis-ci.org/ArminJo/SmoothServo)

## Smooth servo movement library for Arduino
This is an extension to the Arduino Servo library.
It enables linear movement for all servos attached to the Arduino Servo library. All servos can move synchronized.<br/>
You have the choice not to call the update function, since we can use interrupts of the servo timer to call update internally.<br/>Until now only one timer is supported, which means not more than 12 servos are supported.

## Examples
The `EndPositionsTest` example helps you determine the right end values for your servo.<br/>These values are needed for the `attach()` function, if your servo does not comply to the standard values. E.g. my SG90 servos have a lower period of 620 instead of the standard 544.<br/>This example does not use the SmoothServo functions.

The `SpeedTest` example gives you a feeling how fast your servo can move.<br/>This example does not use the SmoothServo functions.
