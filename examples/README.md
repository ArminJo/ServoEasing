# ServoEasing examples
All examples with up to 2 Servos can be used without modifications with the [Lightweight Servo library](https://github.com/ArminJo/LightweightServo)
for AVR by by [commenting out line 46 in the file ServoEasing.h](https://github.com/ArminJo/ServoEasing#using-the-included-lightweight-servo-library).

YouTube video of SymmetricEasing and AsymmetricEasing example

[![Demonstration of different servo easings](https://i.ytimg.com/vi/fC9uxdOBhfA/hqdefault.jpg)](https://www.youtube.com/watch?v=fC9uxdOBhfA)

## Simple example
This example does not use interrupts and should therefore run on any platform where the Arduino Servo library is available.

## OneServo example
This example moves one Servo with different speeds and using blocking and interrupt commands. The internal LED blinks when using interrupt based commands

## TwoServo and ThreeServo examples
This example shows how to move 2 or 3 servos synchronized or independently.

## SymmetricEasing example
This example shows symmetric (end movement is mirror of start movement) linear, quadratic and cubic movements for 3 servos synchronously.

## AsymmetricEasing example
This example shows asymmetric (end movement is different from start movement) partially **user defined** (line 134) non linear movements for 3 servos synchronously.

## CatMover example
Demo of using two servos in a pan tilt housing to move a laser pointer.

## QuadrupedControl example
This example is available [here](https://github.com/ArminJo/QuadrupedControl)

[![mePed V2 in actions](https://i.ytimg.com/vi/cLgj_sr7f1o/hqdefault.jpg)](https://youtu.be/cLgj_sr7f1o)

## RobotArmControl example
Program for controlling a robot arm with 4 servos using 4 potentiometers and/or an IR Remote.

## PCA9685_Expander example
The OneServo example modified for using a PCA9685 expander board and the standard Arduino Wire library.

# Other examples

## EndPositionsTest example
This example helps you determine the right end values for your servo.<br/>
These values are needed for the `attach()` function, if your servo does not comply to the standard values.
E.g. some of my SG90 servos have a 0 degree period of 620 us instead of the standard 544.<br/>
This example does not use the ServoEasing functions.

## SpeedTest example
This example gives you a feeling how fast your servo can move, what the end position values are and which refresh rate they accept.<br/>
This example does not use the ServoEasing functions.