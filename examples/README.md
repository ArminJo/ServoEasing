# ServoEasing examples
All examples with up to 2 Servos can be used without modifications with the [Lightweight Servo library](https://github.com/ArminJo/LightweightServo)
for AVR by by [commenting out line 44 `#define USE_LEIGHTWEIGHT_SERVO_LIB` in the file ServoEasing.h](https://github.com/ArminJo/ServoEasing#using-the-included-lightweight-servo-library).

YouTube video of SymmetricEasing and AsymmetricEasing example

[![Demonstration of different servo easings](https://i.ytimg.com/vi/fC9uxdOBhfA/hqdefault.jpg)](https://www.youtube.com/watch?v=fC9uxdOBhfA)

## Simple example
This example does not use interrupts and should therefore run on any platform where the Arduino Servo library is available.<br/><br/>
**Arduino Serial Plotter** result of this example if `#define PRINT_FOR_SERIAL_PLOTTER` in the library file *ServoEasing.h* is enabled.<br/>
![Arduino plot](https://github.com/ArminJo/ServoEasing/blob/master/pictures/ServoEasing-Linear-Cubic-Circular.png)

## OneServo example
This example moves one Servo with different speeds and using blocking and interrupt commands. The internal LED blinks when using interrupt based commands

## TwoServo and ThreeServo examples
This example shows how to move 2 or 3 servos synchronized or independently.

## SymmetricEasing example
This example shows symmetric (end movement is mirror of start movement) linear, quadratic and cubic movements for 3 servos synchronously.
**Arduino Serial Plotter** result of this example if `#define PRINT_FOR_SERIAL_PLOTTER` in the library file *ServoEasing.h* is enabled.<br/>
![Arduino plot](https://github.com/ArminJo/ServoEasing/blob/master/pictures/SymmetricEasing.png)

## AsymmetricEasing example
This example shows asymmetric (end movement is different from start movement) non linear movements for 3 servos synchronously.
It includes a partially **user defined easing function**  `EaseQuadraticInQuarticOut()`.
**Arduino Serial Plotter** result of this example if `#define PRINT_FOR_SERIAL_PLOTTER` in the library file *ServoEasing.h* is enabled.<br/>
![Arduino plot](https://github.com/ArminJo/ServoEasing/blob/master/pictures/AsymmetricEasing.png)

## ContinuousRotatingServo example
Example for using the servoEasing library to create speed ramps for a continuous rotating servo. This example rely on your servos stop value being **exacly 1500 microseconds**. If the stop value of your servo is NOT exactly 1500 microseconds, you must modify the `MICROSECONDS_FOR_ROTATING_SERVO_STOP` value in the library file *ServoEasing.h*.

## CatMover example
Demo of using two servos in a pan tilt housing to move a laser pointer.

## QuadrupedControl example
Control 8 servos to move a Quadruped robot.<br/>
The full example with IR remote control, NeoPixel and US distance sensor support is available [here](https://github.com/ArminJo/QuadrupedControl).

### YouTube Video
[![mePed V2 in actions](https://i.ytimg.com/vi/cLgj_sr7f1o/hqdefault.jpg)](https://youtu.be/cLgj_sr7f1o)

## RobotArmControl example
Program for controlling a [robot arm with 4 servos](https://www.instructables.com/id/4-DOF-Mechanical-Arm-Robot-Controlled-by-Arduino) using 4 potentiometers and/or an IR Remote.

## PCA9685_Expander example
The OneServo example modified for using a PCA9685 expander board and the standard Arduino Wire library.

# Servo utilities

## EndPositionsTest example
This example helps you determine the right end values for your servo.<br/>
These values are needed for the `attach()` function, if your servo does not comply to the standard values.
E.g. some of my SG90 servos have a 0 degree period of 620 us instead of the standard 544.<br/>
This example does not use the ServoEasing functions.

## SpeedTest example
This example gives you a feeling how fast your servo can move, what the end position values are and which refresh rate they accept.<br/>
This example does not use the ServoEasing functions.