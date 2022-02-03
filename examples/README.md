# ServoEasing examples
All examples with up to 2 Servos can be used without modifications with the [Lightweight Servo library](https://github.com/ArminJo/LightweightServo)
for AVR by by [commenting out the line `#define USE_LEIGHTWEIGHT_SERVO_LIB` in the file ServoEasing.h](https://github.com/ArminJo/ServoEasing#using-the-included-lightweight-servo-library).

YouTube video of SymmetricEasing and AsymmetricEasing example

[![Demonstration of different servo easings](https://i.ytimg.com/vi/fC9uxdOBhfA/hqdefault.jpg)](https://www.youtube.com/watch?v=fC9uxdOBhfA)

# Table of content
  * [Simple example](#simple-example)
  * [OneServo example](#oneservo-example)
  * [TwoServo](#twoservo)
  * [ThreeServo examples](#threeservo-examples)
  * [SymmetricEasing example](#symmetriceasing-example)
  * [AsymmetricEasing example](#asymmetriceasing-example)
  * [ContinuousRotatingServo example](#continuousrotatingservo-example)
  * [LightweightServoExample example](#lightweightservoexample-example)
  * [CatMover example](#catmover-example)
  * [QuadrupedControl example](#quadrupedcontrol-example)
  * [RobotArmControl example](#robotarmcontrol-example)
  * [PCA9685_Expander example](#pca9685_expander-example)
  * [PCA9685_ExpanderAndServo example](#pca9685_expanderandservo-example)
  * [PCA9685_ExpanderFor32Servos example](#pca9685_expanderfor32servos-example)
  
## [Simple example](https://github.com/ArminJo/ServoEasing/blob/master/examples/Simple/Simple.ino)
This example does not use interrupts and should therefore run on any platform where the Arduino Servo library is available.<br/><br/>
**Arduino Serial Plotter** result of this example if `#define PRINT_FOR_SERIAL_PLOTTER` in the library file *ServoEasing.h* is enabled.<br/>
![Arduino plot](https://github.com/ArminJo/ServoEasing/blob/master/pictures/ServoEasing-Linear-Cubic-Circular.png)

## [OneServo example](https://github.com/ArminJo/ServoEasing/blob/master/examples/OneServo/OneServo.ino)
This example moves one Servo with different speeds and using blocking and interrupt commands. The internal LED blinks when using interrupt based commands.

## [TwoServo](https://github.com/ArminJo/ServoEasing/blob/master/examples/TwoServo/TwoServo.ino)
This example shows how to move 2 servos attached at pin 9 and 10 synchronized or independently using the LightweightServo library. This saves 640 bytes program memory compared to using Arduino Servo library.

## [ThreeServo examples](https://github.com/ArminJo/ServoEasing/blob/master/examples/ThreeServo/ThreeServo.ino)
This example shows how to move 3 servos synchronized or independently.<br/>
WOKWI online simulation of the ThreeServo example.<br/>
[![WOKWI online simulation of the ThreeServo example](https://github.com/ArminJo/ServoEasing/blob/master/pictures/Wokwi_ThreeServos.png)](https://wokwi.com/arduino/projects/299552195816194570).

## [SymmetricEasing example](https://github.com/ArminJo/ServoEasing/blob/master/examples/SymmetricEasing/SymmetricEasing.ino)
This example shows symmetric (end movement is mirror of start movement) linear, quadratic and cubic movements for 3 servos synchronously.
**Arduino Serial Plotter** result of this example if `#define PRINT_FOR_SERIAL_PLOTTER` in the library file *ServoEasing.h* is enabled.<br/>
![Arduino plot](https://github.com/ArminJo/ServoEasing/blob/master/pictures/SymmetricEasing.png)

## [AsymmetricEasing example](https://github.com/ArminJo/ServoEasing/blob/master/examples/AsymmetricEasing/AsymmetricEasing.ino)
This example shows asymmetric (end movement is different from start movement) non linear movements for 3 servos synchronously.
It includes a partially **user defined easing function**  `EaseQuadraticInQuarticOut()`.
**Arduino Serial Plotter** result of this example if `#define PRINT_FOR_SERIAL_PLOTTER` in the library file *ServoEasing.h* is enabled.<br/>
![Arduino plot](https://github.com/ArminJo/ServoEasing/blob/master/pictures/AsymmetricEasing.png)

## [ContinuousRotatingServo example](https://github.com/ArminJo/ServoEasing/blob/master/examples/ContinuousRotatingServo/ContinuousRotatingServo.ino)
Example for using the servoEasing library to create speed ramps for a continuous rotating servo. This example rely on your servos stop value being **exacly 1500 microseconds**. If the stop value of your servo is NOT exactly 1500 microseconds, you must modify the `MICROSECONDS_FOR_ROTATING_SERVO_STOP` value in the library file *ServoEasing.h*.

## [LightweightServoExample example](https://github.com/ArminJo/ServoEasing/blob/master/examples/LightweightServoExample/LightweightServoExample.ino)
This example moves 2 servos attached at pin 9 and 10 using the LightweightServo library for ATmega328*.

## [CatMover example](https://github.com/ArminJo/ServoEasing/blob/master/examples/CatMover/CatMover.ino)
Demo of using two servos in a pan tilt housing to move a laser pointer.

## [QuadrupedControl example](https://github.com/ArminJo/ServoEasing/blob/master/examples/QuadrupedControl/QuadrupedControl.ino)
Control 8 servos to move a Quadruped robot.<br/>
The full example with IR remote control, NeoPixel and US distance sensor support is available [here](https://github.com/ArminJo/QuadrupedControl).
Only for AVR, because it uses EEPROM.

### YouTube Videos
[![mePed V2 in actions](https://i.ytimg.com/vi/MsIjTRRUyGU/hqdefault.jpg)](https://youtu.be/MsIjTRRUyGU)
[![Another implementation](https://i.ytimg.com/vi/CSodffeebyg/hqdefault.jpg)](https://youtu.be/CSodffeebyg)

## [RobotArmControl example](https://github.com/ArminJo/ServoEasing/blob/master/examples/RobotArmControl/RobotArmControl.ino)
Program for controlling a [robot arm with 4 servos](https://www.instructables.com/id/4-DOF-Mechanical-Arm-Robot-Controlled-by-Arduino) using 4 potentiometers and/or an IR Remote.
Only for AVR, because it uses EEPROM.

## [PCA9685_Expander example](https://github.com/ArminJo/ServoEasing/blob/master/examples/PCA9685_Expander/PCA9685_Expander.ino)
The OneServo example modified for using a PCA9685 expander board and the standard Arduino Wire library.<br/>
You must activate the line `#define USE_PCA9685_SERVO_EXPANDER` in *ServoEasing.h* to make the expander example work.

## [PCA9685_ExpanderAndServo example](https://github.com/ArminJo/ServoEasing/blob/master/examples/PCA9685_ExpanderAndServo/PCA9685_ExpanderAndServo.ino)
Combination of OneServo example and PCA9685_Expander example. Move one servo attached to the Arduino board and one servo attached to the PCA9685 expander board **simultaneously**.

## [PCA9685_ExpanderFor32Servos example](https://github.com/ArminJo/ServoEasing/blob/master/examples/PCA9685_ExpanderFor32Servos/PCA9685_ExpanderFor32Servos.ino)
Program to show the usage of 2 PCA9685 expander boards with 32 servos.
On the ESP32, the I2C library interferes with the 29 millisecond timer and therefore can only run at 100000 Hz or lower.<br/>
You must activate the line `#define USE_PCA9685_SERVO_EXPANDER` in *ServoEasing.h* to make the expander example work.

### YouTube Video
[![Servos 16-19 and 28-31 in action](https://i.ytimg.com/vi/XMVh3IT5BgU/hqdefault.jpg)](https://youtu.be/XMVh3IT5BgU)


### mePed V2 with PCA9685 expander
| | |
|-|-|
| ![mePed V2 with PCA9685 expander](https://github.com/ArminJo/ServoEasing/blob/master/pictures/mePedWithPCA9685.jpg) | ![mePed V2 with PCA9685 expander](https://github.com/ArminJo/ServoEasing/blob/master/pictures/mePed_topWithPCA9685.jpg) |



# Servo utilities

## [EndPositionsTest example](https://github.com/ArminJo/ServoEasing/blob/master/examples/EndPositionsTest/EndPositionsTest.ino)
This example helps you determine the right end values for your servo.<br/>
These values are required for the `attach(int aPin, int aInitialDegree, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree)` function, if your servo does not comply to the standard values.
E.g. some of my SG90 servos have a 0 degree period of 620 µs instead of the standard 544.<br/>
This example does not use the ServoEasing functions.

## [SpeedTest example](https://github.com/ArminJo/ServoEasing/blob/master/examples/SpeedTest/SpeedTest.ino)
This example gives you a feeling how fast your servo can move, what the end position values are and which refresh rate they accept.<br/>
This example does not use the ServoEasing functions.
Not for ESP8266 because it requires 2 analog inputs.