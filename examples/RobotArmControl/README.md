<div align = center>

# RobotArmControl for a MeArm V0.4 robot
Program for controlling a [robot arm with 4 servos](https://www.instructables.com/id/4-DOF-Mechanical-Arm-Robot-Controlled-by-Arduino) using 4 potentiometers and/or an IR Remote.

Smooth servo movements are controlled by the [Servo easing library for Arduino](https://github.com/ArminJo/ServoEasing).<br/>
The code is available as [RobotArmControl](https://github.com/ArminJo/ServoEasing/tree/master/examples/RobotArmControl) example of the Arduino library [ServoEasing](https://github.com/ArminJo/ServoEasing/tree/master/examples/RobotArmControl).

</div>

&#x1F30E; [Google Translate](https://translate.google.com/translate?sl=en&u=https://github.com/ArminJo/ServoEasing/tree/master/examples/RobotArmControl)

<br/>

| RobotArm | Control board | Instructable |
|-|-|-|
| ![RobotArm](https://github.com/ArminJo/ServoEasing/blob/master/pictures/RobotArmBlack.jpg) | ![Control board](https://github.com/ArminJo/ServoEasing/blob/master/pictures/RobotArmControlBoard.jpg) | [![Instructable](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/instructables-logo-v2.png)](https://www.instructables.com/id/4-DOF-Mechanical-Arm-Robot-Controlled-by-Arduino) |

<br/>

# Calibration
To calibrate your robot arm, open the Serial Monitor, move the arm manually and change the microsecond values for the `PIVOT_MICROS_AT_*`, `LIFT_MICROS_AT_*`, `HORIZONTAL_MICROS_AT_*` and `CLAW_MICROS_AT_*` positions in *RobotArmServoConfiguration.h*.

The example uses the `EASE_USER_DIRECT` easing type for all servos except the claw to implement **movements by inverse kinematics**.

<br/>

# Compile options / macros for this software
To customize the software to different requirements, there are some compile options / macros available.<br/>
Modify them by enabling / disabling them, or change the values if applicable.

| Name | Default value |Description |
|-|-|-|
| `ROBOT_ARM_HAS_IR_CONTROL` | disabled | The movements can be controlled by an IR remote. |
| `ROBOT_ARM_DRAWS_RTC_TIME` | disabled | A DS3231 is attached and the robot arm draws the time into a sand bed - still experimental. |

