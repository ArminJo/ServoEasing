<div align = center>

# QuadrupedControl a quadruped / mePed V2 spider robot control library
Arduino library for generating all the quadruped movements for controlling a mePed robot V2 with 8 servos.<br/>
It also controls the optional IR receiver and NeoPixels.

[![Badge License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
 &nbsp; &nbsp; 
[![Badge Version](https://img.shields.io/github/v/release/ArminJo/QuadrupedControl?include_prereleases&color=yellow&logo=DocuSign&logoColor=white)](https://github.com/ServoEasing/QuadrupedControl/releases/latest)
 &nbsp; &nbsp; 
[![Badge Commits since latest](https://img.shields.io/github/commits-since/ArminJo/QuadrupedControl/latest?color=yellow)](https://github.com/ArminJo/QuadrupedControl/commits/master)
 &nbsp; &nbsp; 
[![Badge Build Status](https://github.com/ArminJo/QuadrupedControl/workflows/LibraryBuild/badge.svg)](https://github.com/ArminJo/QuadrupedControl/actions)
 &nbsp; &nbsp; 
![Badge Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_QuadrupedControl)
<br/>
<br/>
[![Stand With Ukraine](https://raw.githubusercontent.com/vshymanskyy/StandWithUkraine/main/badges/StandWithUkraine.svg)](https://stand-with-ukraine.pp.ua)

Available as [QuadrupedControl](https://github.com/ArminJo/ServoEasing/tree/master/examples/QuadrupedControl) example of the Arduino library [ServoEasing](https://github.com/ArminJo/ServoEasing).

</div>

#### If you find this library useful, please give it a star.

&#x1F30E; [Google Translate](https://translate.google.com/translate?sl=en&u=https://github.com/ArminJo/QuadrupedControl)

<br/>

# Features
- **Smooth movements** due to the use of [Servo easing library for Arduino](https://github.com/ArminJo/ServoEasing).
- **Demo mode** after initial timeout.
- Predefined **complex movements** like twist, wave and dance.
- Complete **control by a simple IR remote** from an Arduino IR set.
- Speed, direction and height can be controlled independent by the remote.
- If EEPROM existent -which is true for AVR CPU's- the **servos inital positions can be calibrated** and stored by the IR remote.
- Can be extended with Buzzer, NeoPixel strips and a US distance sensor.

<br/>

## YouTube videos

| mePed V2 demo mode | Another implementation |
|-|-|
| [![mePed V2 demo mode](https://i.ytimg.com/vi/MsIjTRRUyGU/hqdefault.jpg)](https://youtu.be/MsIjTRRUyGU) | [![Another implementation](https://i.ytimg.com/vi/CSodffeebyg/hqdefault.jpg)](https://youtu.be/CSodffeebyg) |

For lifting the legs, the lift servos just use the ServoEasing easing type EASE_QUADRATIC_BOUNCING.

### A very simple and easy to understand version of controlling a mePed can be found [here](https://github.com/oracid/Easy-Quadruped-kinematic)

<br/>

# Installation
- Install **[ServoEasing library](https://github.com/ArminJo/ServoEasing)** with *Tools > Manage Libraries...* or *Ctrl+Shift+I*. Use "ServoEasing" as filter string.<br/>
- Open the example **[QuadrupedControl](https://github.com/ArminJo/ServoEasing/tree/master/examples/QuadrupedControl)**, available at File > Examples > Examples from Custom Libraries / ServoEasing.

<br/>

# Compile options / macros for this software
To customize the software to different requirements, there are some compile options / macros available.<br/>
Modify them by enabling / disabling them in the file *QuadrupedConfiguration.h*, or change the values if applicable.

| Name | Default value | Description |
|-|-|-|
| `QUADRUPED_HAS_IR_CONTROL` | disabled | IR remote control is enabled. |
| `IR_RECEIVE_PIN` | A0 | Pin for IR remote control sensor. |
| `PIN_BUZZER` | 3 | Pin for buzzer / piezo. |
| `QUADRUPED_HAS_NEOPIXEL` | disabled | NeoPattern animations on a 24 pieces Neopixel strip handled logically as 3 8 pieces strips is enabled. |
| `QUADRUPED_HAS_US_DISTANCE` | disabled | US distance sensor at pin A3 + A4 is enabled. The distance is displayed on the middle/front 8 pieces of the Neopixel strips. |
| `QUADRUPED_HAS_US_DISTANCE_SERVO` | disabled | A pan servo for the US distance sensor is enabled at pin 13. |
| `QUADRUPED_ENABLE_RTTTL` | disabled | The quadruped plays a melody at startup. |
| `QUADRUPED_1_WITH_DVD_REMOTE`, `QUADRUPED_2_WITH_LAFVIN_REMOTE`, `QUADRUPED_3_WITH_KEYES_CLONE_REMOTE` | disabled | 3 predefined configurations. |

<br/>

# Pictures
| Remote front | Remote back |
|-|-|
| ![Remote front](https://github.com/ArminJo/QuadrupedControl/blob/master/pictures/IRRemoteFront.jpg) | ![Remote back](https://github.com/ArminJo/QuadrupedControl/blob/master/pictures/IRRemoteBack.jpg) |
| Bottom view of my mePed. You can see the two lipos connected parallel resulting in a 4.2 to 3.6 volt supply. | Using a PCA9685 expander for the servos, gaining pins for other purposes. |
| ![Bottom view](https://github.com/ArminJo/QuadrupedControl/blob/master/pictures/mePed_bottom.jpg) | ![PCA9685 expander](https://github.com/ArminJo/QuadrupedControl/blob/master/pictures/mePedWithPCA9685.jpg) |
