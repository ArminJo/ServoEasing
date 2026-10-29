<div align = center>

# QuadrupedControl a quadruped / mePed V2 spider robot control library
Arduino library for generating all the quadruped movements for controlling a mePed robot V2 with 8 servos.<br/>
It also controls the optional buzzer, IR receiver, US distance sensor and NeoPixels.

[![Badge License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
 &nbsp; &nbsp; 
[![Badge Version](https://img.shields.io/github/v/release/ArminJo/QuadrupedControl?color=yellow&logo=DocuSign&logoColor=white)](https://github.com/ArminJo/QuadrupedControl/releases/latest)
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
- Speed, direction and height can be controlled independently by the remote.
- If EEPROM existent -which is true for AVR CPU's- the **servos initial positions can be calibrated** and stored by the IR remote.
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
This library is not available in the Arduino Library Manager, so it must be installed manually.

### 1. Install the QuadrupedControl library
- Download the repository as ZIP file: on the [GitHub page](https://github.com/ArminJo/QuadrupedControl) click *Code > Download ZIP*.
- In the Arduino IDE, choose *Sketch > Include Library > Add .ZIP Library...* and select the downloaded ZIP file.

Alternatively, unpack the ZIP file or clone the repository into the *libraries* folder of your sketchbook,
e.g. *Documents/Arduino/libraries/QuadrupedControl*.
If you unpack the ZIP file, rename the folder from *QuadrupedControl-master* to *QuadrupedControl*.

### 2. Install the required libraries
These libraries are available in the Library Manager (*Tools > Manage Libraries...* or *Ctrl+Shift+I*):
- [ServoEasing](https://github.com/ArminJo/ServoEasing) - always required.
- [IRremote](https://github.com/Arduino-IRremote/Arduino-IRremote) - required for `QUADRUPED_HAS_IR_CONTROL`.
- [PlayRtttl](https://github.com/ArminJo/PlayRtttl) - required for `QUADRUPED_ENABLE_RTTTL`.
- [NeoPatterns](https://github.com/ArminJo/NeoPatterns) - required for `QUADRUPED_HAS_NEOPIXEL`. Choose *Install all* to also install the required Adafruit NeoPixel library.

### 3. Open, configure and upload the example
- Restart the Arduino IDE.
- Open the example with *File > Examples > Examples from Custom Libraries > QuadrupedControl > QuadrupedControl*.
- Save it to your sketchbook, so you can modify it.
- Select your configuration in *QuadrupedConfiguration.h* of the saved sketch (see below).
- Choose an AVR board like *Arduino Uno* or *Nano* and upload.

## Option 2: Use the example of the ServoEasing library (maybe older version)
The [ServoEasing library](https://github.com/ArminJo/ServoEasing) contains a self-contained copy of the
[QuadrupedControl example](https://github.com/ArminJo/ServoEasing/tree/master/examples/QuadrupedControl),
which includes all quadruped sources in the sketch folder.
- Install the **ServoEasing** library with *Tools > Manage Libraries...* or *Ctrl+Shift+I*. Use "ServoEasing" as filter string.
- For PlayRtttl and NeoPixel support, install also the **PlayRtttl** and **NeoPatterns** libraries (see Option 1).
  The IR receiver code is already contained in the example.
- Open the example with *File > Examples > Examples from Custom Libraries > ServoEasing > QuadrupedControl*, save it to your sketchbook and configure it in *QuadrupedConfiguration.h*.

<br/>

# Operation
### Startup
After power on, all servos move to 90 degree and the body to its default height.
After 2 seconds a short melody (if `QUADRUPED_ENABLE_RTTTL` is enabled) or a beep is played.

### With IR remote (`QUADRUPED_HAS_IR_CONTROL` enabled)
- **Demo mode**: If no IR command has been received since power on, a demo starts after 1 minute, announced by a short beep.
  - If pin A2 (`SHORT_DEMO_MODE_PIN`) is open, the **long demo** runs every 2 minutes:<br/>
    down and up, twist, creep forward, turn 90 degree right, creep back sideways, creep forward, turn 90 degree left, creep back sideways, dance and wave.
  - If pin A2 is connected to ground, every minute one of the **short demo moves** is performed in turn:<br/>
    down and up, twist, turn right and back left, lean in all directions, wave.
  - Demo mode ends with the first IR command. It also ends if a melody or a wave was triggered by the distance sensor.
- **Attention**: 2 minutes after the last IR command, the quadruped moves down and up to remind you to switch it off.
  This is repeated every minute.

### Without IR remote
The **feature move** starts 20 seconds after power on and is repeated every 4 minutes.
It needs more space than the demo, lasts around 3 minutes and shows creep, trot and turn at different speeds as well as dance and wave.

### US distance sensor (`QUADRUPED_HAS_US_DISTANCE_SENSOR` enabled)
The distance is measured 5 times per second. If NeoPixels are attached, it is shown on the front bar:
pixel 1 to 3 represent 5 cm each starting at 5 cm, pixel 4 to 7 represent 10 cm each starting at 20 cm, pixel 8 means 60 cm and more.<br/>
If no movement is running:
- A random melody is played if an object is between 5 and 9 cm for around 0.8 seconds (requires `QUADRUPED_ENABLE_RTTTL`).
- The quadruped waves if an object is between 20 and 29 cm. This is done at most every 10 seconds after the last command or wave.

### Undervoltage
The supply voltage is checked every 10 seconds. If it is below 3.4 volt (3.7 volt if the US distance sensor is enabled) for 6 consecutive checks,
or below 3.0 volt once, the body is lowered and the shutdown melody is played.
Instead of the next attention move or feature move, the shutdown melody (and NeoPixel signal) is repeated.
The next IR command resets the undervoltage state.

<br/>

# IR remote commands
Key assignment for the default remotes KEYES and KEYES_CLONE. For the other remotes see *QuadrupedIRCommandMapping.h*.

| Key | Command |
|-|-|
| Up / Down | Creep forward / backward. While creeping or trotting, the direction is changed. |
| Left / Right | Turn left / right. While creeping or trotting, the direction is changed to sideways. |
| OK | Center all servos |
| # | Stop current movement |
| 1 | Dance and wave |
| 3 | Wave |
| 5 | Feature move |
| 7 | Twist |
| 9 | Trot |
| * | Test: random melody, or twist if RTTTL is disabled |
| 6 / 4 | Increase / decrease speed by 25 % |
| 2 / 8 | Increase / decrease body height |
| 0 | Calibration (see below) |

| Keyes Clone remote front | Remote back |
|-|-|
| ![Remote front](https://github.com/ArminJo/QuadrupedControl/blob/master/pictures/IRRemoteFront.jpg) | ![Remote back](https://github.com/ArminJo/QuadrupedControl/blob/master/pictures/IRRemoteBack.jpg) |

### Calibration
Press 0 to enter calibration mode, starting with the front left leg. The leg which is to be calibrated is indicated by lifting and lowerering it..
Use Left / Right to adjust the pivot servo and Up / Down to adjust the lift servo, until both are exactly at 90 degree.
OK shows the 135 and 45 degree positions, stores the values in EEPROM and switches to the next leg.
After the 4th leg or by pressing any other key, the values are stored and calibration ends.

<br/>

# Compile options / macros for this software
To customize the software to different requirements, there are some compile options / macros available.<br/>
Modify them by enabling / disabling them in the file *QuadrupedConfiguration.h*, or change the values if applicable.

| Name | Default value | Description |
|-|-|-|
| `QUADRUPED_HAS_IR_CONTROL` | disabled | IR remote control is enabled. Requires additionally 8600 bytes program memory. |
| `IR_RECEIVE_PIN` | A0 | Pin for IR remote control sensor. |
| `BUZZER_PIN` | 3 | Pin for buzzer / piezo. |
| `SHORT_DEMO_MODE_PIN` | A2 | If connected to ground, short demo moves are performed every minute instead of the long demo every 2 minutes. |
| `QUADRUPED_HAS_NEOPIXEL` | disabled | NeoPattern animations on a 24 pieces NeoPixel strip at pin 4, handled logically as 3 strips with 8 pieces each. Requires additionally 6300 to 6600 bytes. |
| `QUADRUPED_HAS_US_DISTANCE_SENSOR` | disabled | US distance sensor at pin A3 (trigger) and A4 (echo) is enabled. If Neopixel strips are attached, the distance is displayed on the middle/front 8 Neopixels. Triggers wave and melody, see above. Requires additionally 800 bytes. |
| `QUADRUPED_HAS_US_DISTANCE_SENSOR_SERVO` | disabled | A pan servo for the US distance sensor is enabled at pin 13. Requires additionally 710 bytes. |
| `QUADRUPED_ENABLE_RTTTL` | disabled | The quadruped plays a melody at startup and at undervoltage shutdown, and a random melody at a distance of 5 to 9 cm of the optional distance sensor. Requires additionally 1300 bytes. |
| `QUADRUPED_1_WITH_DVD_REMOTE`, `QUADRUPED_2_WITH_LAFVIN_REMOTE`, `QUADRUPED_3_WITH_KEYES_CLONE_REMOTE` | disabled | 3 predefined configurations for different remote controls. |
| `INFO` | disabled | Prints information about the current action, speed, body height etc. on the serial output. Requires additionally 2850 bytes. |

<br/>

# Pictures
| Top View  of my mePed. You can see the sturdy power connector the blocking capacitor and the IR receiver. | Bottom view. You can see the two lipos connected parallel resulting in a 4.2 to 3.6 volt supply. |
|-|-|
| ![Top view](https://github.com/ArminJo/QuadrupedControl/blob/master/pictures/mePed_top.jpg) | ![Bottom view](https://github.com/ArminJo/QuadrupedControl/blob/master/pictures/mePed_bottom.jpg) |
| | |
| If you require additional pins for other purposes you can use a PCA9685 expander for the servos to free up pins 5 to 12. | View from the other side. |
| ![PCA9685 expander](https://github.com/ArminJo/QuadrupedControl/blob/master/pictures/mePed_topWithPCA9685.jpg) | ![PCA9685 expander](https://github.com/ArminJo/QuadrupedControl/blob/master/pictures/mePedWithPCA9685.jpg) |
| | |
| Details of the attached ultrasonic distance sensor. |  |
| ![US sensor detail](https://github.com/ArminJo/QuadrupedControl/blob/master/pictures/mePedUSSensor.jpg) | |