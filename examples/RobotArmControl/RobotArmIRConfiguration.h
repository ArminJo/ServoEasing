/*
 * QuadrupedIRConfiguration.h
 *
 * IR remote button codes, strings, and functions to call
 *
 *  Created on: 08.03.2019
 *      Author: Armin
 */

#ifndef SRC_ROBOT_ARM_IR_CONFIGURATION_H_
#define SRC_ROBOT_ARM_IR_CONFIGURATION_H_

#include <Arduino.h>
#define IR_RECEIVER_PIN  A0

#if (IR_RECEIVER_PIN != 2) && (IR_RECEIVER_PIN != 3)
#include <PinChangeInterrupt.h> // must be included if we do not use pin 2 or 3
#endif

/*
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the WM010 remote control with 14 Keys
#define IR_WM010_ADDRESS 0xF708

#define IR_WM010_UP  0x4
#define IR_WM010_DOWN 0x51
#define IR_WM010_RIGHT 0x8
#define IR_WM010_LEFT 0x14
#define IR_WM010_ENTER 0x7

#define IR_WM010_ON_OFF 0xB
#define IR_WM010_MUTE 0x48

#define IR_WM010_SRC 0x1
#define IR_WM010_RETURN 0x1C

#define IR_WM010_VOL_MINUS 0xD
#define IR_WM010_VOL_PLUS 0x1D

#define IR_WM010_FAST_FORWARD 0x16
#define IR_WM010_FAST_BACK 0x59
#define IR_WM010_PLAY_PAUSE 0x1F

#define IR_NEC_REPEAT_ADDRESS 0xFFFF
#define IR_NEC_REPEAT_CODE 0x0

/*
 * msi Remote control with numbers 1 to 0 and cursor cross below
 */
#define IR_MSI_ADDRESS 0xBD02

#define IR_MSI_UP  0x2
#define IR_MSI_DOWN 0x13
#define IR_MSI_RIGHT 0x10
#define IR_MSI_LEFT 0x11
#define IR_MSI_ENTER 0x7

#define IR_MSI_ON_OFF 0x45
#define IR_MSI_MUTE 0xA

#define IR_MSI_ESC 0x1C
#define IR_MSI_RETURN 0x15

#define IR_MSI_1    0x0
#define IR_MSI_2    0x1
#define IR_MSI_3    0x2
#define IR_MSI_4    0x3
#define IR_MSI_5    0x4
#define IR_MSI_6    0x5
#define IR_MSI_7    0x6
#define IR_MSI_8    0x7
#define IR_MSI_9    0x8
#define IR_MSI_0    0x9

/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
// Mapping from IR buttons to commands for direct use in the code and in the mapping array
#define COMMAND_EMPTY       0 // no command received
#define COMMAND_UP          IR_WM010_UP
#define COMMAND_DOWN        IR_WM010_DOWN
#define COMMAND_RIGHT       IR_WM010_RIGHT
#define COMMAND_LEFT        IR_WM010_LEFT

#define COMMAND_CALIBRATE   IR_WM010_MUTE
#define COMMAND_FORWARD     IR_WM010_SRC
#define COMMAND_BACKWARD    IR_WM010_RETURN

#define COMMAND_INCREASE_SPEED  IR_WM010_VOL_PLUS
#define COMMAND_DECREASE_SPEED  IR_WM010_VOL_MINUS
#define COMMAND_OPEN            IR_WM010_FAST_FORWARD
#define COMMAND_CLOSE           IR_WM010_FAST_BACK

#define COMMAND_CENTER      IR_WM010_ENTER
#define COMMAND_STOP        IR_WM010_ON_OFF
#define COMMAND_FOLD        IR_WM010_PLAY_PAUSE

// locally for doCalibration
#define COMMAND_ENTER       IR_WM010_ENTER

/*
 * THIRD:
 * Main mapping of commands to C functions
 */
// list of functions to call at IR command
// Stationary movements
bool doCenter();
bool doFolded();
bool doGoForward();
bool doGoBack();
bool doTurnRight();
bool doTurnLeft();
bool doLiftUp();
bool doLiftDown();
bool doOpenClaw();
bool doCloseClaw();

bool doAutoMove();
bool doSwitchToManual();
/*
 * Instant commandfunctions
 */
bool doIncreaseSpeed();
bool doDecreaseSpeed();

// IR strings of functions for output
static const char up[] PROGMEM ="up";
static const char down[] PROGMEM ="down";
static const char forward[] PROGMEM ="forward";
static const char back[] PROGMEM ="back";
static const char enter[] PROGMEM ="enter";
static const char center[] PROGMEM ="center";
static const char fold[] PROGMEM ="fold";
static const char right[] PROGMEM ="right";
static const char left[] PROGMEM ="left";
static const char open[] PROGMEM ="open";
static const char close[] PROGMEM ="close";
static const char dirForward[] PROGMEM ="dir forward";
static const char dirBack[] PROGMEM ="dir back";
static const char dirRight[] PROGMEM ="dir right";
static const char dirLeft[] PROGMEM ="dir left";
static const char volPlus[] PROGMEM ="increase speed";
static const char volMinus[] PROGMEM ="decrease speed";
static const char wave[] PROGMEM ="wave";
static const char mute[] PROGMEM ="calibration";
static const char onOff[] PROGMEM ="on/off";
static const char manual[] PROGMEM ="manual";
static const char unknown[] PROGMEM ="unknown";

// Basic mapping structure
struct IRToCommandMapping {
    uint8_t IRCode;
    bool (*CommandToCall)();
    const char * CommandString;
};

/*
 * Main mapping array of commands to C functions and command strings
 */
struct IRToCommandMapping IRMW10Mapping[] = { { COMMAND_FORWARD, &doGoForward, forward }, { COMMAND_BACKWARD, &doGoBack, back }, {
COMMAND_RIGHT, &doTurnRight, right }, { COMMAND_LEFT, &doTurnLeft, left }, { COMMAND_UP, &doLiftUp, up }, {
COMMAND_DOWN, &doLiftDown, down }, { COMMAND_OPEN, &doOpenClaw, open }, { COMMAND_CLOSE, &doCloseClaw, close }, { COMMAND_CENTER,
        &doCenter, center }, { COMMAND_FOLD, &doFolded, fold } };

struct IRToCommandMapping IRMW10MappingInstantCommands[] = { { COMMAND_INCREASE_SPEED, &doIncreaseSpeed, volPlus }, {
COMMAND_DECREASE_SPEED, &doDecreaseSpeed, volMinus }, { COMMAND_STOP, &doSwitchToManual, manual } };

// function to search in MappingInstantCommands array
bool checkAndCallInstantCommands(uint8_t aIRCode);

#endif /* SRC_ROBOT_ARM_IR_CONFIGURATION_H_ */
