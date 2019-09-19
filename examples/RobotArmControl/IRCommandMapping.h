/*
 * IRCommandMapping.h
 *
 * IR remote button codes, strings, and functions to call
 *
 *  Created on: 08.03.2019
 *      Author: Armin
 */

#ifndef IR_COMMAND_MAPING_H_
#define IR_COMMAND_MAPING_H_

#include <Arduino.h>
#include "Commands.h" // includes all the commands used in the mapping arrays below

/*
 * !!! Choose your remote !!!
 */
//#define USE_MSI_REMOTE // Is taken by default
//#define USE_KEYES_REMOTE
//#define USE_WM10_REMOTE
//#define USE_CAR_MP3_REMOTE

// Take MSI remote as default if not otherwise specified
#if !defined(USE_KEYES_REMOTE) && !defined(USE_WM10_REMOTE) && !defined(USE_MSI_REMOTE) && !defined(USE_CAR_MP3_REMOTE)
#define USE_MSI_REMOTE
#endif
//#define USE_KEYES_REMOTE
//#define USE_WM10_REMOTE
//#define USE_MSI_REMOTE
//#define USE_CAR_MP3_REMOTE

#if defined(USE_KEYES_REMOTE) && defined(USE_WM10_REMOTE)
#error "Please choose only one remote for compile"
#endif

// If no remote specified, choose KEYES_REMOTE
#if not( defined(USE_KEYES_REMOTE) || defined(USE_WM10_REMOTE) || defined(USE_MSI_REMOTE) || defined(USE_CAR_MP3_REMOTE))
#define USE_KEYES_REMOTE // the original remote of he mePed v2
#endif

#define IR_NEC_REPEAT_ADDRESS 0xFFFF
#define IR_NEC_REPEAT_CODE 0x0

#ifdef USE_KEYES_REMOTE
#define IR_REMOTE_NAME "KEYES"
/*
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the KEYES remote control with 17 Keys
#define IR_ADDRESS 0xFF00

#define IR_UP    0x46
#define IR_DOWN  0x15
#define IR_RIGHT 0x43
#define IR_LEFT  0x44
#define IR_OK    0x40

#define IR_1    0x16
#define IR_2    0x19
#define IR_3    0x0D
#define IR_4    0x0C
#define IR_5    0x18
#define IR_6    0x5E
#define IR_7    0x08
#define IR_8    0x1C
#define IR_9    0x5A
#define IR_0    0x52

#define IR_STAR 0x42
#define IR_HASH 0x4A
/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
// Mapping from IR buttons to commands for direct use in the code and in the mapping array
#define COMMAND_EMPTY       0 // no command received
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_FORWARD     IR_2
#define COMMAND_BACKWARD    IR_8
#define COMMAND_OPEN        IR_4
#define COMMAND_CLOSE       IR_6

#define COMMAND_INCREASE_SPEED  IR_1
#define COMMAND_DECREASE_SPEED  IR_3

#define COMMAND_CENTER      IR_OK
#define COMMAND_FOLD        IR_HASH
#define COMMAND_MOVE        IR_7
#define COMMAND_EASE_TYPE   IR_9
#define COMMAND_IK_TOGGLE   IR_5

#define COMMAND_STOP        IR_STAR

#endif

#ifdef USE_WM10_REMOTE
#define IR_REMOTE_NAME "WM10"
/*
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the WM010 remote control with 14 Keys
#define IR_ADDRESS 0xF708

#define IR_UP  0x4
#define IR_DOWN 0x51
#define IR_RIGHT 0x8
#define IR_LEFT 0x14
#define IR_ENTER 0x7

#define IR_ON_OFF 0xB
#define IR_MUTE 0x48

#define IR_SRC 0x1
#define IR_RETURN 0x1C

#define IR_VOL_MINUS 0xD
#define IR_VOL_PLUS 0x1D

#define IR_FAST_FORWARD 0x16
#define IR_FAST_BACK 0x59
#define IR_PLAY_PAUSE 0x1F

/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_EMPTY       0 // no command received
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_FORWARD     IR_SRC
#define COMMAND_BACKWARD    IR_RETURN
#define COMMAND_OPEN        IR_FAST_FORWARD
#define COMMAND_CLOSE       IR_FAST_BACK

#define COMMAND_INCREASE_SPEED  IR_VOL_PLUS
#define COMMAND_DECREASE_SPEED  IR_VOL_MINUS

#define COMMAND_CENTER      IR_ENTER
#define COMMAND_FOLD        IR_PLAY_PAUSE
#define COMMAND_MOVE        IR_MUTE

#define COMMAND_STOP        IR_ON_OFF
#endif

/*
 * msi Remote control with numbers 1 to 0 and cursor cross below
 */
#ifdef USE_MSI_REMOTE
#define IR_REMOTE_NAME "MSI"

#define IR_ADDRESS 0xBD02

#define IR_UP  0x12
#define IR_DOWN 0x13
#define IR_RIGHT 0x10
#define IR_LEFT 0x11
#define IR_OK 0x15

#define IR_ON_OFF 0x45
#define IR_MUTE 0xA

#define IR_ESC 0x1C

#define IR_1    0x0
#define IR_2    0x1
#define IR_3    0x2
#define IR_4    0x3
#define IR_5    0x4
#define IR_6    0x5
#define IR_7    0x6
#define IR_8    0x7
#define IR_9    0x8
#define IR_0    0x9

/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_EMPTY       0xFF // no command received, use 0xFF since 0x00 is a valid key on this remote
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_FORWARD     IR_2
#define COMMAND_BACKWARD    IR_8
#define COMMAND_OPEN        IR_4
#define COMMAND_CLOSE       IR_6

#define COMMAND_INCREASE_SPEED  IR_ESC
#define COMMAND_DECREASE_SPEED  IR_0

#define COMMAND_CENTER      IR_OK
#define COMMAND_FOLD        IR_MUTE
#define COMMAND_MOVE        IR_7

#define COMMAND_IK_ON       IR_3
#define COMMAND_IK_OFF      IR_1
#define COMMAND_EASE_TYPE   IR_9

#define COMMAND_STOP        IR_ON_OFF
#define COMMAND_TEST        IR_5
#endif

#ifdef USE_CAR_MP3_REMOTE
#define IR_REMOTE_NAME "CAR MP3"

#define IR_ADDRESS 0xFF00

#define IR_CH_MINUS 0x45
#define IR_CH       0x46
#define IR_CH_PLUS  0x47

#define IR_FAST_BACK    0x44
#define IR_FAST_FORWARD 0x40
#define IR_PLAY_PAUSE   0x43

#define IR_MINUS    0x7
#define IR_PLUS     0x15
#define IR_EQ       0x9

#define IR_0    0x16
#define IR_100  0x19
#define IR_200  0xD

#define IR_1    0xC
#define IR_2    0x18
#define IR_3    0x5E

#define IR_4    0x8
#define IR_5    0x1C
#define IR_6    0x5A

#define IR_7    0x42
#define IR_8    0x52
#define IR_9    0x4A


/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_EMPTY       0 // no command received
#define COMMAND_UP          IR_2
#define COMMAND_DOWN        IR_8
#define COMMAND_RIGHT       IR_4
#define COMMAND_LEFT        IR_6

#define COMMAND_FORWARD     IR_CH_PLUS
#define COMMAND_BACKWARD    IR_CH_MINUS
#define COMMAND_OPEN        IR_FAST_BACK
#define COMMAND_CLOSE       IR_FAST_FORWARD

#define COMMAND_INCREASE_SPEED  IR_PLUS
#define COMMAND_DECREASE_SPEED  IR_MINUS

#define COMMAND_CENTER      IR_5
#define COMMAND_FOLD        IR_EQ
#define COMMAND_MOVE        IR_CH

#define COMMAND_STOP        IR_PLAY_PAUSE

#define COMMAND_IK_ON       IR_200
#define COMMAND_IK_OFF      IR_9
#define COMMAND_EASE_TYPE   IR_3

#define COMMAND_TEST        IR_100
#endif

/*
 * This is valid for all remotes above
 */
#define IR_REPEAT_ADDRESS IR_NEC_REPEAT_ADDRESS
#define IR_REPEAT_CODE IR_NEC_REPEAT_CODE

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
static const char volPlus[] PROGMEM ="increase speed";
static const char volMinus[] PROGMEM ="decrease speed";
static const char move[] PROGMEM ="auto move";
static const char onOff[] PROGMEM ="on/off";
static const char manual[] PROGMEM ="manual";
static const char ik_on[] PROGMEM ="IK on";
static const char ik_off[] PROGMEM ="IK off";
static const char ik_toggle[] PROGMEM ="toggle IK";
static const char type[] PROGMEM ="easing type";
static const char test[] PROGMEM ="test";
static const char unknown[] PROGMEM ="unknown";

/*
 * THIRD:
 * Main mapping of commands to C functions
 */

// Basic mapping structure
struct IRToCommandMapping {
    uint8_t IRCode;
    void (*CommandToCall)();
    const char * CommandString;
};

/*
 * Main mapping array of commands to C functions and command strings
 */
struct IRToCommandMapping IRMapping[] = { { COMMAND_FORWARD, &doGoForward, forward }, { COMMAND_BACKWARD, &doGoBack, back }, {
COMMAND_RIGHT, &doTurnRight, right }, { COMMAND_LEFT, &doTurnLeft, left }, { COMMAND_UP, &doLiftUp, up }, {
COMMAND_DOWN, &doLiftDown, down }, { COMMAND_OPEN, &doOpenClaw, open }, { COMMAND_CLOSE, &doCloseClaw, close }, { COMMAND_CENTER,
        &doCenter, center }, { COMMAND_FOLD, &doFolded, fold }, { COMMAND_MOVE, &doAutoMove, move }, { COMMAND_TEST, &doStartClock, test }  };

struct IRToCommandMapping IRMappingInstantCommands[] = { { COMMAND_INCREASE_SPEED, &doIncreaseSpeed, volPlus }, {
COMMAND_DECREASE_SPEED, &doDecreaseSpeed, volMinus }, { COMMAND_STOP, &doSwitchToManual, manual },
#ifdef COMMAND_IK_ON
        { COMMAND_IK_ON, &doInverseKinematicOn, ik_on }, { COMMAND_IK_OFF, &doInverseKinematicOff, ik_off },
#endif
#ifdef COMMAND_IK_TOGGLE
        {   COMMAND_IK_TOGGLE, &doToggleInverseKinematic, ik_toggle},
#endif
        { COMMAND_EASE_TYPE, &doSwitchEasingType, type } };

#endif /* IR_COMMAND_MAPING_H_ */

#pragma once
