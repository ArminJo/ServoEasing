/*
 * QuadrupedIRConfiguration.h
 *
 * IR remote button codes, strings, and functions to call
 *
 *  Created on: 08.03.2019
 *      Author: Armin
 */

#ifndef SRC_QUADRUPED_IR_CONFIGURATION_H_
#define SRC_QUADRUPED_IR_CONFIGURATION_H_

#include <Arduino.h>
#define IR_RECEIVER_PIN  A0

#if (IR_RECEIVER_PIN != 2) && (IR_RECEIVER_PIN != 3)
#include <PinChangeInterrupt.h> // must be included if we do not use pin 2 or 3
#endif

/*
 * !!! Choose your remote !!!
 */
#if not( defined(USE_KEYES_REMOTE) || defined(USE_WM10_REMOTE))
#define USE_KEYES_REMOTE // the original remote of he mePed v2
//#define USE_WM10_REMOTE
#endif

#ifdef USE_KEYES_REMOTE
/*
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the KEYES remote control with 17 Keys
#define IR_ADDRESS 0x00FF

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
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_OK
#define COMMAND_STOP        IR_HASH
#define COMMAND_CALIBRATE   IR_HASH
#define COMMAND_DANCE       IR_1
#define COMMAND_WAVE        IR_3
#define COMMAND_TWIST       COMMAND_EMPTY
#define COMMAND_TROT        IR_7

#define COMMAND_INCREASE_SPEED  IR_6
#define COMMAND_DECREASE_SPEED  IR_4
#define COMMAND_INCREASE_HEIGHT IR_2
#define COMMAND_DECREASE_HEIGHT IR_8

// locally for doCalibration
#define COMMAND_ENTER       IR_OK
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN
#endif

#ifdef USE_WM10_REMOTE
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
// Mapping from IR buttons to commands for direct use in the code and in the mapping array
#define COMMAND_EMPTY       0 // no command received
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_ENTER
#define COMMAND_STOP        IR_ON_OFF
#define COMMAND_CALIBRATE   IR_MUTE
#define COMMAND_DANCE       IR_SRC
#define COMMAND_WAVE        IR_RETURN
#define COMMAND_TWIST       COMMAND_EMPTY
#define COMMAND_TROT        IR_PLAY_PAUSE

#define COMMAND_INCREASE_SPEED  IR_VOL_PLUS
#define COMMAND_DECREASE_SPEED  IR_VOL_MINUS
#define COMMAND_INCREASE_HEIGHT IR_FAST_FORWARD
#define COMMAND_DECREASE_HEIGHT IR_FAST_BACK

// locally for doCalibration
#define COMMAND_ENTER       IR_ENTER
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN
#endif

#define IR_NEC_REPEAT_ADDRESS 0xFFFF
#define IR_NEC_REPEAT_CODE 0x0

/*
 * THIRD:
 * Main mapping of commands to C functions
 */
// list of functions to call at IR command
// Stationary movements
bool doDance();
bool doWave();
bool doTwist();
bool doBow();
bool doLeanLeft();
bool doLeanRight();

bool doCreepForward();
bool doCreepBack();
bool doTrot();

bool doTurnLeft();
bool doTurnRight();

bool doCenterServos();

bool doCalibration();

/*
 * Instant command functions
 */
bool doSetDirectionForward();
bool doSetDirectionBack();
bool doSetDirectionRight();
bool doSetDirectionLeft();
bool doIncreaseSpeed();
bool doDecreaseSpeed();
bool doIncreaseHeight();
bool doDecreaseHeight();

// IR strings of functions for output
static const char forward[] PROGMEM ="forward";
static const char back[] PROGMEM ="back";
static const char enter[] PROGMEM ="enter";
static const char center[] PROGMEM ="center";
static const char right[] PROGMEM ="right";
static const char left[] PROGMEM ="left";
static const char dirForward[] PROGMEM ="dir forward";
static const char dirBack[] PROGMEM ="dir back";
static const char dirRight[] PROGMEM ="dir right";
static const char dirLeft[] PROGMEM ="dir left";
static const char volPlus[] PROGMEM ="increase speed";
static const char volMinus[] PROGMEM ="decrease speed";
static const char fastBack[] PROGMEM ="decrease height";
static const char fastForward[] PROGMEM ="increase height";
static const char wave[] PROGMEM ="wave";
static const char mute[] PROGMEM ="calibration";
static const char onOff[] PROGMEM ="on/off";
static const char dance[] PROGMEM ="dance";
static const char trot[] PROGMEM ="trot";
static const char twist[] PROGMEM ="twist";
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
struct IRToCommandMapping IRMW10Mapping[] = { { COMMAND_FORWARD, &doCreepForward, forward },
        { COMMAND_BACKWARD, &doCreepBack, back }, {
        COMMAND_RIGHT, &doTurnRight, right }, { COMMAND_LEFT, &doTurnLeft, left }, { COMMAND_CENTER, &doCenterServos, center }, {
        COMMAND_CALIBRATE, &doCalibration, mute }, { COMMAND_DANCE, &doDance, dance }, { COMMAND_TWIST, &doTwist, twist }, {
        COMMAND_WAVE, &doWave, wave }, { COMMAND_TROT, &doTrot, trot } };

struct IRToCommandMapping IRMW10MappingInstantCommands[] = { { COMMAND_FORWARD, &doSetDirectionForward, dirForward }, {
COMMAND_BACKWARD, &doSetDirectionBack, dirBack }, { COMMAND_RIGHT, &doSetDirectionRight, dirRight }, { COMMAND_LEFT,
        &doSetDirectionLeft, dirLeft }, { COMMAND_INCREASE_SPEED, &doIncreaseSpeed, volPlus }, { COMMAND_DECREASE_SPEED,
        &doDecreaseSpeed, volMinus }, {
COMMAND_INCREASE_HEIGHT, &doIncreaseHeight, fastForward }, { COMMAND_DECREASE_HEIGHT, &doDecreaseHeight, fastBack } };

// function to search in MappingInstantCommands array
bool checkAndCallInstantCommands(uint8_t aIRCode);

#endif /* SRC_QUADRUPED_IR_CONFIGURATION_H_ */
