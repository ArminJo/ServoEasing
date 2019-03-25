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
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the WM010 remote control with 14 Keys
#define IR_CONTROL_USED WM010
#define IR_CONTROL_CODING 'N' // NEC

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

#define IR_REPEAT_ADDRESS 0xFFFF
#define IR_REPEAT_CODE 0x0

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
bool doIncreaseSpeed();
bool doDecreaseSpeed();
bool doIncreaseHeight();
bool doDecreaseHeight();

bool doCalibration();

// IR strings of functions for output
static const char forward[] PROGMEM ="forward";
static const char back[] PROGMEM ="back";
static const char enter[] PROGMEM ="enter";
static const char right[] PROGMEM ="right";
static const char left[] PROGMEM ="left";
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
        COMMAND_RIGHT, &doTurnRight, right }, { COMMAND_LEFT, &doTurnLeft, left }, { COMMAND_CENTER, &doCenterServos, enter }, {
        COMMAND_CALIBRATE, &doCalibration, mute }, { COMMAND_DANCE, &doDance, dance }, { COMMAND_TWIST, &doTwist, twist }, {
                COMMAND_WAVE, &doWave, wave }, {
        COMMAND_TROT, &doTrot, trot } };

struct IRToCommandMapping IRMW10MappingInstantCommands[] = { { COMMAND_INCREASE_SPEED, &doIncreaseSpeed, volPlus }, {
COMMAND_DECREASE_SPEED, &doDecreaseSpeed, volMinus }, { COMMAND_INCREASE_HEIGHT, &doIncreaseHeight, fastForward }, {
COMMAND_DECREASE_HEIGHT, &doDecreaseHeight, fastBack } };

// function to search in MappingInstantCommands array
bool checkAndCallInstantCommands(uint8_t aIRCode);

#endif /* SRC_QUADRUPED_IR_CONFIGURATION_H_ */
