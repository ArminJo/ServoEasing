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
//#define USE_KEYES_REMOTE // The mePed 2 Standard remote, will be taken as default
//#define USE_WM10_REMOTE
//#define USE_KEYES_REMOTE_CLONE With number pad and direction control switched
// Take KEYES remote as default if not otherwise specified
#if !defined(USE_KEYES_REMOTE) && !defined(USE_WM10_REMOTE) && !defined(USE_KEYES_REMOTE_CLONE)
#define USE_KEYES_REMOTE // the mePed 2 Standard remote
#endif

#if defined(USE_KEYES_REMOTE) && defined(USE_WM10_REMOTE)
#error "Please choose only one remote for compile"
#endif

#define IR_NEC_REPEAT_ADDRESS 0xFFFF
#define IR_NEC_REPEAT_CODE 0x0

#ifdef USE_KEYES_REMOTE_CLONE
#define IR_REMOTE_NAME "KEYES_CLONE"
// Codes for the KEYES CLONE remote control with 17 Keys with keypad above direction control
#define IR_ADDRESS 0xFF00

#define IR_UP    0x18
#define IR_DOWN  0x52
#define IR_RIGHT 0x5A
#define IR_LEFT  0x08
#define IR_OK    0x1C

#define IR_1    0x46
#define IR_2    0x45
#define IR_3    0x47
#define IR_4    0x44
#define IR_5    0x40
#define IR_6    0x43
#define IR_7    0x07
#define IR_8    0x15
#define IR_9    0x09
#define IR_0    0x19

#define IR_STAR 0x16
#define IR_HASH 0x0D
/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_EMPTY       0 // no command received
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_OK
#define COMMAND_STOP        IR_HASH
#define COMMAND_CALIBRATE   IR_0
#define COMMAND_DANCE       IR_1
#define COMMAND_WAVE        IR_3
#define COMMAND_TWIST       IR_9
#define COMMAND_TROT        IR_7
#define COMMAND_AUTO        IR_5
#define COMMAND_TEST        IR_STAR

#define COMMAND_INCREASE_SPEED  IR_6
#define COMMAND_DECREASE_SPEED  IR_4
#define COMMAND_INCREASE_HEIGHT IR_2
#define COMMAND_DECREASE_HEIGHT IR_8

// locally for doCalibration
#define COMMAND_ENTER       IR_OK
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN
#endif

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
#define COMMAND_EMPTY       0 // no command received
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_OK
#define COMMAND_STOP        IR_HASH
#define COMMAND_CALIBRATE   IR_0
#define COMMAND_DANCE       IR_1
#define COMMAND_WAVE        IR_3
#define COMMAND_TWIST       IR_9
#define COMMAND_TROT        IR_7
#define COMMAND_AUTO        IR_5
#define COMMAND_TEST        IR_STAR

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
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_ENTER
#define COMMAND_STOP        IR_ON_OFF
#define COMMAND_CALIBRATE   IR_MUTE
#define COMMAND_DANCE       IR_SRC
#define COMMAND_WAVE        IR_RETURN
#define COMMAND_TWIST       COMMAND_EMPTY // not on this remote
#define COMMAND_TROT        IR_PLAY_PAUSE
#define COMMAND_AUTO        COMMAND_EMPTY // not on this remote
#define COMMAND_TEST        COMMAND_EMPTY // not on this remote

#define COMMAND_INCREASE_SPEED  IR_VOL_PLUS
#define COMMAND_DECREASE_SPEED  IR_VOL_MINUS
#define COMMAND_INCREASE_HEIGHT IR_FAST_FORWARD
#define COMMAND_DECREASE_HEIGHT IR_FAST_BACK

// locally for doCalibration
#define COMMAND_ENTER       IR_ENTER
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN
#endif

/*
 * This is valid for all remotes above
 */
#define IR_REPEAT_ADDRESS IR_NEC_REPEAT_ADDRESS
#define IR_REPEAT_CODE IR_NEC_REPEAT_CODE

/*
 * THIRD:
 * Main mapping of commands to C functions
 */

// IR strings of functions for output
static const char beep[] PROGMEM ="beep";
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
//static const char onOff[] PROGMEM ="on/off";
static const char stop[] PROGMEM ="stop";
static const char dance[] PROGMEM ="dance";
static const char trot[] PROGMEM ="trot";
static const char twist[] PROGMEM ="twist";
static const char autoMove[] PROGMEM ="auto move";
static const char myMove[] PROGMEM ="my move";
static const char test[] PROGMEM ="test";
static const char unknown[] PROGMEM ="unknown";

// Basic mapping structure
struct IRToCommandMapping {
    uint8_t IRCode;
    void (*CommandToCall)();
    const char * CommandString;
};

#ifndef EMPTY_MAPPING
/*
 * Main mapping array of commands to C functions and command strings
 */
const struct IRToCommandMapping IRMapping[] = { { COMMAND_RIGHT, &doTurnRight, right }, { COMMAND_LEFT, &doTurnLeft, left }, {
COMMAND_CENTER, &doCenterServos, center }, { COMMAND_FORWARD, &doCreepForward, forward }, { COMMAND_BACKWARD, &doCreepBack, back },
        { COMMAND_CALIBRATE, &doCalibration, mute }, { COMMAND_DANCE, &doDance, dance }, { COMMAND_TWIST, &doTwist, twist }, {
        COMMAND_WAVE, &doWave, wave }, { COMMAND_TROT, &doTrot, trot }, { COMMAND_AUTO, &doAutoMove, autoMove }, { COMMAND_TEST,
                &doTest, test } };

const struct IRToCommandMapping IRMappingInstantCommands[] = { { COMMAND_FORWARD, &doSetDirectionForward, dirForward }, {
COMMAND_BACKWARD, &doSetDirectionBack, dirBack }, { COMMAND_RIGHT, &doSetDirectionRight, dirRight }, { COMMAND_LEFT,
        &doSetDirectionLeft, dirLeft }, { COMMAND_INCREASE_SPEED, &doIncreaseSpeed, volPlus }, { COMMAND_DECREASE_SPEED,
        &doDecreaseSpeed, volMinus }, { COMMAND_INCREASE_HEIGHT, &doIncreaseHeight, fastForward }, { COMMAND_DECREASE_HEIGHT,
        &doDecreaseHeight, fastBack }, { COMMAND_STOP, &doStop, stop } };
#else
// empty mapping
 const struct IRToCommandMapping IRMapping[] = {
 {COMMAND_RIGHT, &doTest, myMove }, { COMMAND_LEFT, &doTest, myMove }, { COMMAND_CENTER, &doCenterServos, center },
 { COMMAND_FORWARD, &doBeep, beep }, { COMMAND_BACKWARD, &doBeep, beep },
 {COMMAND_CALIBRATE, &doBeep, beep }, { COMMAND_DANCE, &doBeep, beep }, { COMMAND_TWIST, &doBeep, beep },
 {COMMAND_WAVE, &doBeep, beep }, { COMMAND_TROT, &doBeep, beep }, { COMMAND_AUTO, &doBeep, beep } };

 const struct IRToCommandMapping IRMappingInstantCommands[] = { { COMMAND_FORWARD, &doBeep, beep },
 {COMMAND_BACKWARD, &doBeep, beep }, { COMMAND_RIGHT, &doBeep, beep }, { COMMAND_LEFT, &doBeep, beep },
 { COMMAND_INCREASE_SPEED, &doBeep, beep }, { COMMAND_DECREASE_SPEED, &doBeep, beep },
 { COMMAND_INCREASE_HEIGHT, &doBeep, beep }, { COMMAND_DECREASE_HEIGHT, &doBeep, beep }, { COMMAND_STOP, &doBeep, beep } };
#endif // EMPTY_MAPPING
#endif /* IR_COMMAND_MAPING_H_ */

#pragma once
