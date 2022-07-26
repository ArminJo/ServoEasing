/*
 * IRCommandMapping.h
 *
 * IR remote button codes, strings, and functions to call for quadruped IR control
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 * Mapping for controlling a mePed Robot V2 with 8 servos using an IR Remote at pin A0
 * Supported IR remote are KEYES (the original mePed remote) and WM10 and ...
 * Select the one you have below or define it in the including file.
 */

#ifndef _IR_COMMAND_MAPPING_H
#define _IR_COMMAND_MAPPING_H

#include <Arduino.h>

#include "IRCommandDispatcher.h" // IR_COMMAND_FLAG_BLOCKING etc. are defined here
#include "QuadrupedControlCommands.h" // contains the command definitions used in the mapping table below
#include "QuadrupedHelper.h"    // for additional commands
#if defined(QUADRUPED_HAS_NEOPIXEL)
#include "QuadrupedNeoPixel.h"  // for additional commands
#endif

/*
 * !!! Choose your remote !!!
 */
//#define USE_KEYES_REMOTE_CLONE // With number pad above direction control, will be taken as default
//#define USE_KEYES_REMOTE       // The mePed 2 Standard remote with number pad below direction control. Another name printed on the remote is Lafvin
//#define USE_WM10_REMOTE
//#define USE_WHITE_DVD_REMOTE
//#define USE_DVBT_STICK_REMOTE
#if !defined(USE_KEYES_REMOTE) && !defined(USE_WM10_REMOTE) && !defined(USE_KEYES_REMOTE_CLONE) \
    && !defined(USE_WHITE_DVD_REMOTE) && !defined(USE_DVBT_STICK_REMOTE)
#define USE_KEYES_REMOTE_CLONE // The one you can buy at aliexpress
#endif

#if defined(USE_KEYES_REMOTE_CLONE)
#define IR_REMOTE_NAME "KEYES_CLONE"
// Codes for the KEYES CLONE remote control with 17 keys with keypad above direction control
#define IR_ADDRESS 0x00

#define IR_UP    0x18
#define IR_DOWN  0x52
#define IR_RIGHT 0x5A
#define IR_LEFT  0x08
#define IR_OK    0x1C

#define IR_1    0x45
#define IR_2    0x46
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
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_OK
#define COMMAND_STOP        IR_HASH
#define COMMAND_CALIBRATE   IR_0
#define COMMAND_DANCE       IR_1
#define COMMAND_WAVE        IR_3
#define COMMAND_TWIST       IR_7
#define COMMAND_TROT        IR_9
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
#endif // defined(USE_KEYES_REMOTE_CLONE)

#if defined(USE_KEYES_REMOTE)
#  if defined(IR_REMOTE_NAME)
#error "Please choose only one remote for compile"
#  else
#define IR_REMOTE_NAME "KEYES"
/*
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the KEYES remote control with 17 keys
#define IR_ADDRESS 0x00

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
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_OK
#define COMMAND_STOP        IR_HASH
#define COMMAND_CALIBRATE   IR_0
#define COMMAND_DANCE       IR_1
#define COMMAND_WAVE        IR_3
#define COMMAND_TWIST       IR_7
#define COMMAND_TROT        IR_9
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

#  endif // defined(IR_REMOTE_NAME)
#endif // defined(USE_KEYES_REMOTE)

#if defined(USE_WM10_REMOTE)
#  if defined(IR_REMOTE_NAME)
#error "Please choose only one remote for compile"
#  else
#define IR_REMOTE_NAME "WM10"
/*
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the WM010 remote control with 14 keys
#define IR_ADDRESS 0x08

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

#  endif // defined(IR_REMOTE_NAME)
#endif // defined(USE_WM10_REMOTE)

#if defined(USE_DVBT_STICK_REMOTE)
#  if defined(IR_REMOTE_NAME)
#error "Please choose only one remote for compile"
#  else
#define IR_REMOTE_NAME "DVB-T"
/*
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the silver China DVB-T Stick remote control with 3x7 (21) keys
#define IR_ADDRESS 0x00

#define IR_ON_OFF 0x4D
#define IR_SOURCE 0x54
#define IR_MUTE   0x16

#define IR_RECORD     0x4C
#define IR_TIMESHIFT  0x0C

#define IR_CH_PLUS    0x05
#define IR_CH_MINUS   0x02

#define IR_VOL_MINUS  0xA
#define IR_VOL_PLUS   0x1E

#define IR_FULLSCREEN 0x40
#define IR_RECALL     0x1C

#define IR_0    0x12
#define IR_1    0x09
#define IR_2    0x1D
#define IR_3    0x1F
#define IR_4    0x0D
#define IR_5    0x19
#define IR_6    0x1B
#define IR_7    0x11
#define IR_8    0x15
#define IR_9    0x17

/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
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

#  endif // defined(IR_REMOTE_NAME)
#endif // defined(USE_DVBT_STICK_REMOTE)

#if defined(USE_WHITE_DVD_REMOTE)
#  if defined(IR_REMOTE_NAME)
#error "Please choose only one remote for compile"
#  else
#define IR_REMOTE_NAME "WHITE_DVD"
#define HAS_ADDITIONAL_REMOTE_COMMANDS

// Codes for white remote control for an old DVD Player
#define IR_ADDRESS 0x7B80

#define IR_ON_OFF 0x13

#define IR_1    0x01
#define IR_2    0x02
#define IR_3    0x03
#define IR_4    0x04
#define IR_5    0x05
#define IR_6    0x06
#define IR_7    0x07
#define IR_8    0x08
#define IR_9    0x09
#define IR_0    0x00

#define IR_CH_PLUS      0x0A
#define IR_CH_MINUS     0x0B

#define IR_REC          0x15
#define IR_PAUSE        0x1A

#define IR_UP           0x16 // Play
#define IR_DOWN         0x17 // Stop
#define IR_RIGHT        0x18 // FastForward
#define IR_LEFT         0x19 // Fast Backward

#define IR_ENTER        0x45
#define IR_INDEX        0x14
#define IR_CANCEL       0x4A
#define IR_MENU         0x50

// Lower small keys
#define IR_1_LOWER      0x0D // Timer_REC
#define IR_2_LOWER      0x1D
#define IR_3_LOWER      0x5F // Call
#define IR_4_LOWER      0x51
#define IR_5_LOWER      0x4C
#define IR_6_LOWER      0x4B
#define IR_7_LOWER      0x1E
#define IR_8_LOWER      0x12
#define IR_9_LOWER      0x0E // Audio Select
#define IR_EJECT        0x4E
/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT

#define COMMAND_CENTER      IR_ENTER
#define COMMAND_STOP        IR_ON_OFF
#define COMMAND_PAUSE_RESUME IR_PAUSE
#define COMMAND_CALIBRATE   IR_REC
#define COMMAND_DANCE       IR_1
#define COMMAND_WAVE        IR_3
#define COMMAND_TWIST       IR_7
#define COMMAND_TROT        IR_9
#define COMMAND_AUTO        IR_5
#define COMMAND_TEST        IR_MENU

#define COMMAND_INCREASE_SPEED  IR_6
#define COMMAND_DECREASE_SPEED  IR_4
#define COMMAND_INCREASE_HEIGHT IR_2
#define COMMAND_DECREASE_HEIGHT IR_8

// locally for doCalibration
#define COMMAND_ENTER       IR_ENTER
#define COMMAND_UP          IR_UP
#define COMMAND_DOWN        IR_DOWN

#define COMMAND_US_RIGHT    IR_CH_PLUS
#define COMMAND_US_SCAN     IR_CH_MINUS
#define COMMAND_US_LEFT     IR_0

#define COMMAND_PATTERN_1   IR_1_LOWER
#define COMMAND_PATTERN_2   IR_2_LOWER
#define COMMAND_PATTERN_3   IR_3_LOWER

#define COMMAND_PATTERN_4   IR_4_LOWER
#define COMMAND_PATTERN_5   IR_5_LOWER
#define COMMAND_PATTERN_6   IR_6_LOWER

#define COMMAND_PATTERN_7   IR_7_LOWER
#define COMMAND_PATTERN_8   IR_8_LOWER
#define COMMAND_PATTERN_9   IR_9_LOWER

#define COMMAND_PATTERN_0   IR_EJECT

#  endif // defined(IR_REMOTE_NAME)
#endif // defined(USE_WHITE_DVD_REMOTE)

/*
 * THIRD:
 * Main mapping of commands to C functions
 */

// IR strings of functions for output in alphabetical order
static const char autoMove[] PROGMEM ="auto move";
static const char back[] PROGMEM ="back";
static const char beep[] PROGMEM ="beep";
static const char calibration[] PROGMEM ="calibration";
static const char center[] PROGMEM ="center";
static const char dance[] PROGMEM ="dance";
static const char dirForward[] PROGMEM ="dir forward";
static const char dirBack[] PROGMEM ="dir back";
static const char dirRight[] PROGMEM ="dir right";
static const char dirLeft[] PROGMEM ="dir left";
static const char enter[] PROGMEM ="enter";
static const char forward[] PROGMEM ="forward";
static const char heighIncrease[] PROGMEM ="increase height";
static const char heighDecrease[] PROGMEM ="decrease height";
static const char left[] PROGMEM ="left";
static const char myMove[] PROGMEM ="my move";
static const char melody[] PROGMEM ="Melody";
static const char pattern[] PROGMEM ="NeoPattern";
static const char right[] PROGMEM ="right";
static const char speedIncrease[] PROGMEM ="increase speed";
static const char speedDecrease[] PROGMEM ="decrease speed";
static const char stop[] PROGMEM ="stop";
static const char test[] PROGMEM ="test";
//static const char onOff[] PROGMEM ="on/off";
static const char trot[] PROGMEM ="trot";
static const char twist[] PROGMEM ="twist";
static const char ultrasonicServoLeft[] PROGMEM ="US servo left";
static const char ultrasonicServoRight[] PROGMEM ="US servo right";
static const char ultrasonicServoScan[] PROGMEM ="US servo scan";
static const char unknown[] PROGMEM ="unknown";
static const char wave[] PROGMEM ="wave";
static const char pauseResume[] PROGMEM ="pause/resume";

/*
 * Main mapping array of commands to C functions and command strings
 */
const struct IRToCommandMappingStruct IRMapping[] = { {
/*
 * Commands, which must run exclusively and therefore must first stop other commands running.
 */
COMMAND_DANCE, IR_COMMAND_FLAG_BLOCKING, &doDance, dance }, {
COMMAND_TWIST, IR_COMMAND_FLAG_BLOCKING, &doTwist, twist }, {
COMMAND_WAVE, IR_COMMAND_FLAG_BLOCKING, &doWave, wave }, {
COMMAND_TROT, IR_COMMAND_FLAG_BLOCKING, &doTrot, trot }, {
COMMAND_AUTO, IR_COMMAND_FLAG_BLOCKING, &doQuadrupedAutoMove, autoMove }, {
COMMAND_TEST, IR_COMMAND_FLAG_BLOCKING, &doTest, test }, {
#if defined(QUADRUPED_HAS_IR_CONTROL) && !defined(USE_USER_DEFINED_MOVEMENTS)
        COMMAND_CALIBRATE, IR_COMMAND_FLAG_BLOCKING, &doCalibration, calibration}, {
#endif
        COMMAND_CENTER, IR_COMMAND_FLAG_BLOCKING, &doCenterServos, center }, {
/*
 * Set direction also starts the movement
 */
COMMAND_FORWARD, IR_COMMAND_FLAG_BLOCKING, &doSetDirectionForward, dirForward }, {
COMMAND_BACKWARD, IR_COMMAND_FLAG_BLOCKING, &doSetDirectionBack, dirBack }, {
COMMAND_RIGHT, IR_COMMAND_FLAG_BLOCKING, &doSetDirectionRight, dirRight }, {
COMMAND_LEFT, IR_COMMAND_FLAG_BLOCKING, &doSetDirectionLeft, dirLeft }, {
/*
 * Commands, which can be executed always, since they are short and repeats are allowed
 */
COMMAND_INCREASE_SPEED, IR_COMMAND_FLAG_REPEATABLE_NON_BLOCKING, &doIncreaseSpeed, speedIncrease }, {
COMMAND_DECREASE_SPEED, IR_COMMAND_FLAG_REPEATABLE_NON_BLOCKING, &doDecreaseSpeed, speedDecrease }, {
COMMAND_INCREASE_HEIGHT, IR_COMMAND_FLAG_REPEATABLE_NON_BLOCKING, &doIncreaseHeight, heighIncrease }, {
COMMAND_DECREASE_HEIGHT, IR_COMMAND_FLAG_REPEATABLE_NON_BLOCKING, &doDecreaseHeight, heighDecrease }, {
#if defined(HAS_ADDITIONAL_REMOTE_COMMANDS)
COMMAND_STOP, IR_COMMAND_FLAG_IS_STOP_COMMAND, &doStop, stop }
#else
COMMAND_STOP, IR_COMMAND_FLAG_NON_BLOCKING, &doPauseResume, pauseResume }
#endif

#if defined(HAS_ADDITIONAL_REMOTE_COMMANDS)
        /*
         * Commands not accessible by simple remote because of lack of keys
         */
#if defined(QUADRUPED_HAS_US_DISTANCE_SERVO)
        , { COMMAND_US_RIGHT, IR_COMMAND_FLAG_REPEATABLE_NON_BLOCKING, &doUSRight, ultrasonicServoRight }, {
        COMMAND_US_LEFT, IR_COMMAND_FLAG_REPEATABLE_NON_BLOCKING, &doUSLeft, ultrasonicServoLeft }, {
        COMMAND_US_SCAN, IR_COMMAND_FLAG_NON_BLOCKING, &doUSScan, ultrasonicServoScan }
#endif
        , {
        COMMAND_PAUSE_RESUME, IR_COMMAND_FLAG_NON_BLOCKING, &doPauseResume, pauseResume }, {
        COMMAND_PATTERN_1, IR_COMMAND_FLAG_NON_BLOCKING, &doPattern1, pattern }, {
        COMMAND_PATTERN_2, IR_COMMAND_FLAG_NON_BLOCKING, &doPattern2, pattern }, {
        COMMAND_PATTERN_3, IR_COMMAND_FLAG_NON_BLOCKING, &doPatternStripes, pattern }, {
        COMMAND_PATTERN_4, IR_COMMAND_FLAG_NON_BLOCKING, &doPatternHeartbeat, pattern }, {
        COMMAND_PATTERN_5, IR_COMMAND_FLAG_NON_BLOCKING, &doPatternFire, pattern }, {
        COMMAND_PATTERN_6, IR_COMMAND_FLAG_NON_BLOCKING, &doWipeOutPatterns, pattern }, {
        COMMAND_PATTERN_0, IR_COMMAND_FLAG_NON_BLOCKING, &doRandomMelody, melody }

#endif
        };

#endif // _IR_COMMAND_MAPPING_H
