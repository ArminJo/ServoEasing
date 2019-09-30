/*
 * RobotArmRTCControl.h
 *
 *  Created on: 01.10.2019
 *      Author: Armin
 */

#ifndef ROBOTARMRTCCONTROL_H_
#define ROBOTARMRTCCONTROL_H_

#include "uRTCLib.h"

extern uRTCLib RTC_DS3231;

void initRTC();
void testRTC();
void printRTC();

#endif /* ROBOTARMRTCCONTROL_H_ */

#pragma once
