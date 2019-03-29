// robot.h
// CWR_SZ project

// Created by Song Junlin on 3/21/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "FreeModbus.h"
#include "PID.h"
#include "delay.h"
#include "led.h"
#include "motor.h"
#include "timer.h"

void RobotInit(void);
// function pointer
typedef void (*robot_init)(void);

void RobotEnable(void);
// function pointer
typedef void (*robot_enable)(void);

void RobotDisable(void);
// function pointer
typedef void (*robot_disable)(void);

void RobotManual(u32 cmd_speed,int8_t dir);
// function pointer
typedef void (*robot_manual)(u32,int8_t);

void RobotAuto(void);
// function pointer
typedef void (*robot_auto)(void);

// robot class definition
typedef struct {
  /*data members*/
  u8 no_;
  _Bool mode_;
  int8_t dir_;
  u32 odometer_;
  u32 cmd_speed_;
  /*functions*/
  robot_init Init;
  robot_enable Enable;
  robot_disable Disable;
  robot_manual Manual;
  robot_auto Auto;
} robot;

// constructor
void RobotNew(robot *r);

#endif