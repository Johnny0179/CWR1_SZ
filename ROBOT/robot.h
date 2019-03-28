// robot.h
// CWR_SZ project

// Created by Song Junlin on 3/21/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
#ifndef _ROBOT_H_
#define _ROBOT_H_
#include "motor.h"

void RobotInit(void);
// function pointer
typedef void (*robot_init)(void);

void RobotEnable(void);
// function pointer
typedef void (*robot_enable)(void);

void RobotDisable(void);
// function pointer
typedef void (*robot_disable)(void);

void RobotControl(u16 motion_cycle);
// function pointer
typedef void (*robot_control)(u16);

// robot class definition
typedef struct {
  /*data members*/
  u8 no_;
  /*functions*/
  robot_init Init;
  robot_enable Enable;
  robot_disable Disable;
  robot_control Control;
} robot;

// constructor
void RobotNew(robot *r);

#endif