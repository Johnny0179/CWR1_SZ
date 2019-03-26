// robot.h
// CWR_SZ project

// Created by Song Junlin on 3/21/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
#ifndef _ROBOT_H_
#define _ROBOT_H_

// typedef struct {
//   void (*RobotInit)(void);
// } RobotInit;

void RobotInit(void);
// function pointer
typedef void (*robot_init)(void);

// robot class definition
typedef struct {
  u8 no_;
  /*functions*/
  robot_init init;
} robot;

// constructor
void robot_new(robot *r);

#endif