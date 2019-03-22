// robot.h
// CWR_SZ project

// Created by Song Junlin on 3/21/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
#ifndef _ROBOT_H_
#define _ROBOT_H_

/*typedef struct {
  void (*RobotInit)(void);
} RobotInit;
typedef void (*RobotEnable)(void);*/
typedef struct _robot {
  /*functions*/
  RobotInit init;
} robot;

robot* new_robot(void);
void Robot_Init(void);