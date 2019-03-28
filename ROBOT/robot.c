// robot.c
// CWR_SZ project

// Created by Song Junlin on 3/21/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
#include "robot.h"
#include "delay.h"
#include "led.h"
#include "motor.h"

// motion cycle 8s
// static const u16 kMotionCycle = 8000;

static void RobotInit(void) { LED1 = 0; }

static void RobotEnable(void) { MotorEnable(); }

static void RobotDisable(void) { MotorDisable(); }

static void RobotControl(u16 motion_cycle) {
	
  MoveUp();
  delay_ms(motion_cycle * 1000);
  MotorDisable();
  MotorEnable();

  MoveDown();
  delay_ms(motion_cycle * 1000);
  MotorDisable();
  MotorEnable();
}

void RobotNew(robot *r) {
  r->no_ = 0;
  r->Init = RobotInit;
  r->Enable = RobotEnable;
  r->Disable = RobotDisable;
  r->Control = RobotControl;
}