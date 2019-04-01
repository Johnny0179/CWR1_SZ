// robot.c
// CWR_SZ project

// Created by Song Junlin on 3/21/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
#include "robot.h"

extern int32_t usRegHoldingBuf[REG_HOLDING_NREGS];

// const parameters
static const _Bool kManualMode = 0;
static const _Bool kAutoMode = 1;
const int8_t kDirUp = 0;
const int8_t kDirDown = 1;

// Motor Define
static MOTOR Motor[MotorNum];
static pidData_t PIDMotor[MotorNum];

static void RobotInit(void) {
  u8 i;
  // Motor Init
  MotorInit();
  for (i = 0; i < MotorNum; i++) {
    MotorInitConfig(i + 1, &Motor[i]);
  }

  // PID Init
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[0]);
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[1]);
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[2]);
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[3]);
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[4]);
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[5]);
}

static void RobotEnable(void) { MotorEnable(); }

static void RobotDisable(void) { MotorDisable(); }

static void RobotManual(u32 cmd_speed, int8_t dir) {
  u8 i;
  /*motor control*/
  for (i = 0; i < MotorNum; ++i) {
    /* code */
    MotorCtrlManual(&Motor[i], &PIDMotor[i], cmd_speed, dir);
  }

  /*monitor speed*/
  for (i = 0; i < MotorNum; ++i) {
    /* code */
    usRegHoldingBuf[i + 3] = Motor[i].MotorSpeed_mmps;
  }

  /*Debug*/
  usRegHoldingBuf[12] = Motor[0].PWM;
}

static void RobotAuto(u32 cmd_speed, _Bool init_dir, u8 cycle) {
  u8 i;
  /*motor control*/
  for (i = 0; i < MotorNum; ++i) {
    MotorCtrlAuto(&Motor[i], &PIDMotor[i], cmd_speed, init_dir, cycle);
  }

  /*monitor speed*/
  for (i = 0; i < MotorNum; ++i) {
    /* code */
    usRegHoldingBuf[i + 3] = Motor[i].MotorSpeed_mmps;
  }
  /*Debug*/
  usRegHoldingBuf[12] = Motor[0].PWM;
}

void RobotNew(robot *r) {
  // robot parameters
  r->no_ = 0;
  r->cycle_ = 0;
  r->mode_ = kManualMode;
  r->dir_ = kDirUp;
  r->odometer_ = 0;
  r->cmd_speed_ = 0;

  // functions
  r->Init = RobotInit;

  r->Enable = RobotEnable;
  r->Disable = RobotDisable;
  r->Manual = RobotManual;
  r->Auto = RobotAuto;
}