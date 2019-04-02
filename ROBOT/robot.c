// robot.c
// CWR_SZ project

// Created by Song Junlin on 3/21/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
#include "robot.h"

extern int32_t usRegHoldingBuf[REG_HOLDING_NREGS];

// const parameters
const _Bool kManualMode = 0;
const _Bool kAutoMode = 1;
const int8_t kDirUp = 0;
const int8_t kDirDown = 1;

// robot motion dirction
const _Bool kMotionUp = 0;
const _Bool kMotionDown = 1;

// pid
const uint32_t kPUp = 300;
const uint32_t kIUp = 10;
const uint32_t kDUp = 0;

const uint32_t kPDown = 100;
const uint32_t kIDown = 10;
const uint32_t kDDown = 0;

// default up
static volatile _Bool dir_last_time = 0;
static volatile _Bool dir_this_time;

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
  _Bool motion_state;
  dir_this_time = dir;

  if (motion_state == kMotionUp) {
    // change direction from up to down
    if (dir_last_time == 0 && dir_this_time == 1) {
      // move down
      motion_state = kMotionDown;
      // set the motion up pid parameter
      for (i = 0; i < MotorNum; ++i) {
        pid_Init(kPUp, kIUp, kDUp, &PIDMotor[i]);
      }
    }
  }

  if (motion_state == kMotionDown) {
    // change direction from down to up
    if (dir_last_time == 1 && dir_this_time == 0) {
      // move up
      motion_state = kMotionUp;
      // set the motion down pid parameter
      for (i = 0; i < MotorNum; ++i) {
        pid_Init(0.1 * kPUp, 0.1 * kIUp, 0.1 * kDUp, &PIDMotor[i]);
      }
    }
  }

  // update dirction
  dir_last_time = dir;

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

static void RobotAuto(u32 cmd_speed, _Bool init_dir, u8 cycle,
                      u8 cycle_distance) {
  u8 i;
  /*motor control*/
  for (i = 0; i < MotorNum; ++i) {
    MotorCtrlAuto(&Motor[i], &PIDMotor[i], cmd_speed, init_dir, cycle,
                  cycle_distance);
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
  r->cycle_distance_ = 0;

  // functions
  r->Init = RobotInit;
  r->Enable = RobotEnable;
  r->Disable = RobotDisable;
  r->Manual = RobotManual;
  r->Auto = RobotAuto;
}