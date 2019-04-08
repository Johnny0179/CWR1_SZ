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

    // defualt auto mode
    usRegHoldingBuf[21]=1;
    // defualt 6 cycles
    usRegHoldingBuf[22]=6;
    // defualt speed 200
    usRegHoldingBuf[23]=200;
    // disable reset
    usRegHoldingBuf[9]=0;
}

static void RobotEnable(void) { MotorEnable(); }

static void RobotDisable(void) { MotorDisable(); }
static void RobotReset(void) { MotorReset(); }

static void RobotManual(u32 cmd_speed, int8_t dir) {
  u8 i;

	//move down
/*	if(dir==1){
		for(i=0;i<MotorNum;++i){
	pid_Init(2*P,1*I,0*D,&PIDMotor[i]);}
	}*/



  /*motor control*/
  for (i = 0; i < MotorNum; ++i) {
    /* code */
    MotorCtrlManual(&Motor[i], &PIDMotor[i], &cmd_speed, dir);
  }

  /*monitor speed*/
  for (i = 0; i < MotorNum; ++i) {
    /* code */
    usRegHoldingBuf[i + 24] = Motor[i].MotorSpeed_mmps;
  }

  /*Debug*/
  usRegHoldingBuf[10] = Motor[0].PWM;
  usRegHoldingBuf[11] = Motor[1].PWM;
  usRegHoldingBuf[12] = Motor[2].PWM;
  usRegHoldingBuf[13] = Motor[3].PWM;
  usRegHoldingBuf[14] = Motor[4].PWM;
  usRegHoldingBuf[15] = Motor[5].PWM;
}

static u8 RobotAuto(u32 cmd_speed, _Bool init_dir, u8 cycle) {
  u8 i;
  u8 state=0;
  /*motor control*/
  for (i = 0; i < MotorNum; ++i) {
    state=MotorCtrlAuto(&Motor[i], &PIDMotor[i], &cmd_speed, init_dir, cycle);
  }

  /*monitor speed*/
  for (i = 0; i < MotorNum; ++i) {
    /* code */
    usRegHoldingBuf[i + 24] = Motor[i].MotorSpeed_mmps;
  }
  /*Debug*/
  usRegHoldingBuf[10] = Motor[0].PWM;
  usRegHoldingBuf[11] = Motor[1].PWM;
  usRegHoldingBuf[12] = Motor[2].PWM;
  usRegHoldingBuf[13] = Motor[3].PWM;
  usRegHoldingBuf[14] = Motor[4].PWM;
  usRegHoldingBuf[15] = Motor[5].PWM;
  return state;
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
  r->Reset=RobotReset;
}

