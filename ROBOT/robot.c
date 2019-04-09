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
  usRegHoldingBuf[21] = 1;
  // defualt 6 cycles
  usRegHoldingBuf[22] = 6;
  // defualt speed 200
  usRegHoldingBuf[23] = 200;
  // disable reset
  usRegHoldingBuf[9] = 0;
}

static void RobotEnable(void) { MotorEnable(); }

static void RobotDisable(void) { MotorDisable(); }
static void RobotReset(void) { MotorReset(); }

static void RobotManual(u32 cmd_speed, int8_t dir) {
  u8 i;
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

/*-----------------------auto mode state machine--------*/
const u8 kIdle = 0;
const u8 kCounterCheck = 1;
const u8 kChangeDir = 2;
const u8 kFirstCheck = 3;
const u8 kMotion = 4;
const u8 kStop = 5;
const u8 kDone = 6;

/*----------------------variables----------------------*/
extern u32 odometer[MotorNum];
extern u32 delta_turn[MotorNum];
u32 cycle_counter = 0;
u32 cycle_odometer_last_time = 0;
u32 cycle_odometer_this_time = 0;

const u32 stop_speed = 0;

static volatile _Bool auto_dir;

static u8 RobotAuto(u32 cmd_speed, _Bool init_dir, u8 cycle, u8 *state) {
  u8 i;
  u32 motor_speed[MotorNum];
  for (i = 0; i < MotorNum; ++i) {
    motor_speed[i] = MotorVelCalc(delta_turn[i]);
  }

  cycle_odometer_this_time = odometer[0];

  // debug
  usRegHoldingBuf[1] = cycle_counter;
  usRegHoldingBuf[38] = cycle_odometer_this_time;
  usRegHoldingBuf[39] = cycle_odometer_last_time;

  /*indicate the dir*/
  // up
  if (auto_dir == 0) {
    usRegHoldingBuf[8] = 2;
  } else if (auto_dir == 1) {
    // down
    usRegHoldingBuf[8] = 1;
  }

  // state machine
  switch (*state) {
    case kIdle:
      if (usRegHoldingBuf[3] == 0 || cycle_counter == cycle) {
        // clear the counter
        cycle_counter = 0;

        //
        usRegHoldingBuf[3] = 0;

        *state = kIdle;
      } else if (usRegHoldingBuf[3] == 1) {
        *state = kCounterCheck;
      }
      break;

    case kCounterCheck:
      if (cycle_counter < cycle) {
        *state = kFirstCheck;
      } else if (cycle_counter == cycle) {
        *state = kIdle;
      }
      break;

    case kFirstCheck:
      if (cycle_counter == 0) {
        auto_dir = init_dir;
        *state = kMotion;
      } else {
        *state = kChangeDir;
      }
      break;

    case kChangeDir:
      // change direction
      auto_dir = !auto_dir;

      // update the odometer
      cycle_odometer_last_time = odometer[0];
      *state = kMotion;
      break;

    case kMotion:
      if ((cycle_odometer_this_time - cycle_odometer_last_time) <= 100) {
        /*motor control*/
        for (i = 0; i < MotorNum; ++i) {
          MotorCtrlManual(&Motor[i], &PIDMotor[i], &cmd_speed, auto_dir);
        }

        *state = kMotion;
      } else {
        *state = kStop;
      }
      break;

    case kStop:
      // all motors have been stopped
      if (motor_speed[0] != 0 || motor_speed[1] != 0 || motor_speed[2] != 0 ||
          motor_speed[3] != 0 || motor_speed[4] != 0 || motor_speed[5] != 0) {
        /*motor control*/
        for (i = 0; i < MotorNum; ++i) {
          MotorCtrlManual(&Motor[i], &PIDMotor[i], &stop_speed, auto_dir);
        }
        *state = kStop;
      } else {
        // update cycle counter
        cycle_counter++;
        *state = kCounterCheck;
      }
      break;

    case kDone:;

    default:
      *state = kIdle;
      break;
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
  return *state;
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
  r->Reset = RobotReset;
}
