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
  usRegHoldingBuf[22] = 50;
  // defualt speed 200
  usRegHoldingBuf[23] = 150;
  // disable reset
  usRegHoldingBuf[9] = 0;
  // default direction up
  usRegHoldingBuf[2] = 0;

  // default step distance
  usRegHoldingBuf[17] = 50;
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
static const u8 kMotorStall = 7;

/*----------------------variables----------------------*/
extern u32 odometer[MotorNum];
extern u32 delta_turn[MotorNum];
u32 cycle_counter = 0;

u32 cycle_odometer_last_time = 0;
u32 cycle_odometer_this_time = 0;

u32 decel_odom = 0;

// distance variables
u32 dis_up_start = 0;
u32 dis_up_end = 0;
u32 dis_up = 0;

u32 dis_down_start = 0;
u32 dis_down_end = 0;
u32 dis_down = 0;

u32 offset_correct = 0;

const u32 stop_speed = 0;

static volatile _Bool auto_dir;

static u8 RobotAuto(u32 cmd_speed, _Bool init_dir, u8 cycle, u8 *state) {
  u8 i;
  u32 motor_speed[MotorNum], odom_temp;
  for (i = 0; i < MotorNum; ++i) {
    motor_speed[i] = MotorVelCalc(delta_turn[i]);
  }

  cycle_odometer_this_time = (odometer[0] + odometer[1] + odometer[2] +
                              odometer[3] + odometer[4] + odometer[5]) /
                             MotorNum;

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
        // defualt dir;
        auto_dir = init_dir;
        *state = kCounterCheck;
      }
      break;

    case kCounterCheck:
      if (cycle_counter < cycle) {
        // up
        if (auto_dir == 0) {
          dis_up_start = cycle_odometer_this_time;
          usRegHoldingBuf[40] = dis_up_start;
        } else if (auto_dir == 1) {
          // down
          dis_down_start = cycle_odometer_this_time;
          usRegHoldingBuf[41] = dis_down_start;
        }

        *state = kMotion;
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

      //
      usRegHoldingBuf[2] = auto_dir;

      *state = kCounterCheck;
      break;

    case kMotion:
      /*offset correction*/
      // complete 2 step?
      if (cycle_counter < 2) {
        // up?
        if (auto_dir == 0) {
          // dis up > dis down
          if (dis_up > dis_down) {
            // upward correction
            offset_correct = dis_up - dis_down;
          } else {
            // no correction
            offset_correct = 0;
          }
        } else {
          // down
          if (dis_down > dis_up) {
            offset_correct = dis_down - dis_up;
          } else {
            // no correction
            offset_correct = 0;
          }
        }
      } else {
        // no correction
        offset_correct = 0;
      }

      if ((cycle_odometer_this_time - cycle_odometer_last_time) <=
          usRegHoldingBuf[17] - offset_correct) {
        // motor control
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
        // update the odometer
        cycle_odometer_last_time = (odometer[0] + odometer[1] + odometer[2] +
                                    odometer[3] + odometer[4] + odometer[5]) /
                                   MotorNum;

        // up
        if (auto_dir == 0) {
          dis_up_end = cycle_odometer_this_time;
          usRegHoldingBuf[42] = dis_up_end;
        } else if (auto_dir == 1) {
          // down
          dis_down_end = cycle_odometer_this_time;
          usRegHoldingBuf[43] = dis_down_end;
        }

        // claculate the distance
        dis_up = dis_up_end - dis_up_start;
        dis_down = dis_down_end - dis_down_start;

        usRegHoldingBuf[18] = dis_up;
        usRegHoldingBuf[19] = dis_down;

        // update cycle counter
        cycle_counter++;
        *state = kChangeDir;
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

void RobotAlarmEnable(void) { Tim13Enable(); }

void RobotAlarmDisable(void) { Tim13Disable(); }

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
