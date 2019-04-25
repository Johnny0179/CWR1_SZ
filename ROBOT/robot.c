// robot.c
// CWR_SZ project

// Created by Song Junlin on 3/21/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
#include "robot.h"
#include "stmflash.h"

extern uint32_t usRegHoldingBuf[REG_HOLDING_NREGS];
extern const UCHAR kRobotAddr;

// const parameters
static const _Bool kManualMode = 0;
static const _Bool kAutoMode = 1;

static const kManualEnable = 1;
static const kManualDisable = 0;

const int8_t kDirUp = 0;
const int8_t kDirDown = 1;

// record the step
u32 flash_step;

static const u32 flashe_clear = 0;

// Motor Define
static MOTOR Motor[MotorNum];
static pidData_t PIDMotor[MotorNum];

static void RobotInit(void)
{
  u8 i;

  //初始化延时函数
  delay_init(168);
  LED_Init();

  // Motor Init
  MotorInit();

  for (i = 0; i < MotorNum; i++)
  {
    MotorInitConfig(i + 1, &Motor[i]);
  }

  // PID Init
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[0]);
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[1]);
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[2]);
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[3]);
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[4]);
  pid_Init(1 * P, 1 * I, 0 * D, &PIDMotor[5]);

  // light up led
  LED2 = 1;

#if 1
  // clear flash
  STMFLASH_Write(0x080E0000, (u32 *)(&flashe_clear), 1);
#endif

  // read from flash
  STMFLASH_Read(0x080E0000, (u32 *)(&flash_step), 1);

  usRegHoldingBuf[1] = flash_step;

  // robot address
  usRegHoldingBuf[0] = kRobotAddr;
  // communication indicator flag
  usRegHoldingBuf[6] = 1;
  // defualt auto mode
  usRegHoldingBuf[21] = 0;
  // defualt 10 cycles
  usRegHoldingBuf[22] = 10;
  // defualt speed 150
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

static void RobotManual(u32 cmd_speed, int8_t dir)
{
  u8 i;
  /*motor control*/
  for (i = 0; i < MotorNum; ++i)
  {
    /* code */
    MotorCtrlManual(&Motor[i], &PIDMotor[i], &cmd_speed, dir);
  }

  /*monitor speed*/
  for (i = 0; i < MotorNum; ++i)
  {
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
const u8 kWait = 6;
const u8 kMotorStall = 7;
const u8 kManual = 8;
const u8 kAuto = 9;

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
const u32 fine_tuning_speed = 100;

volatile _Bool auto_dir;

static u8 RobotAuto(u32 cmd_speed, _Bool init_dir, u8 cycle, u8 *state)
{
  u8 i, motor_state[MotorNum];
  u32 motor_speed[MotorNum], odom_temp;

  for (i = 0; i < MotorNum; ++i)
  {
    motor_speed[i] = MotorVelCalc(delta_turn[i]);
  }

  cycle_odometer_this_time = (odometer[0] + odometer[1] + odometer[2] +
                              odometer[3] + odometer[4] + odometer[5]) /
                             MotorNum;

  usRegHoldingBuf[38] = cycle_odometer_this_time;
  usRegHoldingBuf[39] = cycle_odometer_last_time;

  /*indicate the dir*/
  // up
  if (auto_dir == 0)
  {
    usRegHoldingBuf[8] = 1;
  }
  else if (auto_dir == 1)
  {
    // down
    usRegHoldingBuf[8] = 2;
  }

  // state machine
  switch (*state)
  {
  case kIdle:
    if (usRegHoldingBuf[21] == kManualDisable && usRegHoldingBuf[3] == 1)
    {
      // auto mode, defualt dir;
      auto_dir = init_dir;
      *state = kAuto;
    }
    else if (usRegHoldingBuf[21] == kManualEnable)
    {
      *state = kManual;
    }
    else if (usRegHoldingBuf[3] == 0 || cycle_counter == cycle)
    {
      /*        // clear the counter
                cycle_counter = 0;*/

      //
      usRegHoldingBuf[3] = 0;
      *state = kIdle;
    }
    break;

  case kAuto:
    if (usRegHoldingBuf[3] == 1)
    {
      // clear the counter
      cycle_counter = 0;
      // clear odom
      for (i = 0; i < MotorNum; ++i)
      {
        odometer[i] = 0;
      }
      cycle_odometer_last_time = 0;
      cycle_odometer_this_time = 0;

      *state = kCounterCheck;
    }
    else if (usRegHoldingBuf[21] == kManualEnable)
    {
      // back to manual mode
      *state = kManual;
    }
    else if (usRegHoldingBuf[3] == 0)
    {
      // stop
      for (i = 0; i < MotorNum; ++i)
      {
        motor_state[i] = MotorCtrlManual(&Motor[i], &PIDMotor[i], &stop_speed,
                                         usRegHoldingBuf[2]);
      }
      // wait cmd
      *state = kAuto;
    }
    break;

  case kManual:
    if (usRegHoldingBuf[21] == kManualEnable)
    {
      usRegHoldingBuf[3] = 1;
      // motor control
      for (i = 0; i < MotorNum; ++i)
      {
        motor_state[i] = MotorCtrlManual(
            &Motor[i], &PIDMotor[i], &fine_tuning_speed, usRegHoldingBuf[2]);
      }
      *state = kManual;
    }
    else if (usRegHoldingBuf[21] == kManualDisable)
    {
      usRegHoldingBuf[3] = 0;

      // rst
      // RobotReset();

      // // clear odom
      // for (i = 0; i < MotorNum; ++i) {
      //   odometer[i] = 0;
      // }
      // cycle_odometer_last_time = 0;
      // cycle_odometer_this_time = 0;

      *state = kAuto;
    }
    break;

  case kCounterCheck:
    if (cycle_counter < cycle)
    {
      // up
      if (auto_dir == 0)
      {
        dis_up_start = cycle_odometer_this_time;
        usRegHoldingBuf[40] = dis_up_start;
      }
      else if (auto_dir == 1)
      {
        // down
        dis_down_start = cycle_odometer_this_time;
        usRegHoldingBuf[41] = dis_down_start;
      }
      *state = kMotion;
    }
    else if (cycle_counter == cycle)
    {
      // disable
      usRegHoldingBuf[3] = 0;
      *state = kIdle;
    }
    break;

  case kFirstCheck:
    if (cycle_counter == 0)
    {
      auto_dir = init_dir;
      *state = kMotion;
    }
    else
    {
      *state = kChangeDir;
    }
    break;

  case kChangeDir:
    // change direction
    auto_dir = !auto_dir;

    //
    usRegHoldingBuf[2] = auto_dir;

    *state = kWait;
    break;

  case kMotion:
    /*offset correction*/
    // complete 2 step?
    if (cycle_counter < 2)
    {
      // up?
      if (auto_dir == 0)
      {
        // dis up > dis down
        if (dis_up > dis_down)
        {
          // upward correction
          offset_correct = dis_up - dis_down;
        }
        else
        {
          // no correction
          offset_correct = 0;
        }
      }
      else
      {
        // down
        if (dis_down > dis_up)
        {
          offset_correct = dis_down - dis_up;
        }
        else
        {
          // no correction
          offset_correct = 0;
        }
      }
    }
    else
    {
      // no correction
      offset_correct = 0;
    }

    if ((cycle_odometer_this_time - cycle_odometer_last_time) <=
        usRegHoldingBuf[17] - offset_correct)
    {
      // motor control
      for (i = 0; i < MotorNum; ++i)
      {
        motor_state[i] =
            MotorCtrlManual(&Motor[i], &PIDMotor[i], &cmd_speed, auto_dir);
      }

      *state = kMotion;
    }
    else
    {
      *state = kStop;
    }

    break;

  case kStop:
    // all motors have been stopped
    if (motor_speed[0] != 0 || motor_speed[1] != 0 || motor_speed[2] != 0 ||
        motor_speed[3] != 0 || motor_speed[4] != 0 || motor_speed[5] != 0)
    {
      // stop
      for (i = 0; i < MotorNum; ++i)
      {
        motor_state[i] =
            MotorCtrlManual(&Motor[i], &PIDMotor[i], &stop_speed, auto_dir);
      }
      *state = kStop;
    }
    else
    {
      // update the odometer
      cycle_odometer_last_time = (odometer[0] + odometer[1] + odometer[2] +
                                  odometer[3] + odometer[4] + odometer[5]) /
                                 MotorNum;

      // up
      if (auto_dir == 0)
      {
        dis_up_end = cycle_odometer_this_time;
        usRegHoldingBuf[42] = dis_up_end;
      }
      else if (auto_dir == 1)
      {
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

      // read from flash
      STMFLASH_Read(0x080E0000, (u32 *)(&flash_step), 1);

      flash_step++;

      // write to flash
      STMFLASH_Write(0x080E0000, (u32 *)(&flash_step), 1);

      // read from flash
      // STMFLASH_Read(0x080E0000, (u32 *)(&flash_step), 1);

      usRegHoldingBuf[1] = flash_step;

      *state = kChangeDir;
    }
    break;

  case kWait:
    if (usRegHoldingBuf[3] == 0)
    {
      *state = kWait;
    }
    else
    {
      *state = kCounterCheck;
    }
    break;

  default:
    *state = kIdle;
    break;
  }

  /*monitor speed*/
  for (i = 0; i < MotorNum; ++i)
  {
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

  // motor stall
  if (motor_state[0] == 1 || motor_state[1] == 1 || motor_state[2] == 1 ||
      motor_state[3] == 1 || motor_state[4] == 1 || motor_state[5] == 1)
  {
    *state = kMotorStall;
  }

  return *state;
}

void RobotAlarmEnable(void) { Tim13Enable(); }

void RobotAlarmDisable(void) { Tim13Disable(); }

void RobotNew(robot *r)
{
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
