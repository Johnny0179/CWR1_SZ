#include "FreeModbus.h"
#include "PID.h"
#include "motor.h"
#include "timer.h"

extern int32_t usRegHoldingBuf[REG_HOLDING_NREGS];
u32 delta_turn[MotorNum];
u32 MotorSpeed[MotorNum];
/*ms*/
const u8 kTimeInteval = 50;
const float kPI = 3.14;
// diameter
const u8 kDiameter = 45;
// reduction ratio
const u8 kReductionRatio = 43;
/*motor turns*/
static volatile u32 motor_turn_this_time[MotorNum] = {0};
static volatile u32 motor_turn_last_time[MotorNum] = {0};
/*motor dir*/
static volatile u8 motor_dir_last_time = 0;
static volatile u8 motor_dir_this_time = 0;

// default up
static volatile _Bool dir_last_time = 1;
static volatile _Bool dir_this_time;

// odometer
u32 odometer[MotorNum] = {0};

void MotorInit(void) {
  // Freq=20k,DutyRatio=50
  TIM1_PWM_Init(PWMfreq);
  TIM8_PWM_Init(PWMfreq);

  Motor_Init();

  // Capture Init

  MotorFGInit();

  /*timer counter init*/
  Tim5IntInit(kTimeInteval * 10 - 1, 8400 - 1);
}

void MotorCtrlManual(struct MOTOR_DATA *motor, struct PID_DATA *pid,
                     u32 cmd_speed, _Bool dir) {
  motor->direction = dir;

  dir_this_time = dir;

  // motor speed feedback
  motor->MotorSpeed_mmps = MotorVelCalc(delta_turn[motor->num - 1]);

  if (dir_last_time == dir_this_time) {
    if (abs(cmd_speed - motor->MotorSpeed_mmps) > 100) {
      // stop
      if (cmd_speed == 0) {
        motor->CmdSpeed = abs(cmd_speed - motor->MotorSpeed_mmps) * 0.8;
      } else {
        motor->CmdSpeed = 0.8 * cmd_speed;
      }

    } else {
      motor->CmdSpeed = cmd_speed;
    }
  }
  // change direction
  else {
    // check the motors have been stopped
    if (motor->MotorSpeed_mmps != 0) {
      motor->CmdSpeed = 0;
    } else {
      if (abs(cmd_speed - motor->MotorSpeed_mmps) > 100) {
        // stop
        if (cmd_speed == 0) {
          motor->CmdSpeed = abs(cmd_speed - motor->MotorSpeed_mmps) * 0.8;
        } else {
          motor->CmdSpeed = 0.8 * cmd_speed;
        }

      } else {
        motor->CmdSpeed = cmd_speed;
      }
    }
  }

  // update dirction
  dir_last_time = dir;
  // if (dir == 1) {
  //   // move up
  //   dir_this_time = 1;
  // }

  // if (dir == 0) {
  //   // move down
  //   dir_this_time = -1;
  // }

  // motor->CmdSpeed = cmd_speed;

  // if (motor->MotorSpeed_mmps < 0.5 * cmd_speed) {
  //   motor->CmdSpeed = 0.5 * cmd_speed;
  // } else {
  //   motor->CmdSpeed = cmd_speed;
  // }

  motor->PWM = motor->PWM +
               pid_Controller(motor->CmdSpeed, motor->MotorSpeed_mmps, pid) / 2;

  // PWM 0~100%
  if (motor->PWM < 0) {
    motor->PWM = 0;
  } else if (motor->PWM > 100) {
    // Max motor->PWM Ratio
    motor->PWM = 100;
  }

  /*mannual mode*/
  if (motor->mode == 0) {
    // Direction
    switch (motor->direction) {
      case moveup:
        MoveUp();
        break;
      case movedown:
        MoveDown();
        break;
      case stop:;
      default:;
    }
  }

  // Speed
  switch (motor->num) {
    case 1:
      TIM1->CCR4 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
      break;
    case 2:
      TIM1->CCR3 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
      break;
    case 3:
      TIM1->CCR2 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
      break;
    case 4:
      TIM1->CCR1 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
      break;
    case 5:
      TIM8->CCR1 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
      break;
    case 6:
      TIM8->CCR2 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
      break;
    default:
      break;
  }
}

void MotorCtrlAuto(struct MOTOR_DATA *motor, struct PID_DATA *pid) {}

void MotorInitConfig(u8 num, struct MOTOR_DATA *motor) {
  motor->num = num;
  // up
  motor->direction = 0;
  motor->CmdSpeed = 0;
  motor->PWM = 0;
  // manual
  motor->mode = 0;
  motor->MotorSpeed_mmps = 0;
}

u32 MotorVelCalc(u32 delta_turn) {
  // 6 pulses per turn
  return (delta_turn * kPI * kDiameter * 1000) /
         (6 * kReductionRatio * kTimeInteval);
}

u32 DeltaTurnCalc(u32 *motor_turn, u8 motor_num) {
  u8 i = 0;

  for (i = 0; i <= motor_num - 1; i++) {
    // protect full
    if (motor_turn[i] < 0x0AFF) {
      motor_turn_this_time[i] = motor_turn[i];
      delta_turn[i] = motor_turn_this_time[i] - motor_turn_last_time[i];
      // update
      motor_turn_last_time[i] = motor_turn_this_time[i];
    } else {
      // odometer
      odometer[i] = odometer[i] + (motor_turn[i] * kPI * kDiameter) /
                                      (6 * kReductionRatio * 100);
      motor_turn_this_time[i] = motor_turn[i];
      delta_turn[i] = motor_turn_this_time[i] - motor_turn_last_time[i];
      // update, reset the counter
      motor_turn_last_time[i] = 0;
      // empty the turn counter
      motor_turn[i] = 0;
    }
  }
  usRegHoldingBuf[9] = odometer[0];
  usRegHoldingBuf[10] = motor_turn[0] / 6;
}