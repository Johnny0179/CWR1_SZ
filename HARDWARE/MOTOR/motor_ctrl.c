#include "FreeModbus.h"
#include "PID.h"
#include "motor.h"
#include "timer.h"

extern int32_t usRegHoldingBuf[REG_HOLDING_NREGS];
u32 delta_turn[MotorNum];
u32 MotorSpeed[MotorNum];

/*constant prarameters*/

const _Bool kTrue = 1;
const _Bool kFalse = 0;

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
static volatile _Bool dir_last_time = 0;
static volatile _Bool dir_this_time;
// retain the direction when stop the motor
static volatile _Bool dir_stop;
static volatile _Bool auto_dir;

static volatile _Bool dir_change;
static volatile u32 stop_counter;
const u16 kStopTime = 0;
// odometer
u32 odometer[MotorNum] = {0};
u32 cycle_counter = 0;
u32 cycle_odometer_last_time = 0;
u32 cycle_odometer_this_time = 0;

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
  dir_this_time = dir;
  // change direction
  if (dir_last_time != dir_this_time) {
    dir_change = kTrue;
    dir_stop = dir_last_time;
  }
  // motor speed feedback
  motor->MotorSpeed_mmps = MotorVelCalc(delta_turn[motor->num - 1]);

  if (!dir_change) {
    motor->direction = dir;
    motor->CmdSpeed = MotorSetCmdSpeed(cmd_speed, motor->MotorSpeed_mmps);
  }
  if (dir_change) {
    motor->direction = dir_stop;
    // check the motors have been stopped
    if (motor->MotorSpeed_mmps != 0) {
      motor->CmdSpeed = MotorSetCmdSpeed(0, motor->MotorSpeed_mmps);
    } else {
      // motors have been stopped
      if (stop_counter < kStopTime) {
        stop_counter++;
      } else {
        dir_change = kFalse;
      }
    }
  }

  // update dirction
  dir_last_time = dir;

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

void MotorCtrlAuto(struct MOTOR_DATA *motor, struct PID_DATA *pid,
                   u32 cmd_speed, _Bool init_dir, u8 cycle) {
  u8 state;
  cycle_odometer_this_time = odometer[0];

  // first time
  if (cycle_counter == 0) {
    auto_dir = init_dir;
  }

  if (cycle_counter < cycle-1) {
    // 1m
    if ((cycle_odometer_this_time - cycle_odometer_last_time) > 100) {
      cycle_odometer_last_time = odometer[0];

      // change direction
      auto_dir = !auto_dir;
      cycle_counter++;
    }
    MotorCtrlManual(motor, pid, cmd_speed, auto_dir);

  } else {
    // stop
    MotorCtrlManual(motor, pid, 0, auto_dir);
  }
}

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
    if (motor_turn[i] < 0x01FF) {
      motor_turn_this_time[i] = motor_turn[i];
      delta_turn[i] = motor_turn_this_time[i] - motor_turn_last_time[i];
      // update
      motor_turn_last_time[i] = motor_turn_this_time[i];
    } else {
      // odometer
      odometer[i] = odometer[i] + (motor_turn[i] * kPI * kDiameter) /
                                      (6 * kReductionRatio * 10);
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

u32 MotorSetCmdSpeed(u32 cmd_speed, u32 motor_feedback_speed) {
  u32 motor_cmd_speed;
  if (abs(cmd_speed - motor_feedback_speed) > 100) {
    // stop
    if (cmd_speed == 0) {
      motor_cmd_speed = abs(cmd_speed - motor_feedback_speed) * 0.8;
    } else {
      motor_cmd_speed = 0.8 * cmd_speed;
    }
  } else {
    motor_cmd_speed = cmd_speed;
  }

  return motor_cmd_speed;
}