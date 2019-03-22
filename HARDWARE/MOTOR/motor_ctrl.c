#include "FreeModbus.h"
#include "PID.h"
#include "motor.h"
#include "timer.h"

u32 MotorSpeed[MotorNum];
extern int32_t usRegHoldingBuf[REG_HOLDING_NREGS];
u32 delta_turn[MotorNum];
/*ms*/
const u16 kTimeInteval = 100;
const float kPI = 3.14;
// diameter
const u8 kDiameter = 45;
// reduction ratio
const u8 kReductionRatio = 43;

static volatile motor_turn_this_time[MotorNum] = {0};
static volatile motor_turn_last_time[MotorNum] = {0};

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

void MotorCtrl(struct MOTOR_DATA *motor, struct PID_DATA *pid) {
  motor->direction = usRegHoldingBuf[0];
  motor->CmdSpeed = usRegHoldingBuf[2];
  motor->MotorSpeed_mmps = MotorVelCalc(delta_turn[motor->num - 1]);

  motor->PWM = motor->PWM +
               pid_Controller(motor->CmdSpeed, motor->MotorSpeed_mmps, pid) / 2;

  // PWM 0~100%
  if (motor->PWM < 0) {
    motor->PWM = 0;
  } else if (motor->PWM > 100) {
    // Max motor->PWM Ratio
    motor->PWM = 100;
  }

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

void MotorInitConfig(u8 num, struct MOTOR_DATA *motor) {
  motor->num = num;
  motor->direction = 2;
  motor->CmdSpeed = 0;
  motor->PWM = 0;
  motor->MotorSpeed_mmps = 0;
}

u32 MotorVelCalc(u32 delta_turn) {
  return (delta_turn * kPI * kDiameter * kTimeInteval) /
         (kReductionRatio * 1000);
}

u32 DeltaTurnCalc(u32 *motor_turn,u8 motor_num) {
	u8 i=0;
  for (i = 0; i <= motor_num - 1; i++) {
    motor_turn_this_time[i] = motor_turn[i];
    delta_turn[i] = motor_turn_this_time[i] - motor_turn_last_time[i];
    // update
    motor_turn_last_time[i] = motor_turn_this_time[i];
  }
}