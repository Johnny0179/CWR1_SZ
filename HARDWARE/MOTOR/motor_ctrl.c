#include "FreeModbus.h"
#include "PID.h"
#include "motor.h"
#include "timer.h"

int32_t MotorSpeed[4];
extern int32_t usRegHoldingBuf[REG_HOLDING_NREGS];

void MotorInit(void) {
  // Freq=20k,DutyRatio=50
  TIM1_PWM_Init(PWMfreq);
  TIM8_PWM_Init(PWMfreq);

  Motor_Init();

  // Capture Init
  TIM4_CH1_Cap_Init(0XFFFF, 84 - 1);   //以1Mhz的频率计数
  TIM12_CH1_Cap_Init(0XFFFF, 84 - 1);  //以1Mhz的频率计数
  TIM13_CH1_Cap_Init(0XFFFF, 84 - 1);  //以1Mhz的频率计数
}

void MotorCtrl(struct MOTOR_DATA *motor, struct PID_DATA *pid) {
  motor->direction = usRegHoldingBuf[0];
  motor->CmdSpeed = usRegHoldingBuf[2];

  motor->MotorSpeed_mmps = MotorSpeed[motor->num - 1];

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

  /*  //清零
    if (motor->CmdSpeed == 0) {
      //motor->PWM = 0;
    }*/

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
