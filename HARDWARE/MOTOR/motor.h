#ifndef _MOTOR_H
#define _MOTOR_H
#include "sys.h"
#include "PID.h"
//////////////////////////////////////////////////////////////////////////////////	 
//Copyright(C) IRIM 2018
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////



#define PWMfreq 20000
#define moveup 2
#define movedown 1
#define stop 0
#define ModeAuto 1
#define ModeManual 0

#define P 300
#define I 2
#define D 0

typedef struct MOTOR_DATA{
	u8 num;
  int32_t CmdSpeed;
  uint32_t MotorSpeed_mmps;
  uint32_t direction;
  int32_t PWM;
}MOTOR;

void Motor_Init(void);
void MoveUp(void);
void MoveDown(void);


void TIM1_Config(void);
void TIM1_PWM_Init(u32 freq);

void TIM1_PWM_SET(u32 freq,u32 Duty);

void MotorInit(void);
void MotorInitConfig(u8 num,struct MOTOR_DATA *motor);
void MotorCtrl(struct MOTOR_DATA *motor, struct PID_DATA *pid);

#endif
