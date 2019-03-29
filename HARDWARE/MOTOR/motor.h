#ifndef _MOTOR_H
#define _MOTOR_H
#include "PID.h"
#include "sys.h"
#include "stm32f4xx_exti.h"
#include "timer.h"
#include "delay.h"
#include "core_cm4.h"
#include "core_cmFunc.h"
#include <math.h>
//////////////////////////////////////////////////////////////////////////////////
// Copyright(C) IRIM 2018
// All rights reserved
//////////////////////////////////////////////////////////////////////////////////

#define MotorNum 6

#define PWMfreq 20000

#define moveup 0
#define movedown 1

#define stop 2
#define ModeAuto 1
#define ModeManual 0

#define P 300
#define I 10
#define D 0

typedef struct MOTOR_DATA {
  u8 num;
  int32_t CmdSpeed;
  uint32_t MotorSpeed_mmps;
  uint32_t direction;
  int32_t PWM;
  int32_t mode;
} MOTOR;

void Motor_Init(void);
void MoveDir(int8_t dir);
void MoveUp(void);
void MoveDown(void);

void TIM1_Config(void);
void TIM8_Config(void);
void TIM1_PWM_Init(u32 freq);
void TIM8_PWM_Init(u32 freq);

void TIM1_PWM_SET(u32 freq, u32 Duty);
void TIM8_PWM_SET(u32 freq, u32 Duty);

void EXTIX_Init(void);

void MotorFGInit(void);

void MotorInit(void);
void MotorInitConfig(u8 num, struct MOTOR_DATA *motor);
void MotorCtrlManual(struct MOTOR_DATA *motor, struct PID_DATA *pid,u32 cmd_speed,_Bool dir);
void MotorCtrlAuto(struct MOTOR_DATA *motor, struct PID_DATA *pid);

void MotorDisable(void);
void MotorEnable(void);


u32 DeltaTurnCalc(u32 *motor_turn,u8 motor_num);
u32 MotorVelCalc(u32 delta_turn);
#endif