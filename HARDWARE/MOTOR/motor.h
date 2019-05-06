#ifndef _MOTOR_H
#define _MOTOR_H
#include <math.h>
#include "PID.h"
#include "core_cm4.h"
#include "core_cmFunc.h"
#include "delay.h"
#include "stm32f4xx_exti.h"
#include "sys.h"
#include "timer.h"
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

#define P 400
#define I 10
#define D 10

typedef struct MOTOR_DATA {
  u8 num;
  uint32_t CmdSpeed;
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
u32 MotorSetCmdSpeed(u32 cmd_speed, u32 motor_feedback_speed);
void MotorInit(void);
void MotorInitConfig(u8 num, struct MOTOR_DATA *motor);
u8 MotorCtrlManual(struct MOTOR_DATA *motor, struct PID_DATA *pid,
                     const u32 *cmd_speed, _Bool dir);
u8 MotorCtrlAuto(struct MOTOR_DATA *motor, struct PID_DATA *pid,
                 const u32 *cmd_speed, _Bool init_dir, u8 cycle, u8 *state);

void MotorDisable(void);
void MotorEnable(void);
void MotorReset(void);
void MotorPWMSet(u8 motor_id,u32 pwm);

u32 DeltaTurnCalc(u32 *motor_turn, u8 motor_num);
u32 MotorVelCalc(u32 delta_turn);

u32 Acceleration(u8 N, u32 cmd_speed);
u32 Deceleration(u8 N, u32 cmd_speed);
u8 StateCheck(u8 state, u32 this_time, u32 last_time,u32 motor_speed);
u32 SetSpeed(u8 state, u32 cmd_speed);
#endif