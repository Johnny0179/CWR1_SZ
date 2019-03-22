
#include "FreeModbus.h"
#include "led.h"
#include "motor.h"
#include "timer.h"

extern u32 motor_turn[MotorNum];

void Tim5IntInit(u16 arr, u16 psc) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  TIM_TimeBaseInitStructure.TIM_Period = arr;
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Tim5Enable(void) { TIM_Cmd(TIM5, ENABLE); }

/*interrupt handler*/
void TIM5_IRQHandler(void) {
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET) {
    LED0 = !LED0;

    DeltaTurnCalc(motor_turn, MotorNum);
  }
  TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
}