
#include "FreeModbus.h"
#include "led.h"
#include "motor.h"
#include "timer.h"

extern u8 stage_num;
void Tim4IntInit(u16 arr, u16 psc) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIM_TimeBaseInitStructure.TIM_Period = arr;
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Tim4Enable(void) { TIM_Cmd(TIM4, ENABLE); }
void Tim4Disable(void) { TIM_Cmd(TIM4, DISABLE); }

/*interrupt handler*/
void TIM4_IRQHandler(void) {
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) {
    // LED2 = !LED2;
    stage_num=stage_num+1;
  }
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}