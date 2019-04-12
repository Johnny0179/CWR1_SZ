#include "timer.h"
#include "led.h"

void Tim13IntInit(u16 arr, u16 psc) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

  TIM_TimeBaseInitStructure.TIM_Period = arr;
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM13, &TIM_TimeBaseInitStructure);

  TIM_ITConfig(TIM13, TIM_IT_Update, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Tim13Enable(void) { TIM_Cmd(TIM13, ENABLE); }
void Tim13Disable(void) { TIM_Cmd(TIM13, DISABLE); }

/*interrupt handler*/
void TIM8_UP_TIM13_IRQHandler(void) {
  if (TIM_GetITStatus(TIM13, TIM_IT_Update) == SET) {
    LED2 = !LED2;
  }
  TIM_ClearITPendingBit(TIM13, TIM_IT_Update);
}
