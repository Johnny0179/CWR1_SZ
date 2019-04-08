/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

//STM32相关头文件
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_tim.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR( void );

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  //
  uint16_t PrescalerValue = 0;
  
  //使能TIM3时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  //定时器时间基配置说明
  //HCLK为168MHz，APB1经过4分频为42MHz
  //TIM3的时钟倍频后为84MHz（硬件自动倍频,达到84MHz）
  //TIM3的分频系数为4199，时间基频率为84 / (1 + Prescaler) = 20KHz,基准为50us
  //TIM最大计数值为usTim1Timerout50u
  //TIM3时钟为84M，50us基准,定时器计数周期为20KHz
  PrescalerValue = 4200 - 1; 
  //TIM3初始化
  TIM_TimeBaseStructure.TIM_Period = (uint16_t) usTim1Timerout50us;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  //预装载使能
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  
  //
  // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  //TIM3中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //清除溢出中断标志位
  TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
  //TIM3溢出中断关闭
  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
  //TIM3禁能
  TIM_Cmd(TIM3,  DISABLE);
  return TRUE;
}



 void vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		//设定TIM3的初始值
		TIM_SetCounter(TIM3,0x0000); 
		//TIM3启动
		TIM_Cmd(TIM3, ENABLE);
}


 void vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
		TIM_SetCounter(TIM3,0x0000); 
		//关闭TIM3
		TIM_Cmd(TIM3, DISABLE);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}

/**
  * @brief  TIM3中断服务函数
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    //清除定时器T4溢出中断标志位
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    prvvTIMERExpiredISR( );
  }
}
