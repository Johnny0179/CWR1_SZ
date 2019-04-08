/* Includes ------------------------------------------------------------------*/
#include "pwm.h"

/*Init TIM1 in PWM mode*/
/*Frequency in Hz*/ 
void TIM1_PWM_Init(u32 freq) 
{	
	/* Private typedef -----------------------------------------------------------*/
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	//TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

/* Private variables ---------------------------------------------------------*/
	int TimerPeriod = 0;
  /* TIM1 Configuration */
  TIM1_Config();

  /* Compute the value to be set in ARR register to generate the desired signal frequency */
  TimerPeriod = ((168000000/1) / freq) - 1;//SystemCoreClock is 168MHz. 

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1~4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;

  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  //TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  /*50%*/
  TIM_OCInitStructure.TIM_Pulse = (TimerPeriod*50/100);
  
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_OC2Init(TIM1, &TIM_OCInitStructure);

  TIM_OC3Init(TIM1, &TIM_OCInitStructure);

  TIM_OC4Init(TIM1, &TIM_OCInitStructure);

/*  TIM_OCInitStructure.TIM_Pulse = (TimerPeriod/6) ;
  TIM_OC2Init(TIM8, &TIM_OCInitStructure);*/

  /* Automatic Output enable, Break, dead time and lock configuration*/
  /*TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
  TIM_BDTRInitStructure.TIM_DeadTime = 25; ///////// the right value for 250ns delay ////////
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;*/

  //TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/**
  *  Configure the TIM1 Pins.
  */
void TIM1_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOE, GPIOB and GPIOC clocks enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  /*GPIOE Configuration: Channel 1N and BKIN as alternate function push-pull*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_Init(GPIOE, &GPIO_InitStructure);


  /* Connect TIM pins to AF1 */
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
}

void TIM1_PWM_SET(u32 freq,u32 Duty)
{
  TIM1->CCR1=(((168000000/1) / freq) - 1)*Duty/100;
  TIM1->CCR2=(((168000000/1) / freq) - 1)*Duty/100;
  TIM1->CCR3=(((168000000/1) / freq) - 1)*Duty/100;
  TIM1->CCR4=(((168000000/1) / freq) - 1)*Duty/100;
}

