#include "motor.h"
#include "FreeModbus.h"
#include "PID.h"

extern pidData_t PIDMotor1;
extern pidData_t PIDMotor3;
extern int32_t usRegHoldingBuf[REG_HOLDING_NREGS];
u32 motor_turn[MotorNum];
static u8 exit_int_time_motor1 = 0;
static u8 exit_int_time_motor2 = 0;
static u8 exit_int_time_motor3 = 0;
static u8 exit_int_time_motor4 = 0;
static u8 exit_int_time_motor5 = 0;
static u8 exit_int_time_motor6 = 0;

/*Init TIM1 in PWM mode*/
/*Frequency in Hz*/
void TIM1_PWM_Init(u32 freq) {
  /* Private typedef
   * -----------------------------------------------------------*/
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  // TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

  /* Private variables
   * ---------------------------------------------------------*/
  int TimerPeriod = 0;
  /* TIM1 Configuration */
  TIM1_Config();

  /* Compute the value to be set in ARR register to generate the desired signal
   * frequency */
  TimerPeriod = ((168000000 / 1) / freq) - 1;  // SystemCoreClock is 168MHz.

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
  // TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;

  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  // TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  // TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  /*50%*/
  TIM_OCInitStructure.TIM_Pulse = (TimerPeriod * 50 / 100);

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
  TIM_BDTRInitStructure.TIM_DeadTime = 25; ///////// the right value for 250ns
  delay //////// TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;*/

  // TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

  // Initialize the motor with speed = 0.
  TIM1_PWM_SET(freq, 100);
}

void TIM8_PWM_Init(u32 freq) {
  /* Private typedef
   * -----------------------------------------------------------*/
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  // TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

  /* Private variables
   * ---------------------------------------------------------*/
  int TimerPeriod = 0;
  /* TIM1 Configuration */
  TIM8_Config();

  /* Compute the value to be set in ARR register to generate the desired signal
   * frequency */
  TimerPeriod = ((168000000 / 1) / freq) - 1;  // SystemCoreClock is 168MHz.

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

  /* Channel 1~4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  // TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;

  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  // TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  // TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  /*50%*/
  TIM_OCInitStructure.TIM_Pulse = (TimerPeriod * 50 / 100);

  TIM_OC1Init(TIM8, &TIM_OCInitStructure);

  TIM_OC2Init(TIM8, &TIM_OCInitStructure);

  /* TIM8 counter enable */
  TIM_Cmd(TIM8, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM8, ENABLE);

  // Initialize the motor with speed = 0.
  TIM8_PWM_SET(freq, 100);
}

/**
 *  Configure the TIM1 Pins.
 */
void TIM1_Config(void) {
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
  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Connect TIM pins to AF1 */
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
}

void TIM8_Config(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOE, GPIOB and GPIOC clocks enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* TIM8 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  /*GPIOC Configuration: Channel 1N and BKIN as alternate function push-pull*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Connect TIM pins to AF1 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
}
//电机对应IO初始化
void Motor_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  //使能GPIOD时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |
                                GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;  //
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);              //初始化GPIO

  GPIO_SetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);
  GPIO_ResetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5);
}

void TIM1_PWM_SET(u32 freq, u32 Duty) {
  TIM1->CCR1 = (((168000000 / 1) / freq) - 1) * Duty / 100;
  TIM1->CCR2 = (((168000000 / 1) / freq) - 1) * Duty / 100;

  TIM1->CCR3 = (((168000000 / 1) / freq) - 1) * Duty / 100;
  TIM1->CCR4 = (((168000000 / 1) / freq) - 1) * Duty / 100;
}

void TIM8_PWM_SET(u32 freq, u32 Duty) {
  TIM8->CCR1 = (((168000000 / 1) / freq) - 1) * Duty / 100;
  TIM8->CCR2 = (((168000000 / 1) / freq) - 1) * Duty / 100;
}

void MoveUp(void) {
  GPIO_SetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3);
  GPIO_ResetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1);
}

void MoveDown(void) {
  GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3);
  GPIO_SetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1);
}

void MotorFGInit(void) {
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  EXTIX_Init();  //

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /*FG1*/
  // SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource14);

  // EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  // EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  // EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  // EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  // EXTI_Init(&EXTI_InitStructure);

  // NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
  // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  // NVIC_Init(&NVIC_InitStructure);
  /*FG2*/
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);

  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /*FG3~6*/
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource12);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource13);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource14);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource15);

  EXTI_InitStructure.EXTI_Line =
      EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void EXTIX_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(
      RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD,
      ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void EXTI15_10_IRQHandler(void) {
  /*motor 3*/
  if (EXTI_GetITStatus(EXTI_Line12)) {
    /*interrupted first time */
    if (exit_int_time_motor3 == 0) {
      motor_turn[2] = 6;
      // Tim5Enable();
      exit_int_time_motor3 = 1;
    } else {
      motor_turn[2] = motor_turn[2] + 6;
    }
    // usRegHoldingBuf[9] = motor_turn[2];
    EXTI_ClearITPendingBit(EXTI_Line12);
  }
  /*motor 4*/
  if (EXTI_GetITStatus(EXTI_Line13)) {
    /*interrupted first time */
    if (exit_int_time_motor4 == 0) {
      motor_turn[3] = 6;
      // Tim5Enable();
      exit_int_time_motor4 = 1;
    } else {
      motor_turn[3] = motor_turn[3] + 6;
    }
    // usRegHoldingBuf[9] = motor_turn[0];
    EXTI_ClearITPendingBit(EXTI_Line13);
  }
  /*motor 5*/
  if (EXTI_GetITStatus(EXTI_Line14)) {
    /*interrupted first time */
    if (exit_int_time_motor5 == 0) {
      motor_turn[4] = 6;
      // Tim5Enable();
      exit_int_time_motor5 = 1;
    } else {
      motor_turn[4] = motor_turn[4] + 6;
    }
    // usRegHoldingBuf[9] = motor_turn[0];
    EXTI_ClearITPendingBit(EXTI_Line14);
  }
  /*motor 6*/
  if (EXTI_GetITStatus(EXTI_Line15)) {
    /*interrupted first time */
    if (exit_int_time_motor6 == 0) {
      motor_turn[5] = 6;
      // Tim5Enable();
      exit_int_time_motor6 = 1;
    } else {
      motor_turn[5] = motor_turn[5] + 6;
    }
    // usRegHoldingBuf[9] = motor_turn[0];
    EXTI_ClearITPendingBit(EXTI_Line15);
  }
}

void EXTI9_5_IRQHandler(void) {
  /*interrupted first time */
  if (exit_int_time_motor2 == 0) {
    motor_turn[1] = 6;
    Tim5Enable();
    exit_int_time_motor2 = 1;
  } else {
    motor_turn[1] = motor_turn[1] + 6;
  }

  EXTI_ClearITPendingBit(EXTI_Line6);
}