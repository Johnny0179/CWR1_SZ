#include "PID.h"
#include "timer.h"
#include "motor.h"
#include "FreeModbus.h"

extern int32_t MotorSpeed[MotorNum];
// extern int32_t usRegHoldingBuf[REG_HOLDING_NREGS];

// TIM12通道1输入捕获配置
// arr：自动重装值(TIM2,TIM5是32位的!!)
// psc：时钟预分频数
void TIM12_CH1_Cap_Init(u32 arr, u16 psc) {
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM12_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);  // TIM12时钟使能
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  //使能PORTD时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;          // GPIOB
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //速度100MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;      //下拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);              //初始化PB14

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, 0x09);  // PB14复用位TIM12

  TIM_TimeBaseStructure.TIM_Prescaler = psc;                   //定时器分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
  TIM_TimeBaseStructure.TIM_Period = arr;  //自动重装载值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

  TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

  //初始化TIM12输入捕获参数
  TIM12_ICInitStructure.TIM_Channel =
      TIM_Channel_1;  // CC1S=01 	选择输入端 IC1映射到TI1上
  TIM12_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升沿捕获
  TIM12_ICInitStructure.TIM_ICSelection =
      TIM_ICSelection_DirectTI;                            //映射到TI1上
  TIM12_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;  //配置输入分频,不分频
  TIM12_ICInitStructure.TIM_ICFilter = 0x00;  // IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM12, &TIM12_ICInitStructure);
  //允许更新中断 ,允许CC1IE捕获中断
  TIM_ITConfig(TIM12, TIM_IT_Update | TIM_IT_CC1, ENABLE);
  //使能TIM12
  TIM_Cmd(TIM12, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器、
}

// 捕获值记录1、2，捕获次数
static u16 IC_ReadValue1_Motor1 = 0;
static u16 IC_ReadValue2_Motor1 = 0;
static u16 CaptureNumber_Motor1 = 0;
//最终值
static u32 TIM12Capture = 0;
//转速，单位0.001r/s
static u32 rps_e_3_Motor1 = 0;


// TIM12中断服务程序
void TIM8_BRK_TIM12_IRQHandler(void) {
  if (TIM_GetITStatus(TIM12, TIM_IT_Update) != RESET)  //计数器溢出
  {
    TIM_ClearITPendingBit(TIM12, TIM_IT_Update);
  }

  else if (TIM_GetITStatus(TIM12, TIM_IT_CC1) != RESET)  //捕获发生
  {
    TIM_ClearITPendingBit(TIM12, TIM_IT_CC1);
    if (CaptureNumber_Motor1 == 0) {
      IC_ReadValue1_Motor1 = TIM_GetCapture1(TIM12);
      CaptureNumber_Motor1 = 1;
    } else if (CaptureNumber_Motor1 == 1) {
      IC_ReadValue2_Motor1 = TIM_GetCapture1(TIM12);
      if (IC_ReadValue2_Motor1 > IC_ReadValue1_Motor1) {
        TIM12Capture = (IC_ReadValue2_Motor1 - IC_ReadValue1_Motor1);
      } else {
        TIM12Capture = (0xFFFF - IC_ReadValue2_Motor1 + IC_ReadValue1_Motor1);
      }
      CaptureNumber_Motor1 = 0;
    }
  }

  //剔除不正常数据
  if (((u32)10000000 / TIM12Capture) > 150) {
    //转速
    rps_e_3_Motor1 = (u32)1000000000 / TIM12Capture / (6 * ReductionRatio);
    //速度
    MotorSpeed[0] = rps_e_3_Motor1 * 3.14 * 45 / 1000;
  }
}
