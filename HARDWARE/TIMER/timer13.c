#include "PID.h"
#include "timer.h"
#include "motor.h"
#include "FreeModbus.h"

extern int32_t MotorSpeed[MotorNum];
// extern int32_t usRegHoldingBuf[REG_HOLDING_NREGS];

// TIM13通道1输入捕获配置
// arr：自动重装值(TIM2,TIM5是32位的!!)
// psc：时钟预分频数
void TIM13_CH1_Cap_Init(u32 arr, u16 psc) {
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM13_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);  // TIM13时钟使能
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //使能PORTD时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;          // GPIOA
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //速度100MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;      //下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);              //初始化PB14

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, 0x09);  // PA6复用位TIM13

  TIM_TimeBaseStructure.TIM_Prescaler = psc;                   //定时器分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
  TIM_TimeBaseStructure.TIM_Period = arr;  //自动重装载值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

  TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);

  //初始化TIM13输入捕获参数
  TIM13_ICInitStructure.TIM_Channel =
      TIM_Channel_1;  // CC1S=01 	选择输入端 IC1映射到TI1上
  TIM13_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升沿捕获
  TIM13_ICInitStructure.TIM_ICSelection =
      TIM_ICSelection_DirectTI;                            //映射到TI1上
  TIM13_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;  //配置输入分频,不分频
  TIM13_ICInitStructure.TIM_ICFilter = 0x00;  // IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM13, &TIM13_ICInitStructure);
  //允许更新中断 ,允许CC1IE捕获中断
  TIM_ITConfig(TIM13, TIM_IT_Update | TIM_IT_CC1, ENABLE);
  //使能TIM13
  TIM_Cmd(TIM13, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器、
}

// 捕获值记录1、2，捕获次数
static u16 IC_ReadValue1_Motor2 = 0;
static u16 IC_ReadValue2_Motor2 = 0;
static u16 CaptureNumber_Motor2 = 0;
//最终值
static u32 TIM13Capture = 0;
//转速，单位0.001r/s
static u32 rps_e_3_Motor2 = 0;


// TIM13中断服务程序
void TIM8_UP_TIM13_IRQHandler(void) {
  if (TIM_GetITStatus(TIM13, TIM_IT_Update) != RESET)  //计数器溢出
  {
    TIM_ClearITPendingBit(TIM13, TIM_IT_Update);
  }

  else if (TIM_GetITStatus(TIM13, TIM_IT_CC1) != RESET)  //捕获发生
  {
    TIM_ClearITPendingBit(TIM13, TIM_IT_CC1);
    if (CaptureNumber_Motor2 == 0) {
      IC_ReadValue1_Motor2 = TIM_GetCapture1(TIM13);
      CaptureNumber_Motor2 = 1;
    } else if (CaptureNumber_Motor2 == 1) {
      IC_ReadValue2_Motor2 = TIM_GetCapture1(TIM13);
      if (IC_ReadValue2_Motor2 > IC_ReadValue1_Motor2) {
        TIM13Capture = (IC_ReadValue2_Motor2 - IC_ReadValue1_Motor2);
      } else {
        TIM13Capture = (0xFFFF - IC_ReadValue2_Motor2 + IC_ReadValue1_Motor2);
      }
      CaptureNumber_Motor2 = 0;
    }
  }

  //剔除不正常数据
  if (((u32)10000000 / TIM13Capture) > 150) {
    //转速
    rps_e_3_Motor2 = (u32)1000000000 / TIM13Capture / (6 * ReductionRatio);
    //速度
    MotorSpeed[1] = rps_e_3_Motor2 * 3.14 * 45 / 1000;
  }
}
