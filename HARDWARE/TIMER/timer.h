#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

//减速比
#define ReductionRatio  43

void TIM4_CH1_Cap_Init(u32 arr,u16 psc);
void TIM4_CH2_Cap_Init(u32 arr,u16 psc);
void TIM4_CH3_Cap_Init(u32 arr,u16 psc);
void TIM4_CH4_Cap_Init(u32 arr,u16 psc);
void TIM12_CH1_Cap_Init(u32 arr,u16 psc);
void TIM13_CH1_Cap_Init(u32 arr,u16 psc);

#endif























