#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

//减速比
#define ReductionRatio  64

void TIM4_CH1_Cap_Init(u32 arr,u16 psc);
void TIM12_CH1_Cap_Init(u32 arr,u16 psc);
void TIM13_CH1_Cap_Init(u32 arr,u16 psc);

#endif























