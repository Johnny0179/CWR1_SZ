#ifndef __LED_H
#define __LED_H
#include "sys.h"

////////////////////////////////////////////////////////////////////////////////////	

//LED端口定义
#define LED0 PEout(2)	 
#define LED1 PEout(3)	 
 

//函数声明
void LED_Init(void);//初始化	

#endif
