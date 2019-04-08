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
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

//STM32操作相关头文件
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
/**
  * @brief  控制接收和发送状态
  * @param  xRxEnable 接收使能、
  *         xTxEnable 发送使能
  * @retval None
  */
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
	
	  if(xRxEnable)
  {
    //使能接收和接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //MAX485操作 低电平为接收模式
    //GPIO_ResetBits(GPIOG,GPIO_Pin_8);
  }
  else
  {
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); 
    //MAX485操作 高电平为发送模式
    //GPIO_SetBits(GPIOG,GPIO_Pin_8);
  }

  if(xTxEnable)
  {
    //使能发送完成中断
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  }
  else
  {
    //禁止发送完成中断
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
  }
}

/**
  * @brief  串口初始化
  * @param  ucPORT      串口号
  *         ulBaudRate  波特率
  *         ucDataBits  数据位
  *         eParity     校验位 
  * @retval None
  */
BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	(void)ucPORT;     //不修改串口
  (void)ucDataBits; //不修改数据位长度
  (void)eParity;    //不修改校验格式


   //GPIO端口设置
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = ulBaudRate;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1
  

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  //设定USART1 中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //最后配置485发送和接收模式
/*	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOD时钟
	//GPIOG8初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure); */

  return TRUE; 
}

/**
  * @brief  通过串口发送数据
  * @param  None
  * @retval None
  */
BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
	  //发送数据
  USART_SendData(USART1, ucByte);
    return TRUE;
}

/**
  * @brief  从串口获得数据
  * @param  None
  * @retval None
  */
BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	  //接收数据
  *pucByte = USART_ReceiveData(USART1);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{
	//mb.c eMBInit函数中
  //pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM 
  //发送状态机
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
	//mb.c eMBInit函数中
  //pxMBFrameCBByteReceived = xMBRTUReceiveFSM
  //接收状态机
    pxMBFrameCBByteReceived(  );
}


u8 flag_rx = 0;

/**
  * @brief  USART1中断服务函数
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  //发生接收中断
  if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
  {
    prvvUARTRxISR(); 
    //清除中断标志位    
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);   
  }
  
  //发生完成中断
  if(USART_GetITStatus(USART1, USART_IT_TC) == SET)
  {
    prvvUARTTxReadyISR();
    //清除中断标志
    USART_ClearITPendingBit(USART1, USART_IT_TC);
  }
  
  //测试看是否可以去除 2012-07-23
  //溢出-如果发生溢出需要先读SR,再读DR寄存器 则可清除不断入中断的问题
  /*
  if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
  {
    USART_ClearFlag(USART1,USART_FLAG_ORE); //读SR
    USART_ReceiveData(USART1);              //读DR
  }
  */
}
