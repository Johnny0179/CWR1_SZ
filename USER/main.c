/*STM32*/
#include "Motor.h"
#include "delay.h"
#include "led.h"
#include "sys.h"

/*FreeRTOS*/
#include "FreeRTOS.h"
#include "task.h"

/*FreeModbus*/
#include "FreeModbus.h"

/************************************************

************************************************/

//任务优先级
#define START_TASK_PRIO 1
//任务堆栈大小
#define START_STK_SIZE 128
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void* pvParameters);

//任务优先级
#define Modbus_TASK_PRIO 4
//任务堆栈大小
#define Modbus_STK_SIZE 128
//任务句柄
TaskHandle_t ModbusTask_Handler;
//任务函数
void Modbus_task(void* pvParameters);

//任务优先级
#define Motor_TASK_PRIO 3
//任务堆栈大小
#define Motor_STK_SIZE 50
//任务句柄
TaskHandle_t MotorTask_Handler;
//任务函数
void Motor_task(void* pvParameters);



/*//任务优先级
#define LED0_TASK_PRIO 2
//任务堆栈大小
#define LED0_STK_SIZE 50
//任务句柄
TaskHandle_t LED0Task_Handler;
//任务函数
void led0_task(void* pvParameters);*/

// Robot registors
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];

u8 MoveInteval = 5;

const int freq = 20000;  // Freq

int main(void) {
  //设置系统中断优先级分组4
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  //初始化延时函数
  delay_init(168);

  //初始化LED端口
  LED_Init();

  // Freq=20k,DutyRatio=50
  TIM1_PWM_Init(freq);
  Motor_Init();

  // Set the Motor speed.
  usRegHoldingBuf[0] = moveup;  // Direction
  usRegHoldingBuf[1] = 0;       // Speed
  usRegHoldingBuf[2] = ModeManual;
  usRegHoldingBuf[3] = MoveInteval;

  //创建开始任务
  xTaskCreate((TaskFunction_t)start_task,    //任务函数
              (const char*)"start_task",     //任务名称
              (uint16_t)START_STK_SIZE,      //任务堆栈大小
              (void*)NULL,                   //传递给任务函数的参数
              (UBaseType_t)START_TASK_PRIO,  //任务优先级
              (TaskHandle_t*)&StartTask_Handler);  //任务句柄
  vTaskStartScheduler();                           //开启任务调度
  return 0;
}

//开始任务任务函数
static void start_task(void* pvParameters) {
  taskENTER_CRITICAL();  //进入临界区
/*  //创建LED0任务
  xTaskCreate((TaskFunction_t)led0_task, (const char*)"led0_task",
              (uint16_t)LED0_STK_SIZE, (void*)NULL, (UBaseType_t)LED0_TASK_PRIO,
              (TaskHandle_t*)&LED0Task_Handler);*/
  //创建Modbus任务
  xTaskCreate((TaskFunction_t)Modbus_task, (const char*)"Modbus_task",
              (uint16_t)Modbus_STK_SIZE, (void*)NULL,
              (UBaseType_t)Modbus_TASK_PRIO,
              (TaskHandle_t*)&ModbusTask_Handler);
  // Motor任务
  xTaskCreate((TaskFunction_t)Motor_task, (const char*)"Motor_task",
              (uint16_t)Motor_STK_SIZE, (void*)NULL,
              (UBaseType_t)Motor_TASK_PRIO, (TaskHandle_t*)&MotorTask_Handler);

  vTaskDelete(StartTask_Handler);  //删除开始任务
  taskEXIT_CRITICAL();             //退出临界区
}

/*// LED0任务函数
static void led0_task(void* pvParameters) {
  while (1) {
    LED0 = ~LED0;
		vTaskDelay(200);
  }
}*/

// Modbus任务函数
static void Modbus_task(void* pvParameters) {
  // Modbus Init
  eMBInit(MB_RTU, 0x0A, 0x01, 9600, MB_PAR_NONE);
  eMBEnable();
  while (1) {
    eMBPoll();
		vTaskDelay(1);//?为什么需要Delay?
  }
}

// Motor任务
static void Motor_task(void* pvParameters) {
  while (1) {
    /*Mannual Mode*/
    if(usRegHoldingBuf[2]==0){
    MotorCrl(usRegHoldingBuf[0], usRegHoldingBuf[1]);
    delay_ms(1);}

    /*Auto Mode*/
    if(usRegHoldingBuf[2]==1){
      MotorCrl(2, usRegHoldingBuf[1]);//MoveUp
      delay_ms(usRegHoldingBuf[3]*1000);
      MotorCrl(1, usRegHoldingBuf[1]);//MoveUp
      delay_ms(usRegHoldingBuf[3]*1000); 
  }
  }
}

