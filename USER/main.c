// main.c
// CWR_SZ project

// Created by Song Junlin on 3/21/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
/*-------------------------------Includes-------------------------------*/
/*STM32*/
#include "Motor.h"
#include "adc.h"
#include "delay.h"
#include "led.h"
#include "pid.h"
#include "sys.h"
#include "timer.h"

/*PID*/
#include "pid.h"

/*FreeRTOS*/
#include "FreeRTOS.h"
#include "task.h"

/*FreeModbus*/
#include "FreeModbus.h"

/*robot*/
#include "robot.h"

/*--------------------------------TASK---------------------------------*/
//任务优先级
#define START_TASK_PRIO 1
//任务堆栈大小
#define START_STK_SIZE 128
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void* pvParameters);

//任务优先级
#define Modbus_TASK_PRIO 2
//任务堆栈大小
#define Modbus_STK_SIZE 256
//任务句柄
TaskHandle_t ModbusTask_Handler;
//任务函数
void Modbus_task(void* pvParameters);

//任务优先级
#define RobotState_TASK_PRIO 3
//任务堆栈大小
#define RobotState_STK_SIZE 256
//任务句柄
TaskHandle_t RobotStateTask_Handler;
//任务函数
void RobotState_task(void* pvParameters);

//任务优先级
#define ADC_TASK_PRIO 4
//任务堆栈大小
#define ADC_STK_SIZE 256
//任务句柄
TaskHandle_t ADCTask_Handler;
//任务函数
void ADC_task(void* pvParameters);

#define Robot_TASK_PRIO 5
//任务堆栈大小
#define Robot_STK_SIZE 256
//任务句柄
TaskHandle_t RobotTask_Handler;
//任务函数
void Robot_task(void* pvParameters);

/*----------------------------Configuration----------------------------------*/
// extern variables
extern const u8 kInMotion;
extern const u8 kStepDone;
extern const u8 kCycleDone;
// Robot registors
extern int32_t usRegHoldingBuf[REG_HOLDING_NREGS];
extern u32 cycle_counter;

const UCHAR kRobotAddr = 0x0A;
const u8 kRefereshRate = 50;
static const _Bool kManualMode = 0;
static const _Bool kManualAuto = 1;

/*----------------------------Robot state definition------------------------*/
const u8 kPowerOn = 0;
const u8 kInit = 2;
const u8 kEnabled = 3;
const u8 kError = 4;

/*----------------------------variables----------------------------------*/
static u8 robot_state;

/*----------------------------Start Implemention-------------------------*/

int main(void) {
  // power on
  robot_state = kPowerOn;

  //设置系统中断优先级分组4
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  //初始化延时函数
  delay_init(168);

  //初始化LED端口
  LED_Init();
  // light the indicator by default
  LED2 = 1;
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

/*------------------------------TASK Functions------------------------------*/

//开始任务任务函数
static void start_task(void* pvParameters) {
  //进入临界区
  taskENTER_CRITICAL();
  //创建ADC任务
  xTaskCreate((TaskFunction_t)ADC_task, (const char*)"ADC_task",
              (uint16_t)ADC_STK_SIZE, (void*)NULL, (UBaseType_t)ADC_TASK_PRIO,
              (TaskHandle_t*)&ADCTask_Handler);
  xTaskCreate((TaskFunction_t)Robot_task, (const char*)"Robot_task",
              (uint16_t)Robot_STK_SIZE, (void*)NULL,
              (UBaseType_t)Robot_TASK_PRIO, (TaskHandle_t*)&RobotTask_Handler);
  //创建Modbus任务
  xTaskCreate((TaskFunction_t)Modbus_task, (const char*)"Modbus_task",
              (uint16_t)Modbus_STK_SIZE, (void*)NULL,
              (UBaseType_t)Modbus_TASK_PRIO,
              (TaskHandle_t*)&ModbusTask_Handler);
  // RobotState任务
  // xTaskCreate((TaskFunction_t)RobotState_task, (const
  // char*)"RobotState_task",
  //             (uint16_t)RobotState_STK_SIZE, (void*)NULL,
  //             (UBaseType_t)RobotState_TASK_PRIO,
  //             (TaskHandle_t*)&RobotStateTask_Handler);

  vTaskDelete(StartTask_Handler);  //删除开始任务
  taskEXIT_CRITICAL();             //退出临界区
}

// ADC任务函数
static void ADC_task(void* pvParameters) {
  u16 adcx;
  float temp;
  Adc_Init();
  while (1) {
    adcx = Get_Adc_Average(ADC_Channel_5, 20);
    // mv
    temp = (float)adcx * (3.3 / 4096) * 12000;
    adcx = temp;
    usRegHoldingBuf[20] = adcx;
    vTaskDelay(kRefereshRate);  // 20ms刷新一次
  }
}

// Robot task
static void Robot_task(void* pvParameters) {
  u16 cycle;
  u8 dir = 1;
  u8 state;
  _Bool mode = 0;
  // robot class
  robot cwr;
  RobotNew(&cwr);
  cwr.Init();
  usRegHoldingBuf[0] = kRobotAddr;
  while (1) {
    // cycle = usRegHoldingBuf[1];

    cwr.dir_ = usRegHoldingBuf[2];
    cwr.mode_ = usRegHoldingBuf[21];
    cwr.cycle_ = usRegHoldingBuf[22];
    cwr.cmd_speed_ = usRegHoldingBuf[23];

    // reset
    if (usRegHoldingBuf[9] == 1) {
      cwr.Reset();
    }

    // robot enable
    if (usRegHoldingBuf[3] == 1) {
      cwr.Enable();

      if (cwr.mode_ == kManualMode) {
        // manual mode
        cwr.Manual(cwr.cmd_speed_, cwr.dir_);
      }

      if (cwr.mode_ == kManualAuto) {
        state = cwr.Auto(cwr.cmd_speed_, cwr.dir_, cwr.cycle_, &state);
        // if (state == kCycleDone) {
        //   // reset the cycle counter
        //   cycle_counter = 0;
        //   // disable
        //   usRegHoldingBuf[3] = 0;
        // }
      }

    } else if (usRegHoldingBuf[3] == 0) {
      // cwr.Disable();
      // stop
      cwr.Manual(0, cwr.dir_);

      /*
        if(error)
        else if (stop)
        ....
      */
    }

    // cycle done, refresh the enable
    // if (state == kCycleDone) {
    //   // usRegHoldingBuf[3]=0;
    // }

    usRegHoldingBuf[16] = state;
    // 100ms刷新一次
    vTaskDelay(100);
  }
}

// Modbus任务函数
static void Modbus_task(void* pvParameters) {
  // Modbus Init
  eMBInit(MB_RTU, kRobotAddr, 0x01, 19200, MB_PAR_NONE);
  eMBEnable();
  while (1) {
    eMBPoll();
    // 20ms刷新一次，Delay期间任务被BLOCK，可以执行其他任务
    vTaskDelay(kRefereshRate);
  }
}

// RobotState task
static void RobotState_task(void* pvParameters) {
  while (1) {
    vTaskDelay(100);  // Delay期间任务被BLOCK，可以执行其他任务
  }
}
