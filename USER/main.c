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
#include "state.h"
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
#define Modbus_TASK_PRIO 5
//任务堆栈大小
#define Modbus_STK_SIZE 256
//任务句柄
TaskHandle_t ModbusTask_Handler;
//任务函数
void Modbus_task(void* pvParameters);

//任务优先级
#define StateCheck_TASK_PRIO 4
//任务堆栈大小
#define StateCheck_STK_SIZE 256
//任务句柄
TaskHandle_t StateCheckTask_Handler;
//任务函数
void StateCheck_task(void* pvParameters);

//任务优先级
#define ADC_TASK_PRIO 2
//任务堆栈大小
#define ADC_STK_SIZE 256
//任务句柄
TaskHandle_t ADCTask_Handler;
//任务函数
void ADC_task(void* pvParameters);

#define Robot_TASK_PRIO 3
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
static const u8 kRefereshRate = 50;
static const _Bool kManualMode = 0;
static const _Bool kManualAuto = 1;
// battery voltage threhold 23V
static const int32_t kBatVoltTHR = 24000;

/*----------------------------Robot state definition------------------------*/
extern const u8 kIdle;
extern const u8 kCounterCheck;
extern const u8 kChangeDir;
extern const u8 kFirstCheck;
extern const u8 kMotion;
extern const u8 kStop;
extern const u8 kDone;

/*----------------------------variables----------------------------------*/
static u8 robot_state;
static const u8 kPowerOn = 0;
static const u8 kReady = 2;
static const u8 kEnabled = 3;
static const u8 kAlarm = 4;

static u8 err_code;

/*----------------------------Start Implemention-------------------------*/

int main(void) {
  // power on state
  robot_state = kPowerOn;

  //设置系统中断优先级分组4
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

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
  // StateCheck task
  xTaskCreate((TaskFunction_t)StateCheck_task, (const char*)"StateCheck_task",
              (uint16_t)StateCheck_STK_SIZE, (void*)NULL,
              (UBaseType_t)StateCheck_TASK_PRIO,
              (TaskHandle_t*)&StateCheckTask_Handler);

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

    if (adcx < kBatVoltTHR) {
      // low voltage error.
      err_code |= 0x01;

    } else {
      // clear the error
      err_code &= 0xFE;
    }

    vTaskDelay(100);
  }
}

// Robot task
static void Robot_task(void* pvParameters) {
  u8 state;

  // robot class
  robot cwr;
  RobotNew(&cwr);
  cwr.Init();

  // ready state
  robot_state = kReady;

  while (1) {
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
      } else if (cwr.mode_ == kManualAuto) {
        // auto mode
        state = cwr.Auto(cwr.cmd_speed_, cwr.dir_, cwr.cycle_, &state);
      }
    } else if (usRegHoldingBuf[3] == 0) {
      // stop
      { cwr.Manual(0, cwr.dir_); }
    }

    usRegHoldingBuf[16] = state;

    // // LED indicator
    // if (state == kIdle || state == kMotion && ) {
    //   LED2 = 1;
    // }

    // 100ms刷新一次
    vTaskDelay(100);
  }
}

// Modbus task
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
static void StateCheck_task(void* pvParameters) {
  _Bool err;
  while (1) {
    err = ErrCheck(&err_code);
    if (err == 1) {
      // error
      robot_state = kAlarm;
      RobotAlarmEnable();
    } else {
      RobotAlarmDisable();
      LED2=1;
    }

    // enable state check
    if (usRegHoldingBuf[3] == 1) {
      robot_state = kEnabled;
    } else {
      robot_state = kReady;
    }

    // robot state
    usRegHoldingBuf[4] = robot_state;
    usRegHoldingBuf[5] = err_code;
    


    vTaskDelay(100);  // Delay期间任务被BLOCK，可以执行其他任务
  }
}
