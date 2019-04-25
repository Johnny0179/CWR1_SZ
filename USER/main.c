// main.c
// CWR_SZ project

// Created by Song Junlin on 3/21/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
/*-------------------------------Includes-------------------------------*/
/*STM32*/
#include "motor.h"
#include "adc.h"
#include "delay.h"
#include "led.h"
#include "PID.h"
#include "state.h"
#include "sys.h"
#include "timer.h"


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
void start_task(void *pvParameters);

//任务优先级
#define Modbus_TASK_PRIO 5
//任务堆栈大小
#define Modbus_STK_SIZE 256
//任务句柄
TaskHandle_t ModbusTask_Handler;
//任务函数
void Modbus_task(void *pvParameters);

//任务优先级
#define StateCheck_TASK_PRIO 4
//任务堆栈大小
#define StateCheck_STK_SIZE 256
//任务句柄
TaskHandle_t StateCheckTask_Handler;
//任务函数
void StateCheck_task(void *pvParameters);

//任务优先级
#define ADC_TASK_PRIO 2
//任务堆栈大小
#define ADC_STK_SIZE 256
//任务句柄
TaskHandle_t ADCTask_Handler;
//任务函数
void ADC_task(void *pvParameters);

#define Robot_TASK_PRIO 3
//任务堆栈大小
#define Robot_STK_SIZE 256
//任务句柄
TaskHandle_t RobotTask_Handler;
//任务函数
void Robot_task(void *pvParameters);

/*----------------------------Configuration----------------------------------*/
// extern variables
extern const u8 kInMotion;
extern const u8 kStepDone;
extern const u8 kCycleDone;
// Robot registors
extern uint32_t usRegHoldingBuf[REG_HOLDING_NREGS];
extern u32 cycle_counter;

const UCHAR kRobotAddr = 0x0A;
static const u8 kModbusRefreshRate = 100;
static const _Bool kManualMode = 0;
static const _Bool kManualAuto = 1;
// battery voltage threhold 23V
static const int32_t kBatVoltTHR = 23000;

/*----------------------------Robot state definition------------------------*/
extern const u8 kIdle;
extern const u8 kCounterCheck;
extern const u8 kChangeDir;
extern const u8 kFirstCheck;
extern const u8 kMotion;
extern const u8 kStop;
extern const u8 kWait;
extern const u8 kMotorStall;
extern const u8 kManual;
extern const u8 kAuto;

extern volatile _Bool auto_dir;
/*----------------------------variables----------------------------------*/
static u8 robot_state;
static u8 state;

static u8 err_code;
extern const u8 kPowerOn;
extern const u8 kReady;
extern const u8 kEnabled;
extern const u8 kAlarm;

/*----------------------------error code---------------------------------*/

extern const u8 kSkid;
extern const u8 kLowVolt;
extern const u8 kStall;
extern const u8 kComFail;

/*----------------------------Start Implemention-------------------------*/

int main(void)
{
  // power on state
  robot_state = kPowerOn;

  //设置系统中断优先级分组4
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  //创建开始任务
  xTaskCreate((TaskFunction_t)start_task,          //任务函数
              (const char *)"start_task",          //任务名称
              (uint16_t)START_STK_SIZE,            //任务堆栈大小
              (void *)NULL,                        //传递给任务函数的参数
              (UBaseType_t)START_TASK_PRIO,        //任务优先级
              (TaskHandle_t *)&StartTask_Handler); //任务句柄
  vTaskStartScheduler();                           //开启任务调度
  return 0;
}

/*------------------------------TASK Functions------------------------------*/

//开始任务任务函数
static void start_task(void *pvParameters)
{
  //进入临界区
  taskENTER_CRITICAL();
  //创建ADC任务
  xTaskCreate((TaskFunction_t)ADC_task, (const char *)"ADC_task",
              (uint16_t)ADC_STK_SIZE, (void *)NULL, (UBaseType_t)ADC_TASK_PRIO,
              (TaskHandle_t *)&ADCTask_Handler);
  xTaskCreate((TaskFunction_t)Robot_task, (const char *)"Robot_task",
              (uint16_t)Robot_STK_SIZE, (void *)NULL,
              (UBaseType_t)Robot_TASK_PRIO, (TaskHandle_t *)&RobotTask_Handler);
  //创建Modbus任务
  xTaskCreate((TaskFunction_t)Modbus_task, (const char *)"Modbus_task",
              (uint16_t)Modbus_STK_SIZE, (void *)NULL,
              (UBaseType_t)Modbus_TASK_PRIO,
              (TaskHandle_t *)&ModbusTask_Handler);
  // StateCheck task
  xTaskCreate((TaskFunction_t)StateCheck_task, (const char *)"StateCheck_task",
              (uint16_t)StateCheck_STK_SIZE, (void *)NULL,
              (UBaseType_t)StateCheck_TASK_PRIO,
              (TaskHandle_t *)&StateCheckTask_Handler);

  vTaskDelete(StartTask_Handler); //删除开始任务
  taskEXIT_CRITICAL();            //退出临界区
}

// ADC任务函数
static void ADC_task(void *pvParameters)
{
  u16 adcx;
  float temp;
  Adc_Init();
  while (1)
  {
    adcx = Get_Adc_Average(ADC_Channel_5, 20);
    // mv
    temp = (float)adcx * (3.3 / 4096) * 12000;
    adcx = temp;
    usRegHoldingBuf[20] = adcx;

    if (adcx < kBatVoltTHR && (state == kIdle || state == kWait))
    {
      // low voltage error.
      err_code = ErrCodeSet(kLowVolt, &err_code);
    }
    else
    {
      err_code = ErrCodeClear(kLowVolt, &err_code);
    }

    vTaskDelay(100);
  }
}

// Robot task
static void Robot_task(void *pvParameters)
{
  // robot class
  robot cwr;
  RobotNew(&cwr);
  cwr.Init();
  cwr.Enable();
  // ready state
  robot_state = kReady;

  while (1)
  {
    cwr.dir_ = usRegHoldingBuf[2];
    cwr.mode_ = usRegHoldingBuf[21];
    cwr.cycle_ = usRegHoldingBuf[22];
    cwr.cmd_speed_ = usRegHoldingBuf[23];

    // reset
    if (usRegHoldingBuf[9] == 1)
    {
      cwr.Reset();
    }

    if (state != kMotorStall)
    {
      state = cwr.Auto(cwr.cmd_speed_, cwr.dir_, cwr.cycle_, &state);
    }
    else
    {
      // stall error
      err_code = ErrCodeSet(kStall, &err_code);

      // sotp
      cwr.Manual(0, 0);
    }

    usRegHoldingBuf[16] = state;

    vTaskDelay(100);
  }
}

// Modbus task
static void Modbus_task(void *pvParameters)
{
  // Modbus Init
  eMBInit(MB_RTU, kRobotAddr, 0x01, 19200, MB_PAR_NONE);
  eMBEnable();
  while (1)
  {
    // communication error check.
    if (usRegHoldingBuf[7] != 1)
    {
      err_code = ErrCodeSet(kComFail, &err_code);
    }
    else if (usRegHoldingBuf[7] == 1)
    {
      err_code = ErrCodeClear(kComFail, &err_code);
    }
    eMBPoll();
    vTaskDelay(kModbusRefreshRate);
  }
}

// RobotState task
static void StateCheck_task(void *pvParameters)
{
  _Bool err;
  while (1)
  {
    err = ErrCheck(&err_code);
    if (err == 1)
    {
      // error
      robot_state = kAlarm;
      RobotAlarmEnable();
    }
    else
    {
      RobotAlarmDisable();
      LED2 = 1;
    }

    usRegHoldingBuf[5] = err_code;

    // enable state check
    if (usRegHoldingBuf[3] == 1)
    {
      robot_state = kEnabled;
    }
    else
    {
      robot_state = kReady;
    }

    // robot state
    usRegHoldingBuf[4] = RobotStateCheck(err, &robot_state);
    vTaskDelay(100);
  }
}