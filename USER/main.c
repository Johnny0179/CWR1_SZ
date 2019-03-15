/*-------------------------------Includes-------------------------------*/
/*STM32*/
#include "Motor.h"
#include "delay.h"
#include "led.h"
#include "sys.h"
#include "timer.h"

/*PID*/
#include "PID.h"

/*FreeRTOS*/
#include "FreeRTOS.h"
#include "task.h"

/*FreeModbus*/
#include "FreeModbus.h"

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
#define Modbus_STK_SIZE 50
//任务句柄
TaskHandle_t ModbusTask_Handler;
//任务函数
void Modbus_task(void* pvParameters);

//任务优先级
#define Motor_TASK_PRIO 4
//任务堆栈大小
#define Motor_STK_SIZE 50
//任务句柄
TaskHandle_t MotorTask_Handler;
//任务函数
void Motor_task(void* pvParameters);

/*----------------------------Configuration----------------------------------*/

// Robot registors
extern int32_t usRegHoldingBuf[REG_HOLDING_NREGS];
extern int32_t MotorSpeed[4];
u8 MoveInteval = 5;

const u8 RefreshRate = 50;

// CAN
extern u8 canbuf[8];

/*----------------------------Start Implemention-------------------------*/

int main(void) {
  //设置系统中断优先级分组4
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  //初始化延时函数
  delay_init(168);

  //初始化LED端口
  LED_Init();

  // Motor Init
  MotorInit();

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
  taskENTER_CRITICAL();  //进入临界区
  //创建CAN任务
  xTaskCreate((TaskFunction_t)CAN_task, (const char*)"CAN_task",
              (uint16_t)CAN_STK_SIZE, (void*)NULL, (UBaseType_t)CAN_TASK_PRIO,
              (TaskHandle_t*)&CANTask_Handler);
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

// Modbus任务函数
static void Modbus_task(void* pvParameters) {
  // Modbus Init
  eMBInit(MB_RTU, 0x0A, 0x01, 19200, MB_PAR_NONE);
  eMBEnable();
  while (1) {
    eMBPoll();
    // 20ms刷新一次，Delay期间任务被BLOCK，可以执行其他任务
    vTaskDelay(RefreshRate);
  }
}

// Motor任务
static void Motor_task(void* pvParameters) {
  /*Motor*/
  // Motor Define
  MOTOR Motor1;
  MOTOR Motor2;
  MOTOR Motor3;
  // MOTOR Motor4;

  // PID Define
  pidData_t PIDMotor1;
  pidData_t PIDMotor2;
  pidData_t PIDMotor3;
  // pidData_t PIDMotor4;

  MotorInitConfig(1, &Motor1);
  MotorInitConfig(2, &Motor2);
  MotorInitConfig(3, &Motor3);

  // PID Init
  pid_Init(P, I, D, &PIDMotor1);
  pid_Init(P, I, D, &PIDMotor2);
  pid_Init(P, I, D, &PIDMotor3);

  while (1) {
    // usRegHoldingBuf[0]=Motor1.direction;
    MotorCtrl(&Motor1, &PIDMotor1);
    usRegHoldingBuf[3] = Motor1.MotorSpeed_mmps;

    MotorCtrl(&Motor2, &PIDMotor2);
    usRegHoldingBuf[4] = Motor2.MotorSpeed_mmps;

    MotorCtrl(&Motor3, &PIDMotor3);
    usRegHoldingBuf[5] = Motor3.MotorSpeed_mmps;

    /*Debug*/
    usRegHoldingBuf[7] = Motor3.PWM;
    usRegHoldingBuf[8] = Motor3.CmdSpeed;
    // MotorCtrl(Motor4,PIDMotor4);
    vTaskDelay(10);  // Delay期间任务被BLOCK，可以执行其他任务
  }
}
