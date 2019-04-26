#include "FreeModbus.h"
#include "PID.h"
#include "motor.h"
#include "timer.h"

extern uint32_t usRegHoldingBuf[REG_HOLDING_NREGS];
u32 delta_turn[MotorNum];
u32 MotorSpeed[MotorNum];

/*constant prarameters*/

const _Bool kTrue = 1;
const _Bool kFalse = 0;

/*ms*/
const u8 kTimeInteval = 50;
const float kPI = 3.14;
// diameter
const u8 kDiameter = 45;
// reduction ratio
const u8 kReductionRatio = 43;
/*motor turns*/
static volatile u32 motor_turn_this_time[MotorNum] = {0};
static volatile u32 motor_turn_last_time[MotorNum] = {0};
/*motor dir*/
static volatile u8 motor_dir_last_time = 0;
static volatile u8 motor_dir_this_time = 0;

// default up
static volatile _Bool dir_last_time = 0;
static volatile _Bool dir_this_time;
// retain the direction when stop the motor
static volatile _Bool dir_stop;

static volatile _Bool dir_change;
static volatile u32 stop_counter;
const u16 kStopTime = 500;

// acceleration phase parameters, default 0.5s
const u32 kAccelerationPhaseTime = 500;
const u32 kAccelerationStageNum = 50;

const u32 kDecelerationPhaseTime = 500;
const u32 kDecelerationStageNum = 50;

volatile u32 cmd_speed_last_time;
volatile u32 cmd_speed_this_time;
volatile u32 constant_speed;
static volatile u8 motor_state = 0;
volatile u8 stage_num = 0;

// motor control state
const u8 kInMotion = 0;
const u8 kStepDone = 1;
const u8 kCycleDone = 2;

// motor state
static const u8 kStopState = 0;
static const u8 kAccelerationState = 1;
static const u8 kConstantState = 2;
static const u8 kDecelerationState = 3;

extern struct PID_DATA pid_acc;

// odometer
u32 odometer[MotorNum] = {0};

void MotorInit(void)
{
  // Freq=20k,DutyRatio=50
  TIM1_PWM_Init(PWMfreq);
  TIM8_PWM_Init(PWMfreq);

  Motor_Init();

  // Capture Init

  MotorFGInit();

  /*timer counter init*/
  Tim4IntInit(2 * (kAccelerationPhaseTime / kAccelerationStageNum) * 10 - 1,
              8400 - 1);
  Tim5IntInit(kTimeInteval * 10 - 1, 8400 - 1);

  // error indicator
  Tim13IntInit(500 * 10, 8400 - 1);
}

// motor motion state const
static const u8 kMotorNormal = 0;
static const u8 kMotorStall = 1;

u8 MotorCtrlManual(struct MOTOR_DATA *motor, struct PID_DATA *pid,
                   const u32 *cmd_speed, _Bool dir)
{
  u8 state;
  cmd_speed_this_time = *cmd_speed;

  motor->direction = dir;

  // motor speed feedback
  motor->MotorSpeed_mmps = MotorVelCalc(delta_turn[motor->num - 1]);

  // check the state
  motor_state = StateCheck(motor_state, cmd_speed_this_time,
                           cmd_speed_last_time, motor->MotorSpeed_mmps);

  if (motor_state == kConstantState)
  {
    // save the initial decelerate speed
    constant_speed = *cmd_speed;
  }

  if (motor_state == kDecelerationState)
  {
    motor->CmdSpeed = SetSpeed(motor_state, constant_speed);
  }
  else
  {
    // set the motor speed
    motor->CmdSpeed = SetSpeed(motor_state, *cmd_speed);
  }

  usRegHoldingBuf[33] = motor_state;
  usRegHoldingBuf[34] = motor->CmdSpeed;
  usRegHoldingBuf[35] = *cmd_speed;

  // update cmd speed
  cmd_speed_last_time = *cmd_speed;

  // update dirction
  dir_last_time = dir;

  // acceleration down
  if (motor_state == kAccelerationState /* && dir == movedown */)
  {
    // scale the pwm 2 times
    motor->PWM = motor->PWM +
                 pid_Controller(motor->CmdSpeed, motor->MotorSpeed_mmps, &pid_acc) / 2;
  }
  else
  {
    motor->PWM = motor->PWM +
                 pid_Controller(motor->CmdSpeed, motor->MotorSpeed_mmps, pid) / 2;
  }

  // PWM 0~100%
  if (motor->PWM < 0)
  {
    motor->PWM = 0;
  }
  else if (motor->PWM > 100)
  {
    // Max motor->PWM Ratio
    motor->PWM = 100;
  }
  else if (motor->CmdSpeed == 0)
  {
    motor->PWM = 0;
  }

  /*mannual mode*/
  if (motor->mode == 0)
  {
    // Direction
    switch (motor->direction)
    {
    case moveup:
      MoveUp();
      break;
    case movedown:
      MoveDown();
      break;
    case stop:;
    default:;
    }
  }

  // Power out put control
  switch (motor->num)
  {
  case 1:
    TIM1->CCR4 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
    break;
  case 2:
    TIM1->CCR3 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
    break;
  case 3:
    TIM1->CCR2 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
    break;
  case 4:
    TIM1->CCR1 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
    break;
  case 5:
    TIM8->CCR1 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
    break;
  case 6:
    TIM8->CCR2 = (((168000000 / 1) / PWMfreq) - 1) * (100 - motor->PWM) / 100;
    break;
  default:
    break;
  }

  // motor stall detection
  if (motor_state == kConstantState && motor->MotorSpeed_mmps == 0)
  {
    state = kMotorStall;
  }

  return state;
}

u8 MotorCtrlAuto(struct MOTOR_DATA *motor, struct PID_DATA *pid,
                 const u32 *cmd_speed, _Bool init_dir, u8 cycle, u8 *state) {}

void MotorInitConfig(u8 num, struct MOTOR_DATA *motor)
{
  motor->num = num;
  // up
  motor->direction = 0;
  motor->CmdSpeed = 0;
  motor->PWM = 0;
  // manual
  motor->mode = 0;
  motor->MotorSpeed_mmps = 0;
}

u32 MotorVelCalc(u32 delta_turn)
{
  // 6 pulses per turn
  return (delta_turn * kPI * kDiameter * 1000) /
         (6 * kReductionRatio * kTimeInteval);
}

u32 DeltaTurnCalc(u32 *motor_turn, u8 motor_num)
{
  u8 i = 0;

  for (i = 0; i <= motor_num - 1; i++)
  {
    // protect full
    if (motor_turn[i] < 0x01FF)
    {
      motor_turn_this_time[i] = motor_turn[i];
      delta_turn[i] = motor_turn_this_time[i] - motor_turn_last_time[i];
      // update
      motor_turn_last_time[i] = motor_turn_this_time[i];
    }
    else
    {
      // odometer
      odometer[i] = odometer[i] + (motor_turn[i] * kPI * kDiameter) /
                                      (6 * kReductionRatio * 10);
      motor_turn_this_time[i] = motor_turn[i];
      delta_turn[i] = motor_turn_this_time[i] - motor_turn_last_time[i];
      // update, reset the counter
      motor_turn_last_time[i] = 0;
      // empty the turn counter
      motor_turn[i] = 0;
    }
  }
  usRegHoldingBuf[31] = odometer[0];
  usRegHoldingBuf[32] = motor_turn[0] / 6;
}

u32 MotorSetCmdSpeed(u32 cmd_speed, u32 motor_feedback_speed)
{
  /*   u32 motor_cmd_speed;

  if (abs(cmd_speed - motor_feedback_speed) > 100)
  {
    // stop
    if (cmd_speed == 0)
    {
      motor_cmd_speed = abs(cmd_speed - motor_feedback_speed) * 0.8;
    }
    else
    {
      motor_cmd_speed = 0.8 * cmd_speed;
    }
  }
  else
  {
    motor_cmd_speed = cmd_speed;
  }

  // motor_cmd_speed=cmd_speed;
  return motor_cmd_speed; */
}

u32 Acceleration(u8 N, u32 cmd_speed)
{
  u32 speed_temp;
  speed_temp = N * cmd_speed / kAccelerationStageNum;
  if (speed_temp <= cmd_speed)
  {
    return speed_temp;
  }
  else
  {
    return cmd_speed;
  }
}

u32 Deceleration(u8 N, u32 cmd_speed)
{
  return cmd_speed - (N * cmd_speed / kDecelerationStageNum);
}

u8 StateCheck(u8 state, u32 this_time, u32 last_time, u32 motor_speed)
{
  switch (state)
  {
  case kStopState:
    if (this_time == 0)
    {
      state = kStopState;
      stage_num = 0;
    }
    else
    {
      // change to acceleration
      state = kAccelerationState;
      Tim4Enable();
    }
    break;

  case kAccelerationState:
    if (stage_num < kAccelerationStageNum)
    {
      state = kAccelerationState;
    }
    // have not been speed up, return to stop state.
    else if (motor_speed == 0)
    {
      state = kStopState;
      Tim4Disable();
    }
    // change to the constantState
    else
    {
      state = kConstantState;
      Tim4Disable();
      stage_num = 0;
    }
    break;

  case kConstantState:
    if (this_time == last_time)
    {
      // stay in constant state
      state = kConstantState;
    }
    else if (this_time < last_time || this_time == 0)
    {
      // change to deceleration state
      state = kDecelerationState;
      Tim4Enable();
    }
    break;

  case kDecelerationState:
    if (stage_num < kDecelerationStageNum)
    {
      // stay in the deceleration state
      state = kDecelerationState;
    }
    else
    {
      // change to stop state
      state = kStopState;
      Tim4Disable();
      stage_num = 0;
    }
    break;

  default:
    state = kStopState;
    break;
  }

  return state;
}

u32 SetSpeed(u8 state, u32 cmd_speed)
{
  u32 set_motor_speed;
  usRegHoldingBuf[36] = stage_num;
  switch (state)
  {
  case 0:
    set_motor_speed = 0;
    break;
  case 1:
    set_motor_speed = Acceleration(stage_num, cmd_speed);
    break;
  case 2:
    set_motor_speed = cmd_speed;
    break;
  case 3:
    set_motor_speed = Deceleration(stage_num + 1, cmd_speed);
    break;
  default:
    set_motor_speed = 0;
    break;
  }
  return set_motor_speed;
}