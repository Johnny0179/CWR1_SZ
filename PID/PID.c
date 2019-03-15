/*This file has been prepared for Doxygen automatic documentation generation.*/
#include "pid.h"

void pid_Init(uint32_t p_factor, uint32_t i_factor, uint32_t d_factor,
              struct PID_DATA *pid)

// Set up PID controller parameters
{
  // Start values for PID controller
  pid->sumError = 0;
  pid->lastProcessValue = 0;
  // Tuning constants for PID loop
  pid->P_Factor = p_factor;
  pid->I_Factor = i_factor;
  pid->D_Factor = d_factor;
  // Limits to avoid overflow
  pid->maxError = MAX_INT / (pid->P_Factor + 1);
  pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
}

int32_t pid_Controller(uint32_t setPoint, uint32_t processValue,
                       struct PID_DATA *pid_st) {
  int32_t error, p_term, d_term;
  int32_t i_term, ret, temp;

  error = setPoint - processValue;
  //误差大于2才更新控制，误差+-2
  if (error > 2 || error < -2) {
    // Calculate Pterm and limit error overflow
    if (error > pid_st->maxError) {
      p_term = MAX_INT;
    } else if (error < -pid_st->maxError) {
      p_term = -MAX_INT;
    } else {
      p_term = pid_st->P_Factor * error;
    }

    // Calculate Iterm and limit integral runaway
    temp = pid_st->sumError + error;
    if (temp > pid_st->maxSumError) {
      i_term = MAX_I_TERM;
      pid_st->sumError = pid_st->maxSumError;
    } else if (temp < -pid_st->maxSumError) {
      i_term = -MAX_I_TERM;
      pid_st->sumError = -pid_st->maxSumError;
    } else {
      pid_st->sumError = temp;
      i_term = pid_st->I_Factor * pid_st->sumError;
    }

    // Calculate Dterm
    d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);

    pid_st->lastProcessValue = processValue;

    ret = (p_term + i_term + d_term) / SCALING_FACTOR;
    if (ret > MAX_INT) {
      ret = MAX_INT;
    } else if (ret < -MAX_INT) {
      ret = -MAX_INT;
    }

    return ((int32_t)ret);
  } else
    return 0;
}

void pid_Reset_Integrator(pidData_t *pid_st) { pid_st->sumError = 0; }
