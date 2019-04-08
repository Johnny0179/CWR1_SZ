/*This file has been prepared for Doxygen automatic documentation generation.*/
#ifndef PID_H
#define PID_H

#include "timer.h"

#define SCALING_FACTOR 1000

typedef struct PID_DATA {
  uint32_t lastProcessValue;
  uint32_t sumError;
  uint32_t P_Factor;
  uint32_t I_Factor;
  uint32_t D_Factor;
  int32_t maxError;
  int32_t maxSumError;
} pidData_t;

// Maximum value of variables
#define MAX_INT 0xFFFFFFFF   // INT32_MAX
#define MAX_LONG 0xFFFFFFFF  // INT32_MAX
#define MAX_I_TERM (MAX_LONG / 2)

// Boolean values
#define FALSE 0
#define TRUE 1

void pid_Init(uint32_t p_factor, uint32_t i_factor, uint32_t d_factor,
              struct PID_DATA *pid);
int32_t pid_Controller(uint32_t setPoint, uint32_t processValue,
                       struct PID_DATA *pid_st);
void pid_Reset_Integrator(pidData_t *pid_st);

#endif
