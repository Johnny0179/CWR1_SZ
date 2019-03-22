

#include "robot.h"

robot* new_robot(void) {
  robot* rObj;
  // Initializing interface for access to functions
  rObj->init = Robot_Init;
}