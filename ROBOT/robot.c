#include "led.h"

#include "robot.h"

static void RobotInit(void) { 
	LED1 = 0; }

void robot_new(robot *r) {
  r->no_ = 0;
  r->init = RobotInit;
}