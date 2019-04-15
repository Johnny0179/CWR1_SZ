// state.h
// CWR_SZ project

// Created by Song Junlin on 4/21/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//

#ifndef _STATE_H_
#define _STATE_H_

#include "robot.h"

_Bool ErrCheck(const u8 *err_code);

u8 RobotStateCheck(_Bool err,u8 *robot_state);
u8 ErrCodeSet(u8 err, u8 *err_code);
u8 ErrCodeClear(u8 err, u8 *err_code);

#endif