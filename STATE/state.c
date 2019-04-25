// state.c
// CWR_SZ project

// Created by Song Junlin on 4/12/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
// const u8 kSkid;

#include "state.h"

static const _Bool kTure = 1;
static const _Bool kFalse = 0;

const u8 kPowerOn = 0;
const u8 kReady = 2;
const u8 kEnabled = 3;
const u8 kAlarm = 4;

const u8 kSkid = 0;
const u8 kLowVolt = 1;
const u8 kStall = 2;
const u8 kComFail = 3;

_Bool ErrCheck(const u8 *err_code)
{
  _Bool err;
  if ((*err_code & 0x0F) == 0x00)
  {
    // no error
    err = kFalse;
  }
  else
  {
    err = kTure;
  }
  return err;
}

u8 RobotStateCheck(_Bool err, u8 *robot_state)
{
  if ((*robot_state == kEnabled || *robot_state == kReady ||
       *robot_state == kPowerOn) &&
      err == 1)
  {
    *robot_state = kAlarm;
  }
  return *robot_state;
}

u8 ErrCodeSet(u8 err, u8 *err_code)
{
  switch (err)
  {
  case kSkid:
    *err_code |= 0x01;
    break;
  case kLowVolt:
    *err_code |= 0x02;
    break;
  case kStall:
    *err_code |= 0x04;
    break;
  case kComFail:
    *err_code |= 0x08;
    break;

  default:
    *err_code = 0;
    break;
  }
  return *err_code;
}

u8 ErrCodeClear(u8 err, u8 *err_code)
{
  switch (err)
  {
  case kSkid:
    *err_code &= 0xFE;
    break;
  case kLowVolt:
    *err_code &= 0xFD;
    break;
  case kStall:
    *err_code &= 0xFB;
    break;
  case kComFail:
    *err_code &= 0xF7;
    break;

  default:
    *err_code = 0;
    break;
  }
  return *err_code;
}
