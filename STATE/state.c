// state.c
// CWR_SZ project

// Created by Song Junlin on 4/12/2019
// Copyright 2019 IRIM, Inc. All right reserved.
//
// const u8 kSkid;

#include "state.h"

static const _Bool kTure = 1;
static const _Bool kFalse = 0;

_Bool ErrCheck(const u8 *err_code) {
  _Bool err;
  if ((*err_code & 0x0F )== 0x00) {
    // no error
    err = kFalse;
  } else {
    err = kTure;
  }
  return err;
}