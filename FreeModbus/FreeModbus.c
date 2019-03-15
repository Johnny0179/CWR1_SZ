#include "FreeModbus.h"

/* -------------------------Registors--------------------------------*/

// InputBuf
uint16_t usRegInputBuf[REG_INPUT_NREGS] = {
    0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007,
    0x0008, 0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000E, 0x000F};

uint16_t usRegInputStart = REG_INPUT_START;

// HoldingBuf, Robot State
int16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {
    2,      0,      0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

uint16_t usRegHoldingStart = REG_HOLDING_START;

// CoilsBuf
uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = {0x00, 0x00};

uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8] = {0x00, 0x00};

// InputCB Function
eMBErrorCode eMBRegInputCB(UCHAR* pucRegBuffer, USHORT usAddress,
                           USHORT usNRegs) {
  eMBErrorCode eStatus = MB_ENOERR;
  int16_t iRegIndex;

  //??????ڼĴ????Χ?
  //Ϊ?˱?⾯?棬???????????
  if (((int16_t)usAddress >= REG_INPUT_START) &&
      (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS)) {
    //?????ƫ????????β???ʼ???-???Ĵ???ĳ?ʼ???
    iRegIndex = (int16_t)(usAddress - usRegInputStart);
    //?????ֵ
    while (usNRegs > 0) {
      //??ֵ??ֽ?
      *pucRegBuffer++ = (uint8_t)(usRegInputBuf[iRegIndex] >> 8);
      //??ֵ??ֽ?
      *pucRegBuffer++ = (uint8_t)(usRegInputBuf[iRegIndex] & 0xFF);
      //ƫ??????
      iRegIndex++;
      //??????Ĵ??????ݼ?
      usNRegs--;
    }
  } else {
    //???ش???̬????Ĵ??
    eStatus = MB_ENOREG;
  }

  return eStatus;
}

// Holding Registor function
eMBErrorCode eMBRegHoldingCB(UCHAR* pucRegBuffer, USHORT usAddress,
                             USHORT usNRegs, eMBRegisterMode eMode) {
  //????̬
  eMBErrorCode eStatus = MB_ENOERR;
  //ƫ???
  int16_t iRegIndex;

  //??ϼĴ?????????Χ?
  if (((int16_t)usAddress >= REG_HOLDING_START) &&
      (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)) {
    //???????
    iRegIndex = (int16_t)(usAddress - usRegHoldingStart);

    switch (eMode) {
      //????????
      case MB_REG_READ:
        while (usNRegs > 0) {
          *pucRegBuffer++ = (uint8_t)(usRegHoldingBuf[iRegIndex] >> 8);
          *pucRegBuffer++ = (uint8_t)(usRegHoldingBuf[iRegIndex] & 0xFF);
          iRegIndex++;
          usNRegs--;
        }
        break;

      //д??????
      case MB_REG_WRITE:
        while (usNRegs > 0) {
          usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
          usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
          iRegIndex++;
          usNRegs--;
        }
        break;
    }
  } else {
    //???ش???̬
    eStatus = MB_ENOREG;
  }

  return eStatus;
}

// Coil register Function
eMBErrorCode eMBRegCoilsCB(UCHAR* pucRegBuffer, USHORT usAddress,
                           USHORT usNCoils, eMBRegisterMode eMode) {
  //????̬
  eMBErrorCode eStatus = MB_ENOERR;
  //?Ĵ?????
  int16_t iNCoils = (int16_t)usNCoils;
  //?Ĵ??ƫ???
  int16_t usBitOffset;

  //???Ĵ???????????Χ?
  if (((int16_t)usAddress >= REG_COILS_START) &&
      (usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE)) {
    //??????ƫ???
    usBitOffset = (int16_t)(usAddress - REG_COILS_START);
    switch (eMode) {
      //?????
      case MB_REG_READ:
        while (iNCoils > 0) {
          *pucRegBuffer++ = xMBUtilGetBits(
              ucRegCoilsBuf, usBitOffset, (uint8_t)(iNCoils > 8 ? 8 : iNCoils));
          iNCoils -= 8;
          usBitOffset += 8;
        }
        break;

      //д???
      case MB_REG_WRITE:
        while (iNCoils > 0) {
          xMBUtilSetBits(ucRegCoilsBuf, usBitOffset,
                         (uint8_t)(iNCoils > 8 ? 8 : iNCoils), *pucRegBuffer++);
          iNCoils -= 8;
        }
        break;
    }

  } else {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}

// Discrete Function
eMBErrorCode eMBRegDiscreteCB(UCHAR* pucRegBuffer, USHORT usAddress,
                              USHORT usNDiscrete) {
  //????̬
  eMBErrorCode eStatus = MB_ENOERR;
  //????Ĵ?????
  int16_t iNDiscrete = (int16_t)usNDiscrete;
  //ƫ???
  uint16_t usBitOffset;

  //??ϼĴ??ʱ???ƶ???Χ?
  if (((int16_t)usAddress >= REG_DISCRETE_START) &&
      (usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE)) {
    //??ƫ???
    usBitOffset = (uint16_t)(usAddress - REG_DISCRETE_START);

    while (iNDiscrete > 0) {
      *pucRegBuffer++ =
          xMBUtilGetBits(ucRegDiscreteBuf, usBitOffset,
                         (uint8_t)(iNDiscrete > 8 ? 8 : iNDiscrete));
      iNDiscrete -= 8;
      usBitOffset += 8;
    }

  } else {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}