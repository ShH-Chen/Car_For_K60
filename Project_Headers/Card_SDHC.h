/*
 * Card_SDHC.h
 *
 *  Created on: Apr 25, 2014
 *      Author: Administrator
 */

#ifndef CARD_SDHC_H_
#define CARD_SDHC_H_
#include "Cpu.h"
#include "Events.h"
#include "SDHC.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PORT_PDD.h"
#include "SDHC_PDD.h"
#include <stdio.h>
#include <string.h>
//#include "FML_DiskIO.h"
#define SDHC_DAT0_PORT_INDEX      1U//E1
#define SDHC_DAT1_PORT_INDEX      0U//E0
#define SDHC_DAT2_PORT_INDEX      5U//E5
#define SDHC_DAT3_PORT_INDEX      4U//E4
typedef struct {
  LDD_TDeviceData *SDHCPtr;     /* SDHC component data */
  LDD_TDeviceData *GPIOPtr;     /* GPIO component data */
  bool Finished;                /* Operation end indication */
  bool Inserted;                /* Card insertion indication */
  uint8_t CardId;               /* Initialized card ID */
  LDD_SDHC_TCardInfo CardInfo;  /* Initialized card info */
} TSDData;


/* ���̺������ؽ��ö������ */
typedef enum {
  RES_OK = 0,		/* 0: �ɹ� */
  RES_ERROR,		/* 1: ��/д���� */
  RES_WRPRT,		/* 2: д���� */
  RES_NOTRDY,		/* 3: δ׼���� */
  RES_PARERR,		/* 4: �������� */
  RES_NONRSPNS      /* 5: δ��Ӧ */
} DRESULT;
#define STA_NOINIT		0x01	/* Drive not initialized */
/////////////////////////////////////////////////////////////
//���̶���
////////////////////////////////////////////////////////////
#define FATM1_BLOCK_SIZE   512            /* user defined block size */
///////////////////////////////////////////////////////////////
#define SD_BLOCK_SIZE   512            /* user defined block size */
LDD_TError write_Blocks(uint32_t Address, uint16_t Size,uint16_t BufferDescCount,uint8_t  *DataPtr);
LDD_TError read_Blocks(uint32_t Address, uint16_t Size,uint16_t BufferDescCount,uint8_t  *DataPtr);
LDD_TError read_mul_Blocks(uint32_t sector,uint16_t BufferDescCount,uint8_t  *DataPtr);
LDD_TError write_mul_Blocks(uint32_t sector,uint16_t BufferDescCount,uint8_t  *DataPtr);
void read_data();
void SD_Init();
void SD_Read(TSDData *SD);
void SD_Wait();
///////////////////////////////////////////////////////////////
#endif /* CARD_SDHC_H_ */
