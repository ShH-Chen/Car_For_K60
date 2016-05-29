/*
 * Card_SDHC.c
 *
 *  Created on: Apr 25, 2014
 *      Author: Administrator
 */
#include"Card_SDHC.h"
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

////////////////////////////////////////////////////////////////////////////////////////////////
LDD_TError  SD_Error;
LDD_SDHC_TCardInfo SD_InfoPtr;
TSDData  SD;

bool Inserted_flag=FALSE;
bool Removed_flag=FALSE;
bool Finished_flag=FALSE;
 

/////////////////////////////////////
 //static volatile DSTATUS  Stat = STA_NOINIT; /* Disk status */
////////////////////////////////////
 void SD_Wait()
{
	word i=65535;
	while(i>2)i--;
	i=65535;
	while(i>2)i--;
	i=65535;
		while(i>2)i--;
}

void SD_Init()
{
	  /* Init application data */
	  SD.SDHCPtr = 0;
	  SD.GPIOPtr = 0;
	  SD.Finished = FALSE;
	  SD.Inserted = FALSE;
	  SD.CardId = SDHC_NO_CARD;

	  /* Enable pull-up on GPIO pin used for SD card detection */
	    
	 /* Init components/devices */
	  //  SD.GPIOPtr = GPI0_Init(&SD);
	    PORT_PDD_SetPinPullSelect(PORTE_BASE_PTR, 4, PORT_PDD_PULL_UP );
	    PORT_PDD_SetPinPullEnable(PORTE_BASE_PTR, 4, PORT_PDD_PULL_ENABLE);
	  //  SD.Inserted = GPI0_GetVal(SD.GPIOPtr);
	    /* Wait until a card is inserted */
	  /*  while (!SD.Inserted) 
	    {
	    	SD.Inserted = GPI0_GetVal(SD.GPIOPtr) ;
	    };*/
	    /*检测到SD卡。开始初始化SD卡*/
	    SD.SDHCPtr = SDHC_Init(&SD);
	    SD_Wait(&SD);
	   // PORT_PDD_SetPinPullSelect(PORTE_BASE_PTR, 4, PORT_PDD_PULL_UP );
	   // PORT_PDD_SetPinPullEnable(PORTE_BASE_PTR, 4, PORT_PDD_PULL_ENABLE);
		Inserted_flag=FALSE;
		
		do{
		 SD_Error=SDHC_DetectCards( SD.SDHCPtr);/* Detect supported cards */	
		 SD_Wait();
		}
		while(!Inserted_flag);
		
		
		if (SD.CardId != SDHC_NO_CARD)
		{
			Removed_flag=FALSE;
			 SD_Error=SDHC_SelectCard( SD.SDHCPtr, SD.CardId );
			 SD_Wait();SD_Wait();SD_Wait();SD_Wait();SD_Wait();
			// while(!Removed_flag);
			
			 Finished_flag=FALSE;
			 SD_Error=SDHC_GetCardInfo( SD.SDHCPtr,&SD.CardInfo);
			 SD_Wait();
			 while(!Finished_flag);
			// GPI0_Deinit(SD.GPIOPtr);				//释放DTA[3]
			 /////////////////////////////////////////////////////////////////
			 //上啦电阻使能；
			 PORT_PDD_SetPinPullSelect(PORTE_BASE_PTR, SDHC_DAT0_PORT_INDEX, PORT_PDD_PULL_UP);
			 PORT_PDD_SetPinPullEnable(PORTE_BASE_PTR, SDHC_DAT0_PORT_INDEX, PORT_PDD_PULL_ENABLE);
			 PORT_PDD_SetPinPullSelect(PORTE_BASE_PTR, SDHC_DAT1_PORT_INDEX, PORT_PDD_PULL_UP);
			 PORT_PDD_SetPinPullEnable(PORTE_BASE_PTR, SDHC_DAT1_PORT_INDEX, PORT_PDD_PULL_ENABLE);
			 PORT_PDD_SetPinPullSelect(PORTE_BASE_PTR, SDHC_DAT2_PORT_INDEX, PORT_PDD_PULL_UP);
			 PORT_PDD_SetPinPullEnable(PORTE_BASE_PTR, SDHC_DAT2_PORT_INDEX, PORT_PDD_PULL_ENABLE);
			 PORT_PDD_SetPinPullSelect(PORTE_BASE_PTR, SDHC_DAT3_PORT_INDEX, PORT_PDD_PULL_UP);
			 PORT_PDD_SetPinPullEnable(PORTE_BASE_PTR, SDHC_DAT3_PORT_INDEX, PORT_PDD_PULL_ENABLE);
			 
			 /* Switch to 4-bit communication if supported by the card */
				if (SD.CardInfo.Caps.DataWidths & LDD_SDHC_CARD_DATA_WIDTH_4_BIT) {
				  SDHC_SetDataWidth(SD.SDHCPtr, LDD_SDHC_CARD_DATA_WIDTH_4_BIT);
				  SD_Wait();
				}
				SDHC_SelectBusClock(SD.SDHCPtr, SDHC_BUS_CLOCK_25MHz);
				  SD_Wait();
				  SD_Read(&SD); 					/* Reading at 16 MHz*/
				  if (SD.CardInfo.Caps.HighSpeed) {
				SDHC_SelectBusClock(SD.SDHCPtr, SDHC_BUS_CLOCK_37_5MHz);
					SD_Wait(&SD);
					SD_Read(&SD); /* Reading at 48 MHz (high slew rate on all the SDHC pins should be set) */
			 }
		}
	
}
void SD_Read(TSDData *SD)
{
  const uint16_t BlockSize = 512;
  const uint16_t BlockCount = 8;
  static uint8_t Buffer[8][512 + 4]; /* Data buffer */
  uint32_t Index;                  /* Read block index */
  uint32_t Address;                /* Read block address (in bytes) */
  uint32_t Start = 0x00044000;     /* Read start address (in bytes) */
//  LDD_SDHC_TBufferDesc BufferDescList[BlockCount];
  LDD_SDHC_TBufferDesc BufferDescList[BlockCount];
 // BufferDescList.DataPtr=&Buffer;
  
  /* Init read buffer descriptors */
 for (Index = 0; Index < BlockCount; Index++)
 {
    BufferDescList[Index].DataPtr = (uint8_t*)(((uint32_t)(Buffer[Index]) + 4) & ~0x03); /* Align buffer to 4-byte boundary */
    BufferDescList[Index].Size = BlockSize;
  }
  /* Convert address in bytes to address in blocks for high capacity cards */
  Address = Start / (SD->CardInfo.Caps.HighCapacity ? BlockSize : 1);
  /* Read card data blocks */
  SDHC_TransferBlocks(SD->SDHCPtr, LDD_SDHC_READ, Address, BufferDescList, BlockCount);
  SD_Wait(SD);
}
LDD_TError write_Blocks(uint32_t Address, uint16_t Size,uint16_t BufferDescCount,uint8_t  *DataPtr)
{
	LDD_SDHC_TBufferDesc BufferDescListPtr;
	LDD_TError  ERROR;
	BufferDescListPtr.DataPtr=DataPtr;
	BufferDescListPtr.Size=Size;
	 Finished_flag=FALSE;
	 ERROR=SDHC_TransferBlocks(SD.SDHCPtr,LDD_SDHC_WRITE,Address,&BufferDescListPtr,BufferDescCount);
	 while(!Finished_flag);
	 //SD_Wait();
	 return  ERROR;
}
LDD_TError read_Blocks(uint32_t Address, uint16_t Size,uint16_t BufferDescCount,uint8_t  *DataPtr)
{
	LDD_SDHC_TBufferDesc BufferDescListPtr;
	LDD_TError  ERROR;
	BufferDescListPtr.DataPtr=DataPtr;
	BufferDescListPtr.Size=Size;
	Finished_flag=FALSE;
	ERROR=SDHC_TransferBlocks(SD.SDHCPtr,LDD_SDHC_READ,Address,&BufferDescListPtr,BufferDescCount);
	while(!Finished_flag);
	//SD_Wait();
	return ERROR;
}

LDD_TError write_mul_Blocks(uint32_t sector,uint16_t BufferDescCount,uint8_t  *DataPtr)
{
	uint16_t count=BufferDescCount;
	uint16_t ERROR;
	if(BufferDescCount==0)   return ERR_OK;
	else 
		if(BufferDescCount==1)  return write_Blocks((uint32_t)sector*SD_BLOCK_SIZE,(uint16_t)SD_BLOCK_SIZE,1, DataPtr) ;  
		else
			if(BufferDescCount>1)
			{
				do{
					ERROR=write_Blocks((uint32_t)sector*SD_BLOCK_SIZE, SD_BLOCK_SIZE,1, DataPtr) ;
					sector++;
					DataPtr+=SD_BLOCK_SIZE;
				   }while(--count);
			}
	  return  ERROR;
}
LDD_TError read_mul_Blocks(uint32_t sector,uint16_t BufferDescCount,uint8_t  *DataPtr)
{
	uint16_t count=BufferDescCount;
	uint16_t ERROR;
	if(BufferDescCount==0)   return ERR_OK;
	else
		if(BufferDescCount==1)  return read_Blocks((uint32_t)sector*SD_BLOCK_SIZE,(uint16_t)SD_BLOCK_SIZE,1, DataPtr) ;
		else
			if(BufferDescCount>1)
			{
				do{
					ERROR=read_Blocks((uint32_t)sector*SD_BLOCK_SIZE, SD_BLOCK_SIZE,1, DataPtr) ;
					sector++;
					DataPtr+=SD_BLOCK_SIZE;
					}while(--count);
			}
	 return  ERROR;
} 
