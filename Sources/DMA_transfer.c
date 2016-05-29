/*
 * DMA_transfer.c
 *
 *  Created on: Feb 21, 2014
 *      Author: Administrator
 */
#include "Cpu.h"
#include "Events.h"
#include "DMAT1.h"
#include "DMA1.h"

/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "DMA_transfer.h"

LDD_TDeviceData *DMA_DeviceData=NULL;
LDD_TError       DMA__TError;

void Init_DMA()
{
	
	PORTE_PCR6|=PORT_PCR_PE_MASK|PORT_PCR_PFE_MASK;
	//PORTE_PCR6|=PORT_PCR_PE_MASK|PORT_PCR_SRE_MASK;
	DMA_DeviceData=DMAT1_Init(NULL);
	DMA__TError=DMAT1_AllocateChannel(DMA_DeviceData);
	DMA__TError=DMAT1_EnableChannel(DMA_DeviceData);
	
}
