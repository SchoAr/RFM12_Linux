/*
 * SPI_Wrapper.c
 *
 *  Created on: 28.12.2014
 *      Author: armetallica
 */
#include "SPI_Wrapper.h"
#include "xspips.h"		/* SPI device driver */
#include <stdio.h>

#define SPI_DEVICE_ID		XPAR_XSPIPS_0_DEVICE_ID
#define RFM12_SPI_SELECT	0

XSpiPs_Config *SpiConfig;
static XSpiPs SpiInstance;

void init_spi(void) {
	int Status;
	/*
	 * Initialize the SPI driver so that it's ready to use
	 */
	SpiConfig = XSpiPs_LookupConfig(SPI_DEVICE_ID);
	if (NULL == SpiConfig) {
		printf("[SPI] ERROR INITIALIZE SPI DRVIER\n");
	}
	Status = XSpiPs_CfgInitialize(&SpiInstance, SpiConfig,
			SpiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		printf("[SPI] ERROR LOADING CONFIG\n");
	}
	/*
	 * Perform a self-test to check hardware build
	 */
	Status = XSpiPs_SelfTest(&SpiInstance);
	if (Status != XST_SUCCESS) {
		printf("[SPI] ERROR HARDWARE SELFTEST");
	}
	/*
	 * Set the SPI device as a master with manual start and manual
	 * chip select mode options
	 */
	XSpiPs_SetOptions(&SpiInstance, XSPIPS_MANUAL_START_OPTION |
	XSPIPS_MASTER_OPTION | XSPIPS_FORCE_SSELECT_OPTION);

	/*
	 * Set the SPI device pre-scalar to divide by 8
	 */
	XSpiPs_SetClkPrescaler(&SpiInstance, XSPIPS_CLK_PRESCALE_128);

	Status = XSpiPs_SetSlaveSelect(&SpiInstance, RFM12_SPI_SELECT);
	if (Status != XST_SUCCESS) {
		printf("[SPI] ERROR SECLECTING THE SLAVE");
	}
}

u16 xfer(u16 cmd) {

	u16 ret;
	u8 TempBufferSend[2];
	u8 TempBufferReceiv[2];
	TempBufferSend[0] = cmd >> 8;
	TempBufferSend[1] = cmd & 0xFF;

	ret = XSpiPs_PolledTransfer(&SpiInstance, TempBufferSend, TempBufferReceiv,
			2);
	if (ret != XST_SUCCESS) {
		printf("[SPI] Error win write_16\n");
	}

	ret =  (TempBufferReceiv[0]<<8) | TempBufferReceiv[1];
	return ret;
}

u8 byte(u8 cmd) {
	int ret;
	u8 TempBufferSend[1];
	u8 TempBufferReceiv[1];
	TempBufferSend[0] = cmd;
	ret = XSpiPs_PolledTransfer(&SpiInstance, TempBufferSend, TempBufferReceiv,1);
	if (ret != XST_SUCCESS) {
		printf("[SPI] Error win write_8\n");
	}

	return TempBufferReceiv[0];
}

u16 writeCmd(u16 cmd) {


	int ret;
	u8 TempBufferSend[2];
	u8 TempBufferReceiv[2];
	TempBufferSend[0] = cmd >> 8;
	TempBufferSend[1] = cmd & 0xFF;

	restart:
	ret = XSpiPs_PolledTransfer(&SpiInstance, TempBufferSend, TempBufferReceiv,
			2);
	if (ret != XST_SUCCESS) {
		printf("[SPI] Error in writeCmd 0x%x \n",cmd);
		goto restart;
	}

	return TempBufferReceiv[0] + TempBufferReceiv[1];
}

