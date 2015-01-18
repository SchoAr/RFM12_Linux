/*
 * GPIO_Interrupt.c
 *
 *  Created on: 29.12.2014
 *      Author: armetallica
 */
#include "GPIO_Interrupt.h"
#include <stdio.h>
#include "xgpiops.h"
#include "xscugic.h"
#include "xparameters.h"

#define GPIO_INTERRUPT_ID	XPAR_XGPIOPS_0_INTR

static XScuGic* GicInstancePtr;  	/* Interrupt controller instance */
static void (*listener)(void);
static int usedBank;

static void GPIO_input_Handler(void *CallBackRef, int Bank, u32 Status)
{
	if(Bank ==1){
		listener();
	}
}


static void gpio_setupInterrupt(int pin,
								int risingFalling,
								XScuGic *GicInstancePtr,
								XGpioPs *Gpio, u16 GpioIntrId){

	XScuGic_Connect(GicInstancePtr, GpioIntrId,
			(Xil_ExceptionHandler)XGpioPs_IntrHandler,
			(void *)Gpio);

	if(risingFalling == RISNG_EDGE){
		XGpioPs_SetIntrTypePin(Gpio,pin,XGPIOPS_IRQ_TYPE_EDGE_RISING);
	}else if(risingFalling == FALLING_EDGE){
		XGpioPs_SetIntrTypePin(Gpio,pin,XGPIOPS_IRQ_TYPE_EDGE_FALLING);
	}
		XGpioPs_SetCallbackHandler(Gpio,(void *)Gpio, GPIO_input_Handler);
	XGpioPs_IntrEnablePin(Gpio,pin);
	XScuGic_Enable(GicInstancePtr, GpioIntrId);
}

void init_GPIOInterrupt(int pin,
						void (*fp)(void),
						int risingFalling,
						XGpioPs* gpiof,
						XGpioPs_Config *ConfigPtrf,
						XScuGic* GicInstancePtrf){
	listener = fp;
	usedBank = pin;
	XGpioPs_SetDirectionPin(gpiof, pin, 0);
	GicInstancePtr = GicInstancePtrf;

	/* enable the gpio interrupts */
	gpio_setupInterrupt(pin,risingFalling,GicInstancePtr,gpiof,GPIO_INTERRUPT_ID);

	/* Enable ARM interrupts */
	Xil_ExceptionEnable();
}
