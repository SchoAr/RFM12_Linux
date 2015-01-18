/*
 * userLed.c
 *
 *  Created on: 24.11.2014
 *      Author: SchoAr
 */
/*
 * ConfigPtr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
 * ret = XGpioPs_CfgInitialize(&gpio, ConfigPtr,ConfigPtr->BaseAddr);
 */
#include "Led.h"

static XGpioPs* gpio ;
static XGpioPs_Config *ConfigPtr;

void init_Led(int pin,XGpioPs* gpiof,XGpioPs_Config *ConfigPtrf){

	gpio = gpiof;
	ConfigPtr = ConfigPtrf;
	XGpioPs_SetDirectionPin(gpio,pin, 1);
	XGpioPs_SetOutputEnablePin(gpio,pin, 1);
}

void toggle_Led(int pin){
	u32 oldvalue;
	oldvalue = XGpioPs_ReadPin(gpio,pin);

	if(oldvalue == 0){
		oldvalue = 1;
	}else{
		oldvalue = 0;
	}
	XGpioPs_WritePin(gpio,pin,oldvalue);
}

void on_Led(int pin){
	XGpioPs_WritePin(gpio,pin,1);
}

void off_Led(int pin){
	XGpioPs_WritePin(gpio,pin,0);
}
