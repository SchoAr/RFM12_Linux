/*
 * input.c
 *
 *  Created on: 27.11.2014
 *      Author: SchoAr
 */
#include "xgpiops.h"
#include <xil_types.h>
#include <stdio.h>
#include "polled_input.h"

#define GPIO_DEVICE_ID  	XPAR_XGPIOPS_0_DEVICE_ID
#define INPUTPIN 51

static XGpioPs* gpio ;
static int pin;

void init_input(int pinf,XGpioPs* gpiof){

	gpio = gpiof;
	pin = pinf;
	XGpioPs_SetDirectionPin(gpio,pin, 0);
}

int is_pressed(){

	return XGpioPs_ReadPin(gpio,pin);
}
