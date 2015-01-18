/*
 * userLed.h
 *
 *  Created on: 24.11.2014
 *      Author: SchoAr
 */

#ifndef USERLED_H_
#define USERLED_H_

#include "xgpiops.h"
#include <xil_types.h>

#define GPIO_DEVICE_ID  	XPAR_XGPIOPS_0_DEVICE_ID
#define USER_LED 47

void init_Led(int pin,XGpioPs* gpiof,XGpioPs_Config *ConfigPtrf);
void toggle_Led(int pin);
void on_Led(int pin);
void off_Led(int pin);

#endif /* USERLED_H_ */
