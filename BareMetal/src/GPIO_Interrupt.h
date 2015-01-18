/*
 * GPIO_Interrupt.h
 *
 *  Created on: 29.12.2014
 *      Author: armetallica
 */

#ifndef GPIO_INTERRUPT_H_
#define GPIO_INTERRUPT_H_

#include "xgpiops.h"
#include "xscugic.h"

#define BUTTON 51
#define RISNG_EDGE 0
#define FALLING_EDGE 1

void init_GPIOInterrupt(int pin,
						void (*fp)(void),
						int risingFalling,
						XGpioPs* gpiof,
						XGpioPs_Config *ConfigPtrf,
						XScuGic* GicInstancePtrf);

#endif /* GPIO_INTERRUPT_H_ */
