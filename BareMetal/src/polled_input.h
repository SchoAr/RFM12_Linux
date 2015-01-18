/*
 * input.h
 *
 *  Created on: 27.11.2014
 *      Author: SchoAr
 */

#ifndef INPUT_H_
#define INPUT_H_

#define GPIO_DEVICE_ID  	XPAR_XGPIOPS_0_DEVICE_ID
#define INPUTPIN 51

void init_input(int pinf,XGpioPs* gpiof);
int is_pressed();

#endif /* INPUT_H_ */
