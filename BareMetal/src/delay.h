/*
 * delay.h
 *
 *  Created on: 30.12.2014
 *      Author: armetallica
 */

#ifndef DELAY_H_
#define DELAY_H_
#include <xil_types.h>

/**
 * This function waits busy x seconds.
 * The function calls es the delay_ms function with the value second*1000
 *
 *
 * @param	seconds: It contains the number of seconds.
 */
void delay_s(u32 seconds);


/**
 * This function waits busy x milliseconds.
 * It use a for loop to wait until x milliseconds.
 *
 * @param	milliseconds: It contains the number of milliseconds.
 */
void delay_ms(u32 milliseconds);

/**
 * This function waits busy x 100 microseconds.
 * It use a for loop to wait until x 100 microseconds.
 *
 * @param	microseconds: It contains the number of 100 microseconds.
 */
void delay_100us(u32 microseconds);

#endif /* DELAY_H_ */
