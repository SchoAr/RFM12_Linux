/*
 * delay.c
 *
 *  Created on: 30.12.2014
 *      Author: armetallica
 */
#include <xil_types.h>

void delay_100us(u32 microseconds){
	u64 i;
	u64 j;
	for(j = 0; j<microseconds; j++)
    	for(i = 0; i < 7318 ; i++);
}

void delay_ms(u32 milliseconds){
	u64 i;
	u64 j;
	for(j = 0; j<milliseconds; j++)
    	for(i = 0; i < 70258 ; i++);
}

void delay_s(u32 seconds){
	int i;
	for(i = 0 ; i<seconds ; i++)
		delay_ms(1000);// call delay_ms with 1000 ms.
}


