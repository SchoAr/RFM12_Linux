/*
 * SPI_Wrapper.h
 *
 *  Created on: 24.11.2014
 *      Author: SchoAr
 */

#ifndef SPI_WRAPPER_H_
#define SPI_WRAPPER_H_

#include <xil_types.h>

void init_spi(void);

u16 xfer(u16 cmd);
u8 byte(u8 cmd);
u16 writeCmd(u16 cmd);

#endif /* SPI_WRAPPER_H_ */
