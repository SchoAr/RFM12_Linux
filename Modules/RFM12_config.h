/*
 * RFM12_config.h
 *
 *  Created on: 30.12.2014
 *      Author: armetallica
 */

#ifndef RFM12_CONFIG_H_
#define RFM12_CONFIG_H_


#define DEBUG 1
/*********************************/
#define IRQ_PIN 0 	//MIO0
#define RX_LED 9
#define TX_LED 14
#define ON_LED 15
/********************************/
#define RF12_433MHZ     1
#define RF12_868MHZ     2
#define RF12_915MHZ     3

#define FREQ_BAND RF12_433MHZ
#define NETWORK_ID		212		/*0xD4 Sync Pattern for RFM12 Required !!!*/
#define TX_POWER 		0 		/* - txPower [optional - default = 0 (max)] (7 is min value)	*/

#endif /* RFM12_CONFIG_H_ */
