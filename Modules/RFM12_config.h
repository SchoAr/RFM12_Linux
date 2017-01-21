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
// Use Defines for enabling and disabling the LEDs
#define ON_LED_ENABLE 1
#define RX_LED_ENABLE 1
#define TX_LED_ENABLE 1
/*********************************/
#define IRQ_PIN  0 	//MIO0
#define RX_LED   9
#define TX_LED   14
#define ON_LED   15
#define INPUTPIN 51
/********************************/
#define SPI_BUS 32766
#define SPI_BUS_CS0 0	
#define SPI_BUS_SPEED 200000

#define DATA_BUF_SIZE 512
#define POWERON_RESET_WAIT_MILLIS   (5000)
#define POWERON_RESET_WAIT_STEP     (20)
/********************************/
#define ACK_TIME                 15000        //R of ms to wait for an ack (usually between 600ms and 1300ms)
#define CLIENT_MOTEINO_NODE          1
#define CLIENT_MBED_NODE             2
#define SERVER_MOTEINO_NODE         10
#define SERVER_MBED_NODE            11
#define NETWORD_ID                   212 //GROUP / NETWORK ID


#endif /* RFM12_CONFIG_H_ */
