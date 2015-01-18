/*
 * main.c
 *
 *  Created on: 28.12.2014
 *      Author: SchoAr
 */
#include <stdio.h>
#include "platform.h"
#include "RFM12.h"
#include "delay.h"

char send_message[255] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

uint8_t KEY[] = "ABCDABCDABCDABCD";

#define ACK_TIME                 15000        //R of ms to wait for an ack (usually between 600ms and 1300ms)
#define CLIENT_MOTEINO_NODE          1
#define CLIENT_MBED_NODE             2
#define SERVER_MOTEINO_NODE         10
#define SERVER_MBED_NODE            11
#define NETWORD_ID                   212 //GROUP / NETWORK ID

int main() {
	init_platform();
	printf("Hello World\n\r");
	Initialize(CLIENT_MBED_NODE, RF12_433MHZ, 212,0,0x08);

    int i = 1;

//   SendStart(SERVER_MBED_NODE, send_message, i, 0,0);
//    printf("Low Battery = %d \n",LowBattery());
	while (1) {

		SendStart(SERVER_MBED_NODE, send_message, i, 0,0);
	    (i >= 60) ? i = 0 : i++;
	    delay_s(5);
	}

	/************************************************************/
	/*	RFM12_ReceiveStart();
	 while (1) {

	 if (RFM12_Receive_Complete()) {
	 for (i = 0; i < RFM12_GetDataLen(); i++) {
	 printf("%c", (RFM12_GetData())[i]);
	 }
	 }
	 }*/

	cleanup_platform();
	return 0;
}
