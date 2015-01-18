/*
 RFM12B Library. Based on work done by JeeLabs.org ported to mBed by SK Pang.
 http://jeelabs.net/projects/cafe/wiki/RF12
 Jan 2012 skpang.co.uk

 RFM12B Library (Moteino Comunication Protocol). Based on work done by Felix Rusu ported to mBed by Hugo Rodrigues
 http://lowpowerlab.com/blog/2012/12/28/rfm12b-arduino-library/
 May 2013 Hugo Rodrigues

 http://opensource.org/licenses/mit-license.php

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 */

#include "RFM12.h"
#include "platform.h"
#include "GPIO_Interrupt.h"
#include "xgpiops.h"
#include <stdio.h>
#define GPIO_DEVICE_ID  	XPAR_XGPIOPS_0_DEVICE_ID

static XGpioPs_Config *ConfigPtr;
static XGpioPs gpio;
static XScuGic GicInstance;

volatile uint8_t nodeID;                    // address of this node
volatile uint8_t networkID;                 // network group

volatile uint8_t* Data;
volatile uint8_t* DataLen;

volatile uint8_t rf12_buf[RF_MAX];   // recv/xmit buf, including hdr & crc bytes

volatile uint8_t rxfill;                    // number of data bytes in rf12_buf
volatile int8_t rxstate;                    // current transceiver state
volatile uint16_t rf12_crc;                 // running crc value
uint32_t seqNum;                            // encrypted send sequence number
uint32_t cryptKey[4];                       // encryption key to use
long rf12_seq;                         // seq number of encrypted packet (or -1)

u8 useEncryption = 0;

uint16_t crc16_update(uint16_t crc, uint8_t data) {
	int i;

	crc ^= data;
	for (i = 0; i < 8; ++i) {
		if (crc & 1)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = (crc >> 1);
	}

	return crc;
}
static int SetupInterruptSystem(u16 IntcDeviceID, XScuGic *IntcInstancePtr) {
	int Status;
	XScuGic_Config *IntcConfig;
	IntcConfig = XScuGic_LookupConfig(IntcDeviceID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}
	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
			IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
			(Xil_ExceptionHandler) XScuGic_InterruptHandler, IntcInstancePtr);
	return XST_SUCCESS;
}

// Call this once with params:
// - node ID (0-31)
// - frequency band (RF12_433MHZ, RF12_868MHZ, RF12_915MHZ)
// - networkid [optional - default = 170] (0-255 for RF12B, only 212 allowed for RF12)
// - txPower [optional - default = 0 (max)] (7 is min value)
// - airKbps [optional - default = 38.31Kbps]
void Initialize(uint8_t nodeid, uint8_t freqBand, uint8_t groupid,
		uint8_t txPower, uint8_t airKbps) {
	init_spi();
	//INPUT
	int status = SetupInterruptSystem(XPAR_SCUGIC_SINGLE_DEVICE_ID,
			&GicInstance);
	if (status != XST_SUCCESS) {
		printf("[GPIO_Interrupt]  ERROR IN INTERRUPT SETUP\n");
	}
	ConfigPtr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
	int ret = XGpioPs_CfgInitialize(&gpio, ConfigPtr, ConfigPtr->BaseAddr);
	if (ret != XST_SUCCESS) {
		print("ERROR INIT Input\n\r");
	}
	init_GPIOInterrupt(0, &InterruptHandler, FALLING_EDGE, &gpio, ConfigPtr,
			&GicInstance);

	nodeID = nodeid;
	networkID = groupid;
	rf12_grp= groupid;

	writeCmd(0x0000);    // initial SPI transfer added to avoid power-up problem
	writeCmd(RF_SLEEP_MODE);            // DC (disable clk pin), enable lbd

	// wait until RFM12B is out of power-up reset, this takes several *seconds*
	writeCmd(RF_TXREG_WRITE);           // in case we're still in OOK mode

	/*    while (NIRQ == 0)
	 writeCmd(0x0000);
	 */
	writeCmd(0x80C7 | (freqBand << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF
	writeCmd(0xA640); // Frequency is exactly 434/868/915MHz (whatever freqBand is)
	writeCmd(0xC600 + airKbps);   //Air transmission baud rate: 0x08= ~38.31Kbps
	writeCmd(0x94A2);                   // VDI,FAST,134kHz,0dBm,-91dBm
	writeCmd(0xC2AC);                   // AL,!ml,DIG,DQD4
	if (networkID != 0) {
		writeCmd(0xCA83);               // FIFO8,2-SYNC,!ff,DR
		writeCmd(0xCE00 | networkID);   // SYNC=2DXX
	} else {
		writeCmd(0xCA8B);               // FIFO8,1-SYNC,!ff,DR
		writeCmd(0xCE2D);               // SYNC=2D
	}

	writeCmd(0xC483);                   // @PWR,NO RSTRIC,!st,!fi,OE,EN
	writeCmd(0x9850 | (txPower > 7 ? 7 : txPower)); // !mp,90kHz,MAX OUT
	writeCmd(0xCC77);                   // OB1, OB0, LPX, ddy, DDIT, BW0
	writeCmd(0xE000);                   // NOT USE
	writeCmd(0xC800);                   // NOT USE
	writeCmd(0xC049);                   // 1.66MHz,3.1V

	rxstate = TXIDLE;
}

void InterruptHandler() {

	// a transfer of 2x 16 bits @ 2 MHz over SPI takes 2x 8 us inside this ISR
	writeCmd(0x0000);

	if (rxstate == TXRECV) {
		uint8_t in = xfer(RF_RX_FIFO_READ);

		if (rxfill == 0 && networkID != 0)
			rf12_buf[rxfill++] = networkID;

		rf12_buf[rxfill++] = in;
		rf12_crc = crc16_update(rf12_crc, in);

		if (rxfill >= rf12_len+ 6 || rxfill >= RF_MAX)
		xfer(RF_IDLE_MODE);
	} else {
		uint8_t out;

		if (rxstate < 0) {
			uint8_t pos = 4 + rf12_len + rxstate++;
			out = rf12_buf[pos];
			rf12_crc = crc16_update(rf12_crc, out);
		} else {
			switch (rxstate++) {
				case TXSYN1:
				out = 0x2D;
				break;
				case TXSYN2:
				out = rf12_grp;
				rxstate = - (3 + rf12_len);
				break;
				case TXCRC1:
				out = rf12_crc;
				break;
				case TXCRC2:
				out = rf12_crc >> 8;
				break;
				case TXDONE:
				xfer(RF_IDLE_MODE); // fall through
				out = 0xAA;
				break;
				default:
				out = 0xAA;
			}
		}
		xfer(RF_TXREG_WRITE + out);
	}
}

void ReceiveStart(void) {
	rxfill = rf12_len= 0;
	rf12_crc = ~0;

	if (networkID != 0)
	rf12_crc = crc16_update(~0, networkID);

	rxstate = TXRECV;
	xfer(RF_RECEIVER_ON);
}

u8 ReceiveComplete(void) {
	if (rxstate == TXRECV && (rxfill >= rf12_len+ 6 || rxfill >= RF_MAX)) {
		rxstate = TXIDLE;

		if (rf12_len > RF12_MAXDATA) {
			rf12_crc = 1; // force bad crc if packet length is invalid
		}
		if (RF12_DESTID == 0 || RF12_DESTID == nodeID) {

			if (rf12_crc == 0 && useEncryption)
			Encryption(0);
			else
			rf12_seq = -1;

#ifdef DEBUG
			printf("\nReceived message from [%d]; crc:%x,  len: %d, message: ", RF12_SOURCEID, rf12_crc, rf12_len);
			for (int i=0; i<rf12_len; i++) {
				printf("%c", rf12_data[i]);
			}
			printf("\n");
#endif

			return 1; // it's a broadcast packet or it's addressed to this node
		}
	}
	if (rxstate == TXIDLE)
	ReceiveStart();

	return 0;
}

u8 CanSend() {
	// no need to test with interrupts disabled: state TXRECV is only reached
	// outside of ISR and we don't care if rxfill jumps from 0 to 1 here
	if (rxstate == TXRECV && rxfill == 0
			&& (byte(0x00) & (RF_RSSI_BIT >> 8)) == 0) {
		xfer(RF_IDLE_MODE); // stop receiver
		rxstate = TXIDLE;
		return 1;
	}
	return 0;
}

void SendStart_short(uint8_t toNodeID, u8 requestACK, u8 sendACK) {

	rf12_hdr1= toNodeID | (sendACK ? RF12_HDR_ACKCTLMASK : 0);
	rf12_hdr2= nodeID | (requestACK ? RF12_HDR_ACKCTLMASK : 0);

#ifdef DEBUG
	printf("SendStart to Node [%d], from Node [%d] \n", toNodeID, nodeID);
#endif

	if (useEncryption)
	Encryption(1);

	rf12_crc = ~0;
	rf12_crc = crc16_update(rf12_crc, rf12_grp);
	rxstate = TXPRE1;

	xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}

void SendStart(uint8_t toNodeID, const void* sendBuf, uint8_t sendLen,
		u8 requestACK, u8 sendACK) {

	rf12_len= sendLen;
	memcpy((void*) rf12_data, sendBuf, sendLen);

#ifdef DEBUG
	printf("\nSending message from [%d]; crc:%x,  len: %d, message: ", nodeID, rf12_crc, rf12_len);
	for (int i=0; i<rf12_len; i++) {
		printf("%c", rf12_data[i]);
	}
#endif

	SendStart_short(toNodeID, requestACK, sendACK);
}

/// Should be called immediately after reception in case sender wants ACK
void SendACK(const void* sendBuf, uint8_t sendLen) {
	while (!CanSend())
		ReceiveComplete();
	SendStart(RF12_SOURCEID, sendBuf,
			(sendLen > 0) ? sendLen : strlen((const char*) sendBuf), 0,
			1);
}

void Send(uint8_t toNodeID, const void* sendBuf, uint8_t sendLen,
		u8 requestACK) {
	while (!CanSend())
		ReceiveComplete();
	SendStart(toNodeID, sendBuf,
			(sendLen > 0) ? sendLen : strlen((const char*) sendBuf), requestACK,
			0);
}

u8 GetSender(void) {
	return RF12_SOURCEID;
}

volatile uint8_t * GetData(void) {

	return (uint8_t*) rf12_data;
}

uint8_t GetDataLen(void) {
	return rf12_len;
}

u8 ACKRequested() {
	return RF12_WANTS_ACK;
}

/// Should be polled immediately after sending a packet with ACK request
u8 ACKReceived(uint8_t fromNodeID) {
	if (ReceiveComplete())
		return CRC_Pass() && RF12_DESTID == nodeID
				&& (RF12_SOURCEID == fromNodeID || fromNodeID == 0)
				&& (rf12_hdr1&RF12_HDR_ACKCTLMASK) &&
		!(rf12_hdr2 & RF12_HDR_ACKCTLMASK);
		return 0;
	}

u8 CRC_Pass(void) {
	return (rf12_crc == 0);
}

// XXTEA by David Wheeler, adapted from http://en.wikipedia.org/wiki/XXTEA
#define DELTA 0x9E3779B9
#define MX (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (cryptKey[(uint8_t)((p&3)^e)] ^ z)))
void Encryption(u8 encrypt) {

	uint32_t y, z, sum, *v = (uint32_t*) rf12_data;
	uint8_t p, e, rounds = 6;

	if (encrypt) {
		// pad with 1..4-byte sequence number
		*(uint32_t*) (rf12_data + rf12_len) = ++seqNum;
		uint8_t pad = 3 - (rf12_len & 3);
		rf12_len += pad;
		rf12_data[rf12_len] &= 0x3F;
		rf12_data[rf12_len] |= pad << 6;
		++rf12_len;
		// actual encoding
		char n = rf12_len / 4;
		if (n > 1) {
			sum = 0;
			z = v[n-1];
			do {
				sum += DELTA;
				e = (sum >> 2) & 3;
				for (p=0; p<n-1; p++)
				y = v[p+1], z = v[p] += MX;
				y = v[0];
				z = v[n-1] += MX;
			}while (--rounds);
		}
	} else if (rf12_crc == 0) {
		// actual decoding
		char n = rf12_len / 4;
		if (n > 1) {
			sum = rounds*DELTA;
			y = v[0];
			do {
				e = (sum >> 2) & 3;
				for (p=n-1; p>0; p--)
				z = v[p-1], y = v[p] -= MX;
				z = v[n-1];
				y = v[0] -= MX;
			}while ((sum -= DELTA) != 0);
		}
		// strip sequence number from the end again
		if (n > 0) {
			uint8_t pad = rf12_data[--rf12_len] >> 6;
			rf12_seq = rf12_data[rf12_len] & 0x3F;
			while (pad-- > 0)
			rf12_seq = (rf12_seq << 8) | rf12_data[--rf12_len];
		}
	}

}

void SetEncryptionKey(const uint8_t* key) {
	uint8_t i;
	if (key != 0) {
		for (i = 0; i < sizeof cryptKey; ++i)
			((uint8_t*) cryptKey)[i] = key[i];

		useEncryption = 1;
	} else
		useEncryption = 0;
}

void Sleep_time(int n) {
	if (n < 0)
		xfer(RF_IDLE_MODE);
	else {
		xfer(RF_WAKEUP_TIMER | 0x0500 | n);
		xfer(RF_SLEEP_MODE);
		if (n > 0)
			xfer(RF_WAKEUP_MODE);
	}
	rxstate = TXIDLE;
}
void Sleep() {

	Sleep_time(0);
}
void Wakeup() {

	Sleep(-1);
}

u8 LowBattery() {

	return (xfer(0x0000) & RF_LBD_BIT) != 0;
}
