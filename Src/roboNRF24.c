/*
 * roboNRF24.c
 *
 *  Created on: Mar 16, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *   Specific commands for using the nRF24 wireless module
 *   with our robot.
 */

#include <roboNRF24.h>
#include "PuttyInterface/PuttyInterface.h" //should be removed after debugging

int8_t initRobo(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t roboID){
	/*
	 * TODO
	 * I need to review all those settings and need to check
	 * if this is acutally compatible with what I configured the basestation with.
	 */
	//reset and flush buffer
	if(NRFinit(spiHandle, nrf24nssHigh, nrf24nssLow, nrf24ceHigh, nrf24ceLow, nrf24irqRead ) != 0) {
		return -1; //error
	}

	//activate all interrupts
	writeReg(CONFIG, readReg(CONFIG) & ~(MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT));

	setFreqChannel(freqChannel);
	//setLowSpeed();


	//enable pipe 0 and 1, disable all other pipes
	setDataPipes(ERX_P1);

	//with respect to the following discussion: https://devzone.nordicsemi.com/f/nordic-q-a/20235/nrf24l01p-data-pipe-forbidden-numbers
	//the address was chosen to not start or end with a lot of zeros or a lot of ones or
	//an alternating series which could be mistaken for the preamble.
	//Therefore, 0x99 was chosen, as it represents the bit series: "10011001"
	uint8_t addressLong[5] = {0x99, 0xB0 + roboID, 0x34, 0x56, 0x99};
	//uint8_t addressLong[5] = {0xA8, 0xA8, 0xE1, 0xF0, 0xC6};
	//set the RX address of data pipe x
	setRXaddress(addressLong, 1);

	//enable dynamic packet length, ack payload, dynamic acks
	writeReg(FEATURE, EN_DPL | EN_ACK_PAY | EN_DYN_ACK);

	//enable Auto Acknowledgment for Pipe 1
	writeReg(EN_AA, ENAA_P1);


	//enable dynamic packet length for data pipe(s)
	writeReg(DYNPD, DPL_P1);

	//writeReg(RX_PW_P1, 32); //set static payload length........ SHOULD NOT BE NEEDED

	//go to RX mode and start listening
	powerUpRX();

	//preparing a dummy-payload which will be sent
	//when the very first packet was received
	uint8_t dummyvalues[32];
	//cold food means: we just booted up
	dummyvalues[0] = 0xc0;
	dummyvalues[1] = 0x1d;
	dummyvalues[2] = 0xf0;
	dummyvalues[3] = 0x0d;
	dummyvalues[4] = 0x31;
	dummyvalues[5] = 0x41;
	dummyvalues[6] = 0x59;

	int8_t error = writeACKpayload(dummyvalues, 7, 1);
	uprintf("writeACKpayload() returned %i\n", error);
	//error = writeACKpayload(dummyvalues, 32, 1);
	//uprintf("writeACKpayload() returned %i\n", error);
	//writeACKpayload(dummyvalues, 32, 1);

	//for(uint8_t i=0; i< 32; i++)
	//	nrfNOP();

	return 0;
}
/*
 * TODO
 * this needs to be extended that it will read payloads in a loop
 * as long as there is data in the FIFO.
 * Read the FIFO_STATUS and check RX_EMPTY.
 *
 */
void roboCallback(){
	uint8_t verbose = 1;
	uint8_t dataArray[32];


	uint8_t status_reg = readReg(STATUS);
	if( (status_reg & RX_DR) == 0) {
		//if no packet arrived, abort
		return;
	}

	//check on which pipe number the new packet arrived
	uint8_t dataPipeNo = (status_reg >> 1) & 0b111; //reading RX_P_NO

	if(verbose) uprintf("New packet on Pipe Number: %i   ", dataPipeNo);

	uint8_t bytesReceived = getDynamicPayloadLength();
	if(verbose) uprintf("with payload length: %i Bytes  --  ", bytesReceived);

	/*
	 * Put that into a readPayload() function ?
	 */
	nrf24ceLow();
	//actually reading the payload
	readData(dataArray, bytesReceived);

	//putting the new data from the packet on the struct
	packetToRoboData(dataArray, &receivedRoboData);
	//clear RX interrupt
	if(verbose) uprintf("Clearing RX_DR interrupt.\n");
	writeReg(STATUS, RX_DR);
	nrf24ceHigh();


	if(verbose) {
		uprintf("Raw packet data in DEC: ");
		for(int i=0; i<bytesReceived; i++) {
			uprintf("%i ", dataArray[i]);
		}
		uprintf("\n");

		uprintf("Raw packet data in HEX: ");
		for(int i=0; i<bytesReceived; i++) {
			uprintf("%02x ", dataArray[i]);
		}
		uprintf("\n");
	}


	flushRX();



	//building a packet from the current roboAckData struct
	uint8_t byteArray[32];
	roboAckDataToPacket(&preparedAckData, byteArray);
	uint8_t ackDataLength = 11;
	if(receivedRoboData.debug_info)
		ackDataLength = 23; //adding xsense data
	if(writeACKpayload(byteArray, ackDataLength, dataPipeNo) != 0) { //eat this, basestation!
		if(verbose) uprintf("Error writing ACK payload. TX FIFO full?\n");
		return;
	} else {
		if(verbose) {
			uprintf("ACK payload written.");
		}
	}

}


/*
 * Pin setters and reader
 */


//put the nss pin corresponding to the SPI used high
void nrf24nssHigh(){
	//NSS / CSN : chip select
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
}

//put the nss pin corresponding to the SPI used low
void nrf24nssLow(){
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
}

//put the ce pin corresponding to the SPI used high
void nrf24ceHigh(){
	//CE: chip enable
	//In this board revision, this pin is directly connected to Vdd ("high").
}

//put the ce pin corresponding to the SPI used low
void nrf24ceLow(){
	//HAL_GPIO_WritePin(GPIOD, CE_SPI3_Pin, GPIO_PIN_RESET);
	//In this board revision, this pin is directly connected to Vdd ("high").
}


//read the interrupt pin
uint8_t nrf24irqRead(){
	return !HAL_GPIO_ReadPin(SPI2_IRQ_GPIO_Port, SPI2_IRQ_Pin);
}
