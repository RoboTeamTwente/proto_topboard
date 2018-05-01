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

	writeACKpayload(dummyvalues, 4, 1);


	//go to RX mode and start listening
	powerUpRX();


	return 0;
}
/*
 * TODO
 * this needs to be extended that it will read payloads in a loop
 * as long as there is data in the FIFO.
 * Read the FIFO_STATUS and check RX_EMPTY.
 *
 */
//negative values on erros.
//0 on success
int8_t roboCallback(uint8_t localRobotID){
	uint8_t dataArray[32];
	uint8_t verbose = 0;

	/*
	uint8_t status_reg = readReg(STATUS);
	if( (status_reg & RX_DR) == 0) {
		//if no packet arrived, abort
		if(verbose) uprintf("No new packet arrived.\n");
		//return -1;
	}

	for(uint8_t i=0; i<3; i++) {
		status_reg = readReg(STATUS);
	}
	*/
	//check on which pipe number the new packet arrived
	//uint8_t dataPipeNo = (status_reg >> 1) & 0b111; //reading RX_P_NO

	//if(verbose) uprintf("New packet on Pipe Number: %i   ", dataPipeNo);

	//uint8_t bytesReceived = getDynamicPayloadLength();
	//if(verbose) uprintf("with payload length: %i Bytes  --  ", bytesReceived);

	/*
	 * Put that into a readPayload() function ?
	 */
	nrf24ceLow();
	//actually reading the payload

	/*
	 * For some reason the payload is always off by 3 bytes.. so we need to apply a hack here
	 * Also, the status register isn't read properly. So, we need to assume a static payload length..
	 */
	readData(dataArray, ROBOPKTLEN+3);

	uint8_t receivedRobotID = (dataArray+3)[0]>>3; //see packet format

	if(receivedRobotID != localRobotID) {
		if(verbose) uprintf("Received RobotID was wrong. Rx'd: %i (0x%02x)\n", receivedRobotID, receivedRobotID);
		return -3; //packet wasn't for me (or, more likely: we did not receive any packet and read bullshit from the buffer)
	}
	//putting the new data from the packet on the struct
	packetToRoboData(dataArray+3, &receivedRoboData);
	//clear RX interrupt
	//if(verbose) uprintf("Clearing RX_DR interrupt.\n");
	writeReg(STATUS, RX_DR);
	nrf24ceHigh();


	/*
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
	*/


	flushRX();



	//building a packet from the current roboAckData struct
	uint8_t txPacket[32];
	//roboAckDataToPacket(&preparedAckData, txPacket);
	//uint8_t ackDataLength = SHORTACKPKTLEN;
	//if(receivedRoboData.debug_info)
	//	ackDataLength = FULLACKPKTLEN; //adding xsense data
	robotDataToPacket(&receivedRoboData, txPacket); //sending back the packet we just received
	//if(writeACKpayload(txPacket, ackDataLength, 1) != 0) { //eat this, basestation!
	if(writeACKpayload(txPacket, ROBOPKTLEN, 1) != 0) { //just for testing sending a robot packet to the basestation.
		//if(verbose) uprintf("Error writing ACK payload. TX FIFO full?\n");
		return -2; //error while writing ACK payload to buffer
	}

	return 0; //success
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
