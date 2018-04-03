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

	//go to RX mode and start listening
	powerUpRX();

	//preparing a dummy-payload which will be sent
	//when the very first packet was received
	uint8_t dummyvalue = 0x33;
	writeACKpayload(&dummyvalue, 1, 1);
	return 0;
}
/*
 * TODO
 * this needs to be extended that it will read payloads in a loop
 * as long as there is data in the FIFO.
 * Read the FIFO_STATUS and check RX_EMPTY.
 *
 */
void roboCallback(dataPacket* dataStruct){
	uint8_t dataArray[12];


	uint8_t status_reg = readReg(STATUS);
	if( (status_reg & RX_DR) == 0) {
		//if no packet arrived, abort
		return;
	}

	//blink
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

	//uprintf("In roboCallback. A packet arrived.");

	//retrieve on which pipe number the new packet arrived
	uint8_t dataPipeNo = (status_reg >> 1) & 0b111; //reading RX_P_NO

	uprintf("New packet on Pipe Number: %i   ", dataPipeNo);

	uint8_t bytesReceived = getDynamicPayloadLength();
	uprintf("with payload length: %i Bytes  --  ", bytesReceived);

	/*
	 * Put that into a readPayload() function ?
	 */
	nrf24ceLow();
	//actually reading the payload
	readData(dataArray, bytesReceived);
	//clear RX interrupt
	uprintf("Clearing RX_DR interrupt.\n");
	writeReg(STATUS, RX_DR);
	nrf24ceHigh();

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

	flushRX();



	dataStruct->robotID = dataArray[0] >> 4;
	dataStruct->robotVelocity = ((dataArray[0] & 0x0F) << 9) + (dataArray[1] << 1) + ((dataArray[2] & 0x80) >> 7);
	dataStruct->movingDirection = ((dataArray[2] & 0x7F) << 2) + ((dataArray[3] & 0xC0) >> 6);
	dataStruct->rotationDirection = dataArray[3] & 0x8;
	dataStruct->angularVelocity = ((dataArray[3] & 0x7) << 8) + dataArray[4];
	dataStruct->kickForce = dataArray[5];
	dataStruct->kick = dataArray[6] & 0x40;
	dataStruct->chipper = dataArray[6] & 0x20;
	dataStruct->forced = dataArray[6] & 0x10;
	dataStruct->driblerDirection = dataArray[6] & 0x8;
	dataStruct->driblerSpeed = dataArray[6] & 0x7;
	dataStruct->currentRobotVelocity = (dataArray[7] << 5) + ((dataArray[8] & 0xF8) >> 3);
	dataStruct->currentMovingDirection = ((dataArray[8] & 0x07) << 6) + ((dataArray[9] & 0xFC) >> 2);
	dataStruct->currentRotationDirection = (dataArray[10] & 0x40) >> 6;
	dataStruct->currentAngularVelocity = ((dataArray[9] &0x03) << 9) + (dataArray[10] << 1) + (dataArray[11] & 0x80);
	dataStruct->videoDataSend = (dataArray[6] & 0x80) >> 7;

//just for testing.. the ACK packets will be looking different when we're done
/* */

	uint8_t dummyvalue = 0xfa; //a dummy value to be sent as an ACK
	if(writeACKpayload(&dummyvalue, 1, dataPipeNo) != 0) { //eat this, basestation!
		uprintf("Error writing ACK payload. TX FIFO full?\n");
		return;
	} else {
		uprintf("ACK payload written with the following payload: ");
		uprintf("%2x \n",dummyvalue);
	}

	//HAL_Delay(10);
	//flushRX();
	//flushTX();
	//clearInterrupts();

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
