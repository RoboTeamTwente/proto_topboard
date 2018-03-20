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

int8_t initRobo(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address){
	/*
	 * TODO
	 * I need to review all those settings and need to check
	 * if this is acutally compatible with what I configured the basestation with.
	 */
	//reset and flush buffer
	if(NRFinit(spiHandle, nrf24nssHigh, nrf24nssLow, nrf24ceHigh, nrf24ceLow, nrf24irqRead ) != 0) {
		return -1; //error
	}

	//enable RX interrupts, disable TX interrupts
	RXinterrupts();

	//activate all interrupts
	//writeReg(CONFIG, readReg(CONFIG) & ~(MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT));

	//set the frequency channel
	setFreqChannel(freqChannel);

	//enable pipe 0 and 1, diabable all other pipes
	setDataPipes(ERX_P0 | ERX_P1);

	uint8_t addressLong[5] = {0x12, 0x34, 0x56, 0x78, 0x90 + address};
	//uint8_t addressLong[5] = {0xA8, 0xA8, 0xE1, 0xF0, 0xC6};
	//set the RX address of data pipe 1
	setRXaddress(addressLong, 1);

	setLowSpeed();

	enableAutoRetransmitSlow();


	//enable Auto Acknowledgment for Pipe 1
	//writeReg(EN_AA, ENAA_P1);

	//enable dynamic packet length, ack payload, dynamic acks
	//writeReg(FEATURE, EN_DPL | EN_ACK_PAY);



	//set the RX buffer size to 12 bytes
	setRXbufferSize(12);

	//go to RX mode and start listening
	powerUpRX();

	return 0;
}

void roboCallback(dataPacket* dataStruct){
	uint8_t dataArray[12];


	uint8_t status_reg = readReg(STATUS);
	if( (status_reg & RX_DR) == 0) {
		//if no packet arrived, abort
		return;
	}
	uprintf("In roboCallback. A packet arrived.");

	//retrieve on which pipe number the new packet arrived
	uint8_t dataPipeNo = (status_reg >> 1) & 0b111; //reading RX_P_NO

	uprintf("New packet on Pipe Number: %i\n", dataPipeNo);

	//retrieve the amount of bytes of the specified data pipe
	uint8_t bytesReceived = readReg(RX_PW_P0 + dataPipeNo) & 0b11111;

	uprintf("Received Amount of Bytes: %i\n", bytesReceived);

	/*
	 * TODO
	 * I DON'T WANT THESE LOW LEVEL FUNCTIONS
	 * IN A HIGH LEVEL SOURCE FILE!!
	 * GO FIX!
	 *
	 * Make a receivePacket() function
	 *
	 *
	 */

	nrf24ceLow();
	readData(dataArray, bytesReceived);
	//clear RX interrupt
	uprintf("Clearing RX_DR interrupt.\n");
	writeReg(STATUS, RX_DR);
	nrf24ceHigh();

	uprintf("Raw packet data in DEC: \n");
	for(int i=0; i<bytesReceived; i++) {
		uprintf("%i ", dataArray[i]);
	}
	uprintf("\n");

	uprintf("Raw packet data in HEX: \n");
	for(int i=0; i<bytesReceived; i++) {
		uprintf("%02x ", dataArray[i]);
	}
	uprintf("\n");





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

	uint8_t dummyvalue = 0xff; //a dummy value to be sent as an ACK
	if(writeACKpayload(&dummyvalue, 1) != 0) { //eat this, basestation!
		uprintf("Error writing ACK payload.\n");
		return;
	} else {
		uprintf("ACK payload written with the following payload: \n");
		uprintf("%2x \n",dummyvalue);
		/*
		for(int i=0; i<bytesReceived; i++) {
			uprintf("%02x ", dataArray[i]);
		}
		uprintf("\n");
		*/
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
