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

void initRobo(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address){
	//reset and flush buffer
	NRFinit(spiHandle);

	//enable RX interrupts, disable TX interrupts
	RXinterrupts(spiHandle);

	//set the frequency channel
	setFreqChannel(spiHandle, freqChannel);

	//enable pipe 0 and 1, diabable all other pipes
	uint8_t dataPipeArray[6] = {1, 1, 0, 0, 0, 0};
	setDataPipeArray(spiHandle, dataPipeArray);

	uint8_t addressLong[5] = {0x12, 0x34, 0x56, 0x78, 0x90 + address};
	//uint8_t addressLong[5] = {0xA8, 0xA8, 0xE1, 0xF0, 0xC6};
	//set the RX address of data pipe 1
	setRXaddress(spiHandle, addressLong, 1);

	setLowSpeed(spiHandle);

	enableAutoRetransmitSlow(spiHandle);

	//set the RX buffer size to 12 bytes
	setRXbufferSize(spiHandle, 12);

	//go to RX mode and start listening
	powerUpRX(spiHandle);


}

void roboCallback(SPI_HandleTypeDef* spiHandle, dataPacket* dataStruct){
	uint8_t dataArray[12];



	ceLow(spiHandle);
	readData(spiHandle, dataArray, 12);
	//clear RX interrupt
	writeReg(spiHandle, 0x07, 0x4E);
	ceHigh(spiHandle);



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

}

void printDataStruct(dataPacket* dataStruct){
/*	sprintf(smallStrBuffer, "robotID = %i\n", dataStruct->robotID);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "robotVelocity = %i\n", dataStruct->robotVelocity);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "movingDirection = %i\n", dataStruct->movingDirection);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "rotationDirection = %i\n", dataStruct->rotationDirection);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "angularVelocity = %i\n", dataStruct->angularVelocity);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "kickForce = %i\n", dataStruct->kickForce);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "kick = %i\n", dataStruct->kick);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "chipper = %i\n", dataStruct->chipper);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "forced = %i\n", dataStruct->forced);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "driblerDirection = %i\n", dataStruct->driblerDirection);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "driblerSpeed = %i\n", dataStruct->driblerSpeed);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "currentRobotVelocity = %i\n", dataStruct->currentRobotVelocity);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "currentMovingDirection = %i\n", dataStruct->currentMovingDirection);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "currentRotationDirection = %i\n", dataStruct->currentRotationDirection);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "currentAngularVelocity = %i\n", dataStruct->currentAngularVelocity);
	TextOut(smallStrBuffer);
	sprintf(smallStrBuffer, "videoDataSend = %i\n", dataStruct->videoDataSend);
	TextOut(smallStrBuffer);*/
}





/*
 * Pin setters
 */


//TODO: fill in the GPIOs of the top board which connect the nRF24 module to the MCU
//put the nss pin corresponding to the SPI used high
void nrf24nssHigh(){
	//NSS / CSN : chip select
	//HAL_GPIO_WritePin(GPIOD, CSN_SPI3_Pin, GPIO_PIN_SET);
}

//put the nss pin corresponding to the SPI used low
void nrf24nssLow(){
	//HAL_GPIO_WritePin(GPIOD, CSN_SPI3_Pin, GPIO_PIN_RESET);
}

//put the ce pin corresponding to the SPI used high
void nrf24ceHigh(){
	//CE: chip enable
	//HAL_GPIO_WritePin(GPIOD, CE_SPI3_Pin, GPIO_PIN_SET);
}

//put the ce pin corresponding to the SPI used low
void nrf24ceLow(){
	//HAL_GPIO_WritePin(GPIOD, CE_SPI3_Pin, GPIO_PIN_RESET);
}
