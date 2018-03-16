/*
 * myNRF24basic.c
 *
 *  Created on: Mar 13, 2018
 *      Author: Ulf Stottmeister
 *
 *  This is the low level function library for communicating with
 *  an nRF24L01 wireless module.
 *  Those functions are supposed to be only called by functions from within the library itself (from functions in myNRF24.c)
 */
#include "myNRF24basic.h"
//#include "gpio.h" //for pin definitions and GPIO functions

//*****************************low level library********************************//
//******************the user is not supposed to use these***********************//


//returns 0 on success; -1 on error
int8_t clearInterrupts(SPI_HandleTypeDef* spiHandle) {
	//0x70 clears the interrupts for: Rx Data Ready, Tx Data Sent and Maximum Retransmits
	//see datasheet page 56 for details
	//forward return code (error code) of writeReg() to caller of clearInterrupts()
	return writeReg(spiHandle, STATUS, (RX_DR | TX_DS | MAX_RT));
}

//write to a register
//returns 0 on success; -1 on error
int8_t writeReg(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t data){
	if(reg == 	RX_ADDR_P0 || reg == RX_ADDR_P1 || reg == TX_ADDR){
		//TextOut("Error, this is a multi-byte register. use writeRegMulti instead\n");
		return -1; //error
	}
	else if(reg > FEATURE || reg == OBSERVE_TX || reg == RPD || (reg > FIFO_STATUS && reg < DYNPD)){
		//TextOut("Error, invalid register. It is either read only or non-existing\n");
		return -1; //error
	}

	//commands can only be given after a falling edge of the nss pin
	//see figure 23 of datasheet
	nssLow(spiHandle);

	//the SPI command to write to register X, you take the number of register X
	//and add 2^5 to it (set the bit on position 5).
	uint8_t sendData = setBit(reg, 5, 1);

	//comand: write to register reg
	if(HAL_SPI_Transmit(spiHandle, &sendData, 1, 100) != HAL_OK)
		return -1; //HAL/SPI error

	sendData = data;
	//set the value of the register
	if(HAL_SPI_Transmit(spiHandle, &sendData, 1, 100) != HAL_OK)
		return -1; //HAL/SPI error

	nssHigh(spiHandle);
	//HAL_Delay(10);
	return 0; //return with no error
}


//write to a multi-byte register
//returns 0 on success; -1 on error
int8_t writeRegMulti(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* pdata, uint8_t size){
	if(!(reg == RX_ADDR_P0 || reg == RX_ADDR_P1 || reg == TX_ADDR)){
		//TextOut("Error, invalid register. It is either read only, single byte or non-existing.\n");
		return -1;
	}
	else if(size > 5){
		//TextOut("Error, size can never be bigger than 5\n");
		return -1;
	}
	//commands can only be given after a falling edge of the nss pin
	//see figure 23 of datasheet
	nssLow(spiHandle);

	uint8_t cmd_w_register = reg | (1<<5); //the W_REGISTER command is the register number with an appended 1 at position 5.
	uint8_t receiveData;

	//comand: write to register reg and get status register
	if(HAL_SPI_TransmitReceive(spiHandle, &cmd_w_register, &receiveData, 1, 100) != HAL_OK)
		return -1; //SPI error

	//Do not remove the i
	//it invokes divine intervention
	//int i = 0;
	//Sorry, but I'm removing the i. My mom says superstition brings misfortune... ~Ulf S.

	//send data to the register
	if(HAL_SPI_TransmitReceive(spiHandle, pdata, &receiveData, size, 100) != HAL_OK)
		return -1; //SPI error

	nssHigh(spiHandle);
	//HAL_Delay(10);
	return 0;
}

//read a register
uint8_t readReg(SPI_HandleTypeDef* spiHandle, uint8_t reg){
	if(reg > 0x1D){
		//TextOut("Error, invalid register\n");
		return 0xF0; //error
	}

	//commands can only be given after a falling edge of the nss pin
	//see figure 23 of datasheet
	nssLow(spiHandle);

	uint8_t sendData = reg; //R_REGISTER = 000A AAAA -> AAAAA = 5 bit register address
	uint8_t receiveData;
	//command: read from register reg
	if(HAL_SPI_Transmit(spiHandle, &sendData, 1, 100) != HAL_OK)
		return 0xF0; //error

	//read data from the register
	HAL_SPI_Receive(spiHandle, &receiveData, 1, 100);

	nssHigh(spiHandle);
	//HAL_Delay(10);

	return receiveData;
}

//read a multi-byte register
//output will be stored in the array dataBuffer
//returns 0 on success; -1 on error
int8_t readRegMulti(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* dataBuffer, uint8_t size){
	if(reg > 0x1D){
		TextOut("Error, invalid register\n");
		return -1; //error
	}
	//commands can only be given after a falling edge of the nss pin
	//see figure 23 of datasheet
	nssLow(spiHandle);

	//command: read reg 5
	uint8_t sendData = reg; //R_REGISTER = 000A AAAA -> AAAAA = 5 bit register address
	//command: read from register reg and get status register
	if(HAL_SPI_Transmit(spiHandle, &sendData,1, 100) != HAL_OK)
		return -1; //HAL/SPI error

	//read data from the register
	if(HAL_SPI_Receive(spiHandle, dataBuffer, 5, 100) != HAL_OK)
		return -1; //HAL/SPI error

	nssHigh(spiHandle);

	HAL_Delay(10);

	return 0; //no error
}

