/*
 * myNRF24.h
 *
 *  Created on: 19 sep. 2016
 *      Author: Hans-van-der-Heide
 */

#ifndef MYNRF24_H_
#define MYNRF24_H_

//TODO: I think best practice would be pulling it from a Math library.
// include math.h and use "M_PI"
#define PI 3.14159

//#include "TextOut.h"
#include "gpio.h"
#include "stm32f3xx_hal.h"
#include "spi.h"



//*************************auxillery functions**********************************//
//*******************not actually for NRF24 control*****************************//

//set a specific bit in a byte to a 1 or a 0
uint8_t setBit(uint8_t byte, uint8_t position, uint8_t value);

//check if a specific bit in a byte is 1
uint8_t readBit(uint8_t byte, uint8_t position);


//*****************************low level library********************************//
//******************the user is not supposed to use these***********************//

//put the csn pin corresponding to the SPI used high
void nssHigh(SPI_HandleTypeDef* spiHandle);

//put the csn pin corresponding to the SPI used low
void nssLow(SPI_HandleTypeDef* spiHandle);

//put the ce pin corresponding to the SPI used high
void ceHigh(SPI_HandleTypeDef* spiHandle);

//put the ce pin corresponding to the SPI used low
void ceLow(SPI_HandleTypeDef* spiHandle);

uint8_t irqRead(SPI_HandleTypeDef* spihandle);

//write to a register and output debug info to the terminal
void writeRegDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t data);

//write to a register
void writeReg(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t data);

//write to a multi-byte register and output debug info to the terminal
void writeRegMultiDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* data, uint8_t size);

//write to a multi-byte register
void writeRegMulti(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* data, uint8_t size);

//read a register and output debug info to the terminal
uint8_t readRegDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg);

//read a register
uint8_t readReg(SPI_HandleTypeDef* spiHandle, uint8_t reg);

//read a multi-byte register and output debug info to terminal
//output will be stored in the array dataBuffer
void readRegmultiDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* dataBuffer, uint8_t size);

//read a multi-byte register
//output will be stored in the array dataBuffer
void readRegMulti(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* dataBuffer, uint8_t size);




//****************************high level library**************************//
//********************the user may use these functions********************//

//--------------------initialization and configuration--------------------//

//reset to reset value on page 54
void reset(SPI_HandleTypeDef* spiHandle);

//initialize the system:
//reset it and enable pipe 1 and 0
//set pipeWith to 1
//flush TX and RX buffer
void NRFinit(SPI_HandleTypeDef* spiHandle);

//set the address you will send to
void setTXaddress(SPI_HandleTypeDef* spiHandle, uint8_t address[5]);

//set own address note: only data pipe 0 is used in this implementation
void setRXaddress(SPI_HandleTypeDef* spiHandle, uint8_t address[5], uint8_t pipeNumber);

//set a frequency channel
void setFreqChannel(SPI_HandleTypeDef* spiHandle, uint8_t channelNumber);

//enable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
//note: RX buffer size should be set
void enableDataPipe(SPI_HandleTypeDef* spiHandle, uint8_t pipeNumber);

//disable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
void disableDataPipe(SPI_HandleTypeDef* spiHandle, uint8_t pipeNumber);

//choose which datapipes to use
//note: that pipeNumber[0] should always be 1, because this pipe is used for acks
//note: RX buffer size should be set
void setDataPipeArray(SPI_HandleTypeDef* spiHandle, uint8_t pipeEnable[6]);

//set the size of the RX buffer in bytes
void setRXbufferSize(SPI_HandleTypeDef* spiHandle, uint8_t size);

//make sure interrupts for the TX functions are enabled
//and those for the RX functions not
void TXinterrupts(SPI_HandleTypeDef* spiHandle);;

//make sure interrupts for the RX functions are enabled
//and those for the TX functions not
void RXinterrupts(SPI_HandleTypeDef* spiHandle);

void enableAutoRetransmitSlow(SPI_HandleTypeDef* spiHandle);

//---------------------------------modes----------------------------------//

//power down the device. SPI stays active.
void powerDown(SPI_HandleTypeDef* spiHandle);

//go to standby. SPI stays active. consumes more power, but can go to TX or RX quickly
void powerUp(SPI_HandleTypeDef* spiHandle);

//device power up and start listening
void powerUpRX(SPI_HandleTypeDef* spiHandle);

//device power up, and be ready to receive bytes.
void powerUpTX(SPI_HandleTypeDef* spiHandle);


//--------------------------sending and receiving-------------------------//

//flush the TX buffer
void flushTX(SPI_HandleTypeDef* spiHandle);

//flush the RX buffer
void flushRX(SPI_HandleTypeDef* spiHandle);

//send a byte. only used in TX mode
//warning: after sending, the CE pin stays high.
//it should be put down manually when either MAX_RT or TX_DS is high
//this can be done using the powerUpTX function
//not doing this will cause the wireless module to stay on, which is a waste of energy.
void sendData(SPI_HandleTypeDef* spiHandle, uint8_t data[], uint8_t length);

//read a byte from the buffer. only used in RX mode
void readData(SPI_HandleTypeDef* spiHandle, uint8_t* receiveBuffer, uint8_t length);

void setLowSpeed(SPI_HandleTypeDef* spiHandle);


//---------------------------------debug----------------------------------//
//print all the registers
void printAllRegisters(SPI_HandleTypeDef* spiHandle);

//**********************application specific code*********************//
//

//------------------------------ initRX ------------------------------//
//reset and init
//mask right interrupts
//set frequency
//enable right data pipe
//set RX buffer size
//set RX address
//powerUpRX
void initRobo(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address);

//------------------------------ initTX ------------------------------//
//rest and init
//mask right interrupts
//set frequency
//enable ack pipe
//set RX buffer size
//set TX address + pipe 0 address
//powerUpTX
void initBase(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address[5]);

//---------------------------TODO TX loop----------------------------//
//get data from pc
//read address -> test
//change TX address to address from packet -> test
//send data
//wait for interrupt
//note timeout and clear interrupt bit
//clear interrupt bits in case of succesful transmission
//set CE low
//send ack data back to pc

//sendpacket is split in 2 parts, so it can be done on 1 microcontroller
//waiting for send done interrupt would halt the receiving part of the program
uint8_t sendPacketPart1(SPI_HandleTypeDef* spiHandle, uint8_t packet[8]);
void waitAck(SPI_HandleTypeDef* spiHandle, uint8_t roboID);

//------------------------TODO RX callback---------------------------//
//interrupt driven, should be executed when IRQ pin is low
//read data from buffer
//TODO decode data and put in struct
//TODO return struct
//TODO put interrupt low

void roboCallback(SPI_HandleTypeDef* spiHandle, dataPacket* dataStruct);

void printDataStruct(dataPacket* dataStruct);

//------------------------TODO RX get info---------------------------//
//TODO get struct from robot
//TODO encode struct in uint8_t array
//TODO put array as acknowledgement

#endif /* MYNRF24_H_ */



