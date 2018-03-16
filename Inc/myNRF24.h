/*
 * myNRF24.h
 *
 *  Created on: 19 sep. 2016
 *      Author: Hans-van-der-Heide
 */

//TODO SPI timeout

#ifndef MYNRF24_H_
#define MYNRF24_H_

//#define PI 3.14159

//#include "TextOut.h"
//#include "gpio.h"
//#include "stm32f3xx_hal.h"
#include "spi.h"
#include <inttypes.h>

#include "myNRF24basic.h"


//--------------------initialization and configuration--------------------//

//initialize the system:
//reset it and enable pipe 1 and 0
//set pipeWith to 1
//flush TX and RX buffer
void NRFinit(SPI_HandleTypeDef* spiHandle, void (*nrf24nssHigh)(), void (*nrf24nssLow)(), void (*nrf24ceHigh)(), void (*nrf24ceLow)(), uint8_t (*nrf24irqRead)() );

//reset to reset value on page 54
void softResetRegisters(SPI_HandleTypeDef* spiHandle);


//set the address you will send to
int8_t setTXaddress(SPI_HandleTypeDef* spiHandle, uint8_t address[5]);

//set own address note: only data pipe 0 is used in this implementation
int8_t setRXaddress(SPI_HandleTypeDef* spiHandle, uint8_t address[5], uint8_t pipeNumber);

//set a frequency channel
int8_t setFreqChannel(SPI_HandleTypeDef* spiHandle, uint8_t channelNumber);

//enable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
//note: RX buffer size should be set
int8_t enableDataPipe(SPI_HandleTypeDef* spiHandle, uint8_t pipeNumber);

//disable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
int8_t disableDataPipe(SPI_HandleTypeDef* spiHandle, uint8_t pipeNumber);

//choose which datapipes to use
//note: that pipeNumber[0] should always be 1, because this pipe is used for acks
//note: RX buffer size should be set
void setDataPipes(SPI_HandleTypeDef* spiHandle, uint8_t pipeEnable);

//set the size of the RX buffer in bytes
int8_t setRXbufferSize(SPI_HandleTypeDef* spiHandle, uint8_t size);

//make sure interrupts for the TX functions are enabled
//and those for the RX functions not
void TXinterrupts(SPI_HandleTypeDef* spiHandle);;

//make sure interrupts for the RX functions are enabled
//and those for the TX functions not
void RXinterrupts(SPI_HandleTypeDef* spiHandle);

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

//write ACK payload to module
//this payload will be included in the payload of ACK packets when automatic acknowledgments are activated
int8_t writeACKpayload(SPI_HandleTypeDef* spiHandle, uint8_t* payloadBytes, uint8_t payload_length);

//called by the basestation to receive ack data
//if there is data, it will be stored in the given ack_payload array
//returns 1 when there is a payload
//on error, returns 0, -1 or -2.
int8_t getAck(SPI_HandleTypeDef* spiHandle, uint8_t* ack_payload);


#endif /* MYNRF24_H_ */
