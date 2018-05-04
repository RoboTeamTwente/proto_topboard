/*
 * robotNRF24.h
 *
 *  Created on: Mar 16, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *   Specific commands for using the nRF24 wireless module
 *   with our robot.
 */

#ifndef ROBONRF24_H_
#define ROBONRF24_H_


#include "myNRF24.h"
#include "packing.h"

//structs to be used in combination with TX and RX packets
roboData receivedRoboData; //unwrapped packet from the basestation
roboAckData preparedAckData; //ready-to-wrap data to build a packet for sending an ACK

//The radio channel for the nRF24 wireless module as agreed on with the organizational team / initiatiors of the RoboCup.
//A team in the RoboCup should select one channel and inform "the RoboCup" about the used frequencies (channels) to avoid
//any distrubance with other teams (e.g. opponents).
#define RADIO_CHANNEL 78

void fillAckData(uint8_t ackDataLength);

int8_t initRobo(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address);

int8_t roboCallback(uint8_t localRobotID);

//void printDataStruct(dataPacket* dataStruct);


/*
 * Pin functions
 */

//put the nss pin corresponding to the SPI used high
void nrf24nssHigh();

//put the nss pin corresponding to the SPI used low
void nrf24nssLow();

//put the ce pin corresponding to the SPI used high
void nrf24ceHigh();

//put the ce pin corresponding to the SPI used low
void nrf24ceLow();

uint8_t nrf24irqRead();

#endif /* ROBONRF24_H_ */
