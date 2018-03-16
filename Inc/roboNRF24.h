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

//The radio channel for the nRF24 wireless module as agreed on with the organizational team / initiatiors of the RoboCup.
//A team in the RoboCup should select one channel and inform "the RoboCup" about the used frequencies (channels) to avoid
//any distrubance with other teams (e.g. opponents).
#define RADIO_CHANNEL 78


void initRobo(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address);

void roboCallback(SPI_HandleTypeDef* spiHandle, dataPacket* dataStruct);

void printDataStruct(dataPacket* dataStruct);




#endif /* ROBONRF24_H_ */
