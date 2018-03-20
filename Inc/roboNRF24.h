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


/*
 * I have the feeling that those structures should be defined
 * in a packet class... or packet library.
 * But it's better to have them here than having them in the general nRF24 library,
 * since it's apparently a specific structure which so far is only used by
 * the robot (not by the basestation).
 */
typedef struct dataPacket {
  uint8_t robotID; // 0 to 15
  uint16_t robotVelocity; //between 0 and 4095mm/s
  uint16_t movingDirection; // resolution: 2pi/512 radians
  uint8_t rotationDirection; //0 = cw; 1 = ccw;
  uint16_t angularVelocity; //0 to 2047 deg/s
  uint8_t kickForce; // 0 to 255
  uint8_t forced; // 0 = normal kick; 1 = forced kick
  uint8_t chipper; // 0 = kicker; 1 = chipper
  uint8_t kick; // 0 = do not kick; 1 = kick according to forced and chipper
  uint8_t driblerDirection; // 0 = cw; 1 = ccw;
  uint8_t driblerSpeed; // between 0 and 7
  uint16_t currentRobotVelocity;
  uint16_t currentMovingDirection; // resolution: 2pi/512 radians
  uint8_t currentRotationDirection; //0 = cw; 1 = ccw;
  uint16_t currentAngularVelocity; //0 to 2047 deg/s
  uint8_t videoDataSend;

} dataPacket;

struct ackPacket {
  uint8_t robotID;
  uint8_t succes;
};

extern dataPacket dataStruct;
/*
 * End of structures which might not belong here
 */

int8_t initRobo(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address);

void roboCallback(dataPacket* dataStruct);

void printDataStruct(dataPacket* dataStruct);


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
