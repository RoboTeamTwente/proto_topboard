/*
 * packing.h
 *
 *  Created on: Mar 27, 2017
 *      Author: gebruiker
 */

#ifndef PACKING_H_
#define PACKING_H_



#endif /* PACKING_H_ */

#include "stm32f3xx_hal.h"


void createRobotPacket(int id, int robot_vel, int ang, uint8_t rot_cclockwise, int w_vel, uint8_t kick_force, uint8_t do_kick, uint8_t chip, uint8_t forced, uint8_t dribble_cclockwise, uint8_t dribble_vel, uint8_t* byteArr);
