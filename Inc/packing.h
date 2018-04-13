/*
 * packing.h
 *
 *  Created on: Mar 27, 2017
 *      Author: gebruiker
 *      (Co-)Author: Ulf Stottmeister, April 2018
 */

#ifndef PACKING_H_
#define PACKING_H_

#include <inttypes.h>

/*
 * A data struct which is easy to work with
 * when accessing variables.
 * It needs to be converted before it can be
 * transmitted, though.
 */

typedef struct roboData{
   uint8_t id:5;
   int16_t rho:11;
   int16_t theta:11;
   uint8_t driving_reference:1;
   uint8_t use_cam_info:1;
   int16_t velocity_angular:9;
   uint8_t debug_info:1;
   uint8_t do_kick:1;
   uint8_t do_chip:1;
   uint8_t kick_chip_forced:1;
   uint8_t kick_chip_power:8;
   uint8_t velocity_dribbler:8;
   uint8_t geneva_drive_state:3;
   uint16_t cam_position_x:13;
   uint16_t cam_position_y:13;
   uint16_t cam_rotation:11;
} roboData;

//between 11 and 23 Bytes, ideally
typedef struct roboAckData{
	//regular fields: 11 Bytes
	uint8_t roboID:5;
	uint8_t wheelLeftFront:1;
	uint8_t wheelRightFront:1;
	uint8_t wheelLeftBack:1;
	uint8_t wheelRightBack:1;
	uint8_t genevaDriveState:1;
	uint8_t batteryState:1;
	int16_t xPosRobot:13;
	int16_t yPosRobot:13;
	int16_t rho:11;
	int16_t theta:11;
	int16_t orientation:11;
	int16_t angularVelocity:11;
	uint8_t ballSensor:7;

	//extra fields (add 12 Bytes)
	uint32_t xAcceleration;
	uint32_t yAcceleration;
	uint32_t angularRate;

} roboAckData;


void robotDataToPacket(roboData *input, uint8_t output[13]);
void packetToRoboData(uint8_t input[13], roboData *output);
void roboAckDataToPacket(roboAckData *input, uint8_t output[23]);
void ackPacketToRoboAckData(uint8_t input[23], uint8_t packetlength, roboAckData *output);


#endif /* PACKING_H_ */

