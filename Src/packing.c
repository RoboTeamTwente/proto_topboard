/*
 * packing.c
 *
 *  Created on: Mar 27, 2017
 *      Author: gebruiker
 */

#include "packing.h"

/**
 * Packet format, inspired by the (old?) RoboJackets protocol.
 *
 * Byte     Config      Description
 * 1        aaaabbbb    aaaa: Robot ID, bbbb: Robot velocity
 * 2        bbbbbbbb    bbbbbbbb: Robot velocity, 0 - 4095 (mm/s)
 * 3        cccccccc    cccccccc: Moving direction, resolution 2 * pi / 512
 * 4        000cdeee    c: Moving direction, d: Rotation direction,
 * 5        eeeeeeee    eeeeeeeeeee: Angular velocity, 0-2047 (deg/s)
 * 6        ffffffff    ffffffff: Kick force, 0 - 255
 * 7        0ghijkkk    g: whether or not to kick
 *                      h: whether or not to chip
 *                      i: forced or not
 *                      j: counterclockwise dribbler
 *                      kkk: dribbler speed, 0 - 7
 */

void createRobotPacket(int id, int robot_vel, int ang, uint8_t rot_cclockwise, int w_vel, uint8_t kick_force, uint8_t do_kick, uint8_t chip, uint8_t forced, uint8_t dribble_cclockwise, uint8_t dribble_vel, uint8_t* byteArr){
	/*
	 * byteArr is a very generic name...
	 * Isn't that the array with the packet data to be sent with the NRF24?
	 */


    // First nibble are the robot id
    // Second nibble are the third nibble of robot velocity
    byteArr[0] = (((id & 15) << 4) | ((robot_vel >> 9) & 15));
    // First and second nibble of robot velocity
    byteArr[1] = (robot_vel >> 1);
    // Second to ninth bit of moving direction
    byteArr[2] = (robot_vel & 1) << 7 | (ang >> 2);;

    //sprintf(smallStrBuffer, "byteArr[2] hex: %x\n", byteArr[2]);
    //TextOut(smallStrBuffer);

    // First bit of moving direction
    // Then bit that designates clockwise rotation or not
    // Last three bits of angular velocity

    byteArr[3] = ((ang & 3) << 6)
                | (rot_cclockwise << 3)
				| ((w_vel >> 8) & 7);
    // First two nibbles of angular velocity
    byteArr[4] = (w_vel);
    // Just plug in the byte of kick-force
    byteArr[5] = kick_force;
    // gggg = 0, 0, chip = 1 kick = 0, forced = 1 normal = 0
    // First the chip and forced bools, then a bool that designates
    // a clockwise dribbler, and then three bits to designate dribble velocity
    byteArr[6] = (do_kick << 6)
                    | (chip << 5)
                    | (forced << 4)
                    | (dribble_cclockwise << 3)
					| (dribble_vel & 7);
}



