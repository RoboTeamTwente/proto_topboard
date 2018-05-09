/*
 * bitops.h
 *
 *  Created on: Mar 13, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *  Interface for bitops.c
 *   Bit Operation functions from the old myNRF24 library.
 *   I'm not a big fan of them since I think those kind of operations
 *   should be known by C programmers, but I will keep them here
 *   for whoever needs them.
 */

#ifndef BITOPS_H_
#define BITOPS_H_

#include <inttypes.h>

//set a specific bit in a byte to a 1 or a 0
uint8_t setBit(uint8_t byte, uint8_t position, uint8_t bitValue);

//check if a specific bit in a byte is 1
uint8_t readBit(uint8_t byte, uint8_t position);

#endif /* BITOPS_H_ */
