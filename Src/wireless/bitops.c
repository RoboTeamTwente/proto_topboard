/*
 * bitops.c
 *
 *  Created on: Mar 13, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *   Bit Operation functions from the old myNRF24 library.
 *   I'm not a big fan of them since I think those kind of operations
 *   should be known by C programmers, but I will keep them here
 *   for whoever needs them.
 */

#include "bitops.h"

//set a specific bit in a byte to a 1 or a 0
uint8_t setBit(uint8_t byte, uint8_t position, uint8_t bitValue){
	/*
	 * A Bit consists of 8 Bits, where the right-most (least significant) Bit
	 * starts with position 0.
	 * This function allows to set (1) or clear (0) a speficif Bit of a byte (uint8_t).
	 *
	 * A read on "Bitwise Operators in C" is advised.
	 * We keep this function as a layer of abstraction and it is suggested to
	 * use this function where it can increase the readability of code.
	 */
	if(bitValue)
		return byte|(1<<position); //set bit
	else
		return byte&~(1<<position); //clear bit
}

//check if a specific bit in a byte is 1
uint8_t readBit(uint8_t byte, uint8_t position){
	return byte & (1<<position);
}



