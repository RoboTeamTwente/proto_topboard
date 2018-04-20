/*
 * myNRF24basic.h
 *
 *  Created on: Mar 13, 2018
 *      Author: Ulf Stottmeister
 *
 *
 *  This is the interface for the low level functions on the nRF24L01 wireless module.
 */

#ifndef MYNRF24BASIC_H_
#define MYNRF24BASIC_H_

#include "bitops.h"  //for setBit() and readBit() in myNRF24.c
#include <inttypes.h> //for uint8_t and so on
#include "spi.h" //for commands like HAL_SPI_TransmitReceive and the type SPI_HandleTypeDef
//#include "myNRF24.h"

//defining SPI Commands (datasheet, page 48)
#define NRF_R_REGISTER 0x00 //000AAAAA, AAAAA= Register
#define NRF_W_REGISTER 0b00100000 //001AAAAA, AAAAA= Register
#define NRF_R_RX_PAYLOAD 0b01100001
#define NRF_W_TX_PAYLOAD 0b10100000
#define NRF_FLUSH_TX 0b11100001
#define NRF_FLUSH_RX 0b11100010
#define NRF_REUSE_TX_PL 0b11100011
#define NRF_R_RX_PL_WID 0b01100000
#define NRF_W_ACK_PAYLOAD 0b10101000 //10101PPP, PPP: pipenumber
#define NRF_W_TX_PAYLOAD_NO_ACK 0b10110000
#define NRF_NOP 0xff


//defining registers
//see datasheet page 54 and following
enum nrfRegister {
	CONFIG, //0x00
	EN_AA, //0x01
	EN_RXADDR, //0x02
	SETUP_AW, //0x03
	SETUP_RETR, //0x04
	RF_CH, //0x05
	RF_SETUP, //0x06
	STATUS, //0x07
	OBSERVE_TX, //0x08
	RPD, //0x09
	RX_ADDR_P0, //0x0A
	RX_ADDR_P1, //0x0B
	RX_ADDR_P2, //0x0C
	RX_ADDR_P3, //0x0D
	RX_ADDR_P4, //0x0E
	RX_ADDR_P5, //0x0F
	TX_ADDR, //0x10
	RX_PW_P0, //0x11
	RX_PW_P1, //0x12
	RX_PW_P2, //0x13
	RX_PW_P3, //0x14
	RX_PW_P4, //0x15
	RX_PW_P5, //0x16
	FIFO_STATUS, //0x17
	DYNPD=0x1C, //0x1C
	FEATURE //0x1D

};

//defining bit flags for each register with single-bit fields
//multi-bit fields are not included -- when you use them, you should
//definitely look it up in the datasheet

enum CONFIG_FLAG {
	MASK_RX_DR = 1<<6,
	MASK_TX_DS = 1<<5,
	MASK_MAX_RT = 1<<4,
	EN_CRC = 1<<3,
	CRC0 = 1<<2,
	PWR_UP = 1<<1,
	PRIM_RX = 1<<0
};

enum EN_AA_FLAG {
	ENAA_P5 = 1<<5,
	ENAA_P4 = 1<<4,
	ENAA_P3 = 1<<3,
	ENAA_P2 = 1<<2,
	ENAA_P1 = 1<<1,
	ENAA_P0 = 1<<0
};

enum EN_RXADDR_FLAG {
	ERX_P5 = 1<<5,
	ERX_P4 = 1<<4,
	ERX_P3 = 1<<3,
	ERX_P2 = 1<<2,
	ERX_P1 = 1<<1,
	ERX_P0 = 1<<0
};

//enum SETUP_AW_FLAG {
	//AW 1:0 (AW is Bit 1 down to 0)
//};

//enum SETUP_RETR_FLAG {
	//ARD 7:4 (ARD is Bit 7 down to 4)
	//ARC 3:0
//};

//enum RF_CH_FLAG {
	//RF_CH 6:0
//};

enum RF_SETUP_FLAG {
	CONT_WAVE = 1<<7,
	//6 reserved
	RF_DR_LOW = 1<<5,
	PLL_LOCK = 1<<4,
	RF_DR_HIGH = 1<<3,
	//RF_PWR 2:1
	//0 Obsolete
};

enum STATUS_FLAG {
	RX_DR = 1<<6, //packet arrived
	TX_DS = 1<<5, //packet (successfully) transmitted
	MAX_RT = 1<<4, //packet did not arrive (no ACK after retransmissions)
	//RX_P_NO 3:1  //pipe number where a new packet arrived
	STATUS_TX_FULL = 1<<0, //check if the TX buffer is full
};

//enum OBSERVE_TX_FLAG {
	//PLOS_CNT 7:4
	//ARC_CNT 3:0
//};

//enum RPD_FLAG {
//	RPD = 1<<0,
//};


//skipping a few registers here, since they are multi-bit registers anyway
//RX_ADDR_P0
//RX_ADDR_P1
//RX_ADDR_P2
//RX_ADDR_P3
//RX_ADDR_P4
//RX_ADDR_P5
//TX_ADDR


//enum RX_PW_PX_FLAG {
	//RX_PW_PX 5:0
//};

enum FIFO_STATUS_FLAG {
	TX_REUSE = 1<<6,
	FIFO_STATUS_TX_FULL = 1<<5,
	TX_EMPTY = 1<<4,
	//3:2 reserved
	RX_FULL = 1<<1,
	RX_EMPTY = 1<<0,
};

enum DYNPD_FLAG {
	DPL_P5 = 1<<5,
	DPL_P4 = 1<<4,
	DPL_P3 = 1<<3,
	DPL_P2 = 1<<2,
	DPL_P1 = 1<<1,
	DPL_P0 = 1<<0,
};

enum FEATURE_FLAG {
	EN_DPL = 1<<2,
	EN_ACK_PAY = 1<<1,
	EN_DYN_ACK = 1<<0, //enables the W_TX_PAYLOAD_NOACK SPI command
};


extern SPI_HandleTypeDef* spiHandle;

/*
 * Below we have pointers to functions implementing the pin setters.
 * The implementations are usually as simple as setting a GPIO pin high or low
 * by utilizing HAL_GPIO_WritePin(PORT, PIN, VALUE);
 * We are using this approach, because the robot apparently does not use
 * the CE pin. Therefore it will leave the function body empty
 * (but it still needs to declare it).
 */
//put the nss pin corresponding to the SPI used high
extern void (*nssHigh)();
//put the nss pin corresponding to the SPI used low
extern void (*nssLow)();
//put the ce pin corresponding to the SPI used high
extern void (*ceHigh)();
//put the ce pin corresponding to the SPI used low
extern void (*ceLow)();

//reading the irq pin state
extern uint8_t (*irqRead)();

//read the interrupt pin
//uint8_t irqRead(SPI_HandleTypeDef* spiHandle);

//returns 0 on success; -1 on error
int8_t clearInterrupts();


//write to a register
//returns 0 on success; -1 on error
int8_t writeReg(uint8_t reg, uint8_t data);

//write to a multi-byte register
//returns 0 on success; -1 on error
int8_t writeRegMulti(uint8_t reg, uint8_t* pdata, uint8_t size);


//read a register
int8_t readReg(uint8_t reg);

//read a multi-byte register
//output will be stored in the array dataBuffer
//returns 0 on success; -1 on error
int8_t readRegMulti(uint8_t reg, uint8_t* dataBuffer, uint8_t size);



#endif /* MYNRF24BASIC_H_ */
