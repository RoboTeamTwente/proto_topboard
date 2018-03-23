/*
' * myNRF24.c
 *
 *  Created on: 19 sep. 2016
 *      Author: Hans van-der-Heide
 *      Author: Ulf Stottmeister, March 2018
 */


/*
 * see datasheet for info:
 * https://www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf
 * Important pages:
 * page 48 command words to chip
 * figure 23 on page 49: spi operations
 * page 54 register map
 * figure 3 on page 21: state diagram
 */

#include "myNRF24.h"
#include "myNRF24basic.h"

//****************************high level library**************************//
//********************the user may use these functions********************//

//--------------------initialization and configuration--------------------//

//initialize the system:
//reset it and enable pipe 1 and 0
//set pipeWidth to 1
//flush TX and RX buffer
//arguments: SpiHandle and functions which implement pin setters for NSS and CE and reader for IRQ
int8_t NRFinit(SPI_HandleTypeDef* nrf24spiHandle, void (*nrf24nssHigh)(), void (*nrf24nssLow)(), void (*nrf24ceHigh)(), void (*nrf24ceLow)(), uint8_t (*nrf24irqRead)() ){
	//set references to pin setter functions
	nssHigh = nrf24nssHigh;
	nssLow = nrf24nssLow;
	ceHigh = nrf24ceHigh;
	ceLow = nrf24ceLow;

	//irq reader
	irqRead = nrf24irqRead;

	//global reference to spiHandle
	spiHandle = nrf24spiHandle;

	//reading the status register to check for errors with the SPI communication
	uint8_t status_reg = getStatusReg(); //if the read fails, it will return -1
	if(status_reg == -1) {
		return -1; // SPI error! Is the module connected and is the SPI configured properly?
	}

	//reset system

	//This sets all registers to their default values.
	//This is needed to ensure consistent behaviour when the MCU is soft-resetted
	//and the nRF module is not (e.g. after a firmware upload).
	softResetRegisters();
	clearInterrupts();

	//enable RX pipe 0 and 1, disable all other pipes
	writeReg(EN_RXADDR, ERX_P1);
	//alternatively you can write:
	//enableDataPipe(0);
	//enableDataPipe(1);
	//I'm not sure which way is more elegant.


	//The datasheet says about RX_PW_PX (any number for X):
	// "Number of bytes in RX payload in data pipe [X]."
	//The value can be read and written. So I wonder what it does when you write to it.
	//writeReg(RX_PW_P0, 12);
	//writeReg(RX_PW_P1, 12);

	flushRX();
	flushTX();

	return 0; //init successful
}

//reset all register values to reset values on page 54, datasheet
void softResetRegisters(){
	uint8_t multRegData[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

	// see page 54 and further for reset values
	writeReg(CONFIG, 0x08);
	writeReg(EN_AA, 0x3F);
	writeReg(EN_RXADDR, 0x03);
	writeReg(SETUP_AW, 0x03);
	writeReg(SETUP_RETR, 0x03);
	writeReg(RF_CH, 0x02);
	writeReg(RF_SETUP, 0x0E);
	writeReg(STATUS, 0x00);
	//register 0x08 and 0x09 are read only
	writeRegMulti(RX_ADDR_P0, multRegData, 5);

	for(int i = 0; i < 5; i++){multRegData[i] = 0xC2;}
	writeRegMulti(RX_ADDR_P1, multRegData, 5);
	writeReg(RX_ADDR_P2, 0xC3);
	writeReg(RX_ADDR_P3, 0xC4);
	writeReg(RX_ADDR_P4, 0xC5);
	writeReg(RX_ADDR_P5, 0xC6);
	for(int i = 0; i < 5; i++){multRegData[i] = 0xE7;}
	writeRegMulti(TX_ADDR, multRegData, 5);
	writeReg(RX_PW_P0, 0x00);
	writeReg(RX_PW_P1, 0x00);
	writeReg(RX_PW_P2, 0x00);
	writeReg(RX_PW_P3, 0x00);
	writeReg(RX_PW_P4, 0x00);
	writeReg(RX_PW_P5, 0x00);
	writeReg(FIFO_STATUS, 0x11);
	//reg 0x18 to 0x1B are undocumented test registers. Don't write to them!
	writeReg(DYNPD, 0x00);
	writeReg(FEATURE, 0x00);

}

int8_t getStatusReg() {
	return readReg(STATUS);
}

//set own address note: only data pipe 0 is used in this implementation
//returns 0 on success; -1 on error
int8_t setTXaddress(uint8_t address[5]){
	if(writeRegMulti(RX_ADDR_P0, address, 5) != 0) // set RX address pipe 0 for auto acks
		return -1; //error
	if(writeRegMulti(TX_ADDR, address, 5) != 0) // set TX address
		return -1; //error

	return 0; //success
}

//set the address you will send to
//pipe 0 is reserved for acks: its address always equals the TX address and is set with setTXaddress
//returns 0 on success; -1 on error
int8_t setRXaddress(uint8_t address[5], uint8_t pipeNumber){
	if(pipeNumber == 0 || pipeNumber > 5)
		return -1; //error. invalid pipeNumber

	return writeRegMulti(RX_ADDR_P0 + pipeNumber, address, 5);
}


//returns 0 on success; -1 on error
int8_t setFreqChannel(uint8_t channelNumber){
	if(channelNumber > 127)
		//TextOut("Error, max channelNumber = 127\n");
		return -1; //error: invalid channel Number

	//forward the return value of writeReg() to caller
	return writeReg(RF_CH, channelNumber);
}

//enable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
//returns 0 on success; -1 on error
int8_t enableDataPipe(uint8_t pipeNumber){
	if(pipeNumber > 5)
		return -1; //error: invalid pipeNumber

	uint8_t en_rxaddr_val = readReg(EN_RXADDR);
	en_rxaddr_val |= (1 << pipeNumber);

	return writeReg(EN_RXADDR, en_rxaddr_val);
}

//disable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
int8_t disableDataPipe(uint8_t pipeNumber){
	if(pipeNumber == 0 || pipeNumber > 5)
		return -1; //error: invalid pipeNumber

	uint8_t pipeEnableReg = readReg(EN_RXADDR);
	pipeEnableReg = setBit(pipeEnableReg, pipeNumber, 0);
	writeReg(EN_RXADDR, pipeEnableReg);// disable pipe
	writeReg(RX_PW_P0 + pipeNumber, 0); //set buffer size to 0;
	return 0;
}

//choose which datapipes to use
//Check out page 54 (Register Map) in the datasheet.
//Set the Bits according to the datapipes in pipeEnable.
void setDataPipes(uint8_t pipeEnable){
	writeReg(EN_RXADDR, pipeEnable);
}

//set the size of the RX buffer in bytes
int8_t setRXbufferSize(uint8_t size){
	if(size > 32){
		//TextOut("Error: size can not be bigger than 32 bytes\n");
		return -1; //error: size too big
	}

	uint8_t rx_addr_reg = readReg(EN_RXADDR);

	//for every activated data pipe in EN_RXADDR, set the buffer size in RX_PW_PX to "size" amount of bytes.
	for(uint8_t i = 0; i < 6; i++){
		if(readBit(rx_addr_reg, i)){
			writeReg(RX_PW_P0 + i, size);
		}
		else{
			writeReg(RX_PW_P0 + i, 0);
		}
	}

	return 0; //success
}

//make sure interrupts for the TX functions are enabled
//and those for the RX functions not
void TXinterrupts(){
	uint8_t config_reg = readReg(CONFIG);
	/* Register 0x00 (CONFIG)
	 * Bit 6: MASK_RX_DR
	 * Bit 5: MASK_TX_DS
	 * Bit 4: MASK_MAX_RT
	 *
	 * For those Bits:
	 * 1 means: disabled; Interrupt not reflected on IRQ Pin
	 * 0 means: enabled; Interrupt on IRQ Pin as active low
	 *
	 */
	config_reg = setBit(config_reg, 6, 1); //disable for RX_DR
	config_reg = setBit(config_reg, 5, 0); //enable for TX_DS
	config_reg = setBit(config_reg, 4, 0); //enable for MAX_RT

	//another way of writing that:
	//config_reg |= MASK_RX_DR;   //1
	//config_reg &= ~MASK_TX_DS;  //0
	//config_reg &= ~MASK_MAX_RT; //0


	writeReg(CONFIG, config_reg);
}

//make sure interrupts for the RX functions are enabled
//and those for the TX functions not
void RXinterrupts(){
	uint8_t reg_config = readReg(CONFIG);
	reg_config = setBit(reg_config, 6, 0);
	reg_config = setBit(reg_config, 5, 1);
	reg_config = setBit(reg_config, 4, 1);
	writeReg(CONFIG, reg_config);
}

//---------------------------------modes----------------------------------//

//power down the device. SPI stays active.
void powerDown(){
	ceLow(); //go to standby mode
	uint8_t reg_config = readReg(CONFIG);

	//clear power bit: bit 2 to 0
	reg_config = setBit(reg_config, 2, 0);

	writeReg(CONFIG, reg_config);
}

//go to standby. SPI stays active. consumes more power, but can go to TX or RX quickly
void powerUp(){
	ceLow();

	uint8_t reg_config = readReg(CONFIG);

	//set power up bit: bit 2 of reg 0.
	reg_config = reg_config | PWR_UP;

	writeReg(CONFIG, reg_config);

}

//device power up and start listening
void powerUpTX(){
	ceLow(); //stay in standby mode, until there is data to send.
	uint8_t reg_config = readReg(CONFIG);

	//flush TX buffer
	flushTX();

	//set power up bit: bit 1 of reg 0
	reg_config = setBit(reg_config, 1, 1);
	//set is Primary Transmitter (PRIM_RX Bit to 0)
	reg_config = setBit(reg_config, 0, 0);


	writeReg(CONFIG, reg_config);

}

//device power up, and be ready to receive bytes.
void powerUpRX(){
	flushRX();
	writeReg(CONFIG, PRIM_RX|PWR_UP);
	//put CE pin high ->  start listening
	ceHigh();
}


//--------------------------sending and receiving-------------------------//

//flush the TX buffer
void flushTX(){
	nssLow();
	uint8_t sendData = NRF_FLUSH_TX; //FLUSH_TX
	HAL_SPI_Transmit(spiHandle, &sendData, 1, 100);
	nssHigh();

}

//flush the RX buffer
void flushRX(){
	nssLow();
	uint8_t sendData = NRF_FLUSH_RX; //FLUSH_RX
	HAL_SPI_Transmit(spiHandle, &sendData, 1, 100);
	nssHigh();
}

//send a byte. only used in TX mode
//warning: after sending, the CE pin stays high.
//it should be put down manually when either MAX_RT or TX_DS is high
//this can be done using the powerUpTX function
//not doing this will cause the wireless module to stay on, which is a wast of energy.
void sendData(uint8_t data[], uint8_t length){
	//TextOut("before sending\n");

	ceLow();

	flushTX();

	nssLow();
	uint8_t spi_command = NRF_W_TX_PAYLOAD; // W_TX_PAYLOAD
	uint8_t spi_timeout = 100;
	HAL_SPI_Transmit(spiHandle, &spi_command, 1, spi_timeout);


	HAL_SPI_Transmit(spiHandle, data, length, spi_timeout);
	nssHigh();


	//send over air

	ceHigh();
}

//read a byte from the buffer. only used in RX mode
//Edit: the datasheet say it is used in RX mode. It doesn't say it is only(!) used in RX mode.
//     Afaik, you also need to use it in TX mode when you want to read an ACK payload
void readData(uint8_t* receiveBuffer, uint8_t length){
	nssLow();
	uint8_t command = NRF_R_RX_PAYLOAD;
	HAL_SPI_Transmit(spiHandle, &command, 1, 100);
	HAL_SPI_Receive(spiHandle, receiveBuffer, length, 100);
	nssHigh();
}


//read data from the RX FIFO until it's empty or until max_length has been reached
//this function hasn't been tested yet.
int8_t readAllData(uint8_t* receiveBuffer, uint8_t max_length){
	//while there is data in the RX FIFO, read byte per byte
	uint8_t bytes_read = 0;
	while((readReg(FIFO_STATUS) & RX_EMPTY) != 0) {
		nssLow();

		uint8_t command = NRF_R_RX_PAYLOAD; //R_RX_PAYLOAD
		HAL_SPI_Transmit(spiHandle, &command, 1, 100);

		HAL_SPI_Receive(spiHandle, receiveBuffer++, 1, 100);

		nssHigh();
		bytes_read++;

		if(bytes_read >= max_length)
			return -1; //stop here and give an error. Only the amount of bytes specified by max_length will be available.
	}
	return 0; //success
}

void setLowSpeed(){
	uint8_t rfsetup_reg = readReg(RF_SETUP);
	rfsetup_reg |= RF_DR_LOW;
	rfsetup_reg &= ~RF_DR_HIGH;
	writeReg(RF_SETUP, rfsetup_reg);
}

void enableAutoRetransmitSlow(){
	uint8_t arc = 0x11; //auto retransmit count (ARC); how many packets to retransmit before giving up
	writeReg(SETUP_RETR, arc&7);
}

//returns the payloadlength of a received packet when received
//on a datapipe with DPL (Dynamic Payload Length)
uint8_t getDynamicPayloadLength() {
	uint8_t bytesReceived;

	nssLow();
	uint8_t command = NRF_R_RX_PL_WID; //read rx payload length
	HAL_SPI_Transmit(spiHandle, &command, 1, 100);
	HAL_SPI_Receive(spiHandle, &bytesReceived, 1, 100);
	nssHigh();

	return bytesReceived;
}

//returns the payload length of a received packet for data pipes
//which don't use DPL (dynamic payload length)
uint8_t getStaticPayloadLength(uint8_t dataPipeNo) {
	if(dataPipeNo > 5)
		return 0;
	return readReg(RX_PW_P0 + dataPipeNo);
}
//write ACK payload to module
//this payload will be included in the payload of ACK packets when automatic acknowledgments are activated
int8_t writeACKpayload(uint8_t* payloadBytes, uint8_t payload_length, uint8_t pipeNo) {
	//This function should be called as often as a packet was received (either before or after reception),
	//because the module can only hold up to 3 ACK packets.
	//It will use up one of the packets as a response when it receives a packet.
	//See: https://shantamraj.wordpress.com/2014/11/30/auto-ack-completely-fixed/ (visited 13th March, 2018)

	//you may want to call writeACKpayload() in the procedure which reads a packet


	if(readReg(FIFO_STATUS) & FIFO_STATUS_TX_FULL) {
		//flushTX(); //will ensure that we don't overflow with 3 ACK packets or more
		return -1; //error: FIFO full
	}

	nssLow();

	uint8_t spi_command = NRF_W_ACK_PAYLOAD | pipeNo;
	//activate spi command
	if(HAL_SPI_Transmit(spiHandle, &spi_command,1, 100) != HAL_OK)
		return -1; //HAL/SPI error

	//transmit values for spi command (send payload to nRF module)
	if(HAL_SPI_Transmit(spiHandle, payloadBytes, payload_length, 100) != HAL_OK)
		return -1; //HAL/SPI error

	nssHigh();

	return 0; //success
}


//called by the basestation to receive ack data
//if there is data, it will be stored in the given ack_payload array
//returns 1 when there is a payload
//on error, returns 0, -1 or -2.
int8_t getAck(uint8_t* ack_payload) {
	/*
	 * The following paragraph assumes that ACKs are enabled (e.g. we expect packets to be answered with ACKs).
	 *
	 * When we send a packet we would need to wait for TX_DS to be set (indicating that an ACK packet was received)
	 * and RX_DR to be set (indicating that the ACK has a payload).
	 * TX_DS and RX_DR are flags which would cause an interrupt on the IRQ pin,
	 * when the CONFIG register is set up accordingly.
	 *
	 * With TXinterrupts(), the TX_DS flag and the MAX_RT flag is set, causing an interrupt
	 * when a packet is sent and answered with an ACK.
	 * The flag RX_DR is not set in this function, since we expect that TX_DS (successful transmission)
	 * will be set before we receive an ACK.
	 * Eventhough an ACK payload will not trigger an interrupt by itself, we should be able to
	 * check the existence of an ACK payload by reading the RX_DR flag.
	 *
	 */
	if(irqRead()){
		//uint8_t succesful = 0;
		uint8_t status_reg = readReg(STATUS);
		ceLow();
		if(status_reg & MAX_RT){
			//maximum retransmissions reached without receiving an ACK
			//we don't care about dropped packets, we could as well just not make this event cause an interrupt,
			//but we might want to decide to retransmit packets in a later implementation.
			//For now I will just reset the flag for this interrupt and go on
			writeReg(STATUS, readReg(STATUS) & MAX_RT);
			flushTX();
			return -1;
		}
		else if(status_reg & TX_DS & RX_DR){
			//packet (successfully) transmitted
			//an ACK with payload was received and put in the RX FIFO of the nRF module
			//succesful = 1;

			//read payload here!

			 //TODO: I don't really like to set it to a fixed length here. I would rather like to read a global macro (#define PAYLOAD_LENGTH ?)
			readData(ack_payload, 12);

			//TODO: only clear relevant interrups (TX_DS and RX_DR).
			clearInterrupts();

			return 1; //success
		}
		else if((status_reg & TX_DS) && !(status_reg & RX_DR)){
			//packet transmitted, but we received no ack payload in return.
			//either we aren't using auto-acknowledgements or the receiver sent back an empty ack (ack with no payload)
			return -1;
		}
		else {
			//This case would be reached when RX_DR is high (indicating that we received a packet),
			//but TX_DS is low (indicating that we didn't send anything successfully prior to this reception).
			//That means: a packet was addressed to us, but we weren't waiting for it.
			//This packet is not an ACK payload, but a regular packet.
			return -1;
		}

	} else {
		//there was no interrupt yet
		return -2;
	}
	//TextOut("00"); //could be sent as an error. needs to be agreed on with the software communicating with the basestation
	return 0; //no ack payload received
}




