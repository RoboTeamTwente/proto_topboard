/*
 * PuttyInterface.h
 *
 *  Created on: Sep 29, 2017
 *      Author: Leon
 */

#ifndef PUTTYINTERFACE_H_
#define PUTTYINTERFACE_H_

#define PUTTY_USART

#ifdef PUTTY_USART
#include "usart.h"
#else
#include "usbd_cdc_if.h"
#endif /* PUTTY_USART */
#include <stdint.h>
#include <string.h>

// amount of commands that will be remembered
#define COMMANDS_TO_REMEMBER 16
#define MAX_COMMAND_LENGTH   64
#define uprintf(...) { sprintf(smallStrBuffer, __VA_ARGS__); \
	TextOut(smallStrBuffer); }

#ifdef PUTTY_USART
#define huartx huart3
#endif /* PUTTY_USART */

// function that will be called when HandlePcInput is done.
typedef void (*HandleLine)(char * input);

extern char smallStrBuffer[1024];

// Print string str to the pc
// str is the string to print
void TextOut(char *str);

// Transmit raw data
// data[] is the data,
// length is the length of the data
void HexOut(uint8_t*, uint8_t);
// This function deals with input by storing values and calling functoni func with a string of the input when its done
// also handles up and down keys
// input is a pointer to the characters to handle,
// n_chars is the amount of chars to handle
// func is the function to call when a command is complete
void HandlePcInput(char * input, size_t n_chars, HandleLine func);

#endif /* PUTTYINTERFACE_H_ */
