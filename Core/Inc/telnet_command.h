/*
 * telnet_command.h
 *
 *  Created on: Nov 30, 2023
 *      Author: juand
 */

#ifndef INC_TELNET_COMMAND_H_
#define INC_TELNET_COMMAND_H_
#include "main.h"
#include "stm32l4xx_ll_usart.h"

uint8_t telnet_receive(USART_TypeDef *USARRx);
void telnet_transmit(USART_TypeDef *USARTx, const char *str);

#endif /* INC_TELNET_COMMAND_H_ */
