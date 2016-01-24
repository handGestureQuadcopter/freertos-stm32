#ifndef __USART_H__
#define __USART_H__
#include "stm32f4xx_conf.h"
#include "misc.h"

#define MAX_UART_INPUT 50

void USART6_Configuration(void);

void USART6_puts(char*);

void UART6_ReadLine();

#endif
