#ifndef __USART_H__
#define __USART_H__
#include "stm32f4xx_conf.h"
#include "misc.h"
#include <stdint.h>

#define MAX_UART_INPUT 50


void USART1_Configuration(void);

void USART1_puts(char*);

void USART1_ReadLine();


#endif
