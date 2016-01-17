#include "stm32f4xx_conf.h"
#include <stdio.h>
#include <stdlib.h>

#define MAX_UART_INPUT 50

void UART1_GPIO_Configuration();
void UART1_Configuration();
void UART1_RCC_Configuration();
void Init_UART1();
void UART1_puts(char* s);
void UART1_int(uint16_t num);
void UART1_ReadLine();
