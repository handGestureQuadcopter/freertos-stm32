#include "stm32f4xx_conf.h"
#include <stdio.h>
#include <stdlib.h>

void UART1_GPIO_Configuration();
void UART1_Configuration();
void UART1_RCC_Configuration();
void Init_UART1();
void UART1_puts(char* s);
void UART1_int(uint16_t num);
void uart1command();
void UART1Task(void *pvParameters);
