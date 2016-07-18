#ifndef _UART_H
#define _UART_H

#include "stm32f4xx_conf.h"
#include "misc.h"
//#include <stdio.h>
#include <string.h>

#define MAX_UART_INPUT 100
#define MAX_UART_INPUT_MINUS_ONE 99
//extern char buffer6[MAX_UART_INPUT];
//extern uint8_t buffer6_index = 0;

void UART1_GPIO_Configuration();
void UART1_Configuration();
void UART1_RCC_Configuration();
void UART6_RCC_Configuration();
void UART6_GPIO_Configuration();
void UART6_Configuration();
void Init_UART1();
void Init_UART6();
void UART1_puts(char* s);
void UART6_puts(char* s);
void UART1_int(uint16_t num);
void UART1_ReadLine();
//void UART6_ReadLine();
void remote_ctrl(char *str);
void wifi_ctrl(char *str);

#endif
