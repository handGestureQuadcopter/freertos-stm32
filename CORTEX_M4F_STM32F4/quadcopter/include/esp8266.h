#ifndef _ESP8266_H
#define _ESP8266_H

#include <stdint.h>
#include <string.h>
//#include <time.h>
//#include <sys/time.h>
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "task.h"

#define TRUE 1
#define FALSE 0

//extern char buffer6[MAX_UART_INPUT];
//extern uint8_t buffer6_index = 0;

//void ESP8266Task(void *pvParameters);
void esp8266_configure();
void Enable_TIM3_INTERRUPT();
uint8_t buffer6_ready();
uint8_t buffer6_read_respond(const char * rsp);
void set_isOK(uint8_t set);
uint8_t get_isOK();
uint8_t buffer6_search(const char * test);
void sendAT();
void putbuffer6();
void UART6_ReadLine();
uint8_t esp8266_read_response(const char * rsp);
uint8_t esp8266_search_buffer6(const char * test);
void esp8266_clear_buffer6();
uint8_t esp8266_buffer6_available();
//uint8_t ESP8266_Task_Creat();

#endif
