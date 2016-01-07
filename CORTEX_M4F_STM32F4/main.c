/**
  ******************************************************************************
  * @file    Template/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-September-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32f4xx_conf.h"

#define MaxPulse 65535
#define MinPulse 0

void prvInit()
{
	//LCD init
	IOE_Config();
	LTDC_Cmd( ENABLE );

	//Button
	STM_EVAL_PBInit( BUTTON_USER, BUTTON_MODE_GPIO );
}
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

    
    // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13 |GPIO_Pin_14| GPIO_Pin_15; //PD12->LED3 PD13->LED4 PD14->LED5 PD15->LED6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure );

    /* UART config */
    GPIO_StructInit(&GPIO_InitStructure);

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   // USART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // USART1_RX
}
void RCC_Configuration(void)
{
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE );
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE );

   /* for uart */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
}
void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // Let PWM frequency equal 50Hz.
    TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInitStruct.TIM_Period = 480 - 1;   
    TIM_TimeBaseInitStruct.TIM_Prescaler = 1000 - 1;  
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;        
    TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );
        
    TIM_OCStructInit( &TIM_OCInitStruct );
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
        
    // Initial duty cycle equals 0%. Value can range from zero to 65535.
    //TIM_Pulse = TIM4_CCR1 register (16 bits)
    TIM_OCInitStruct.TIM_Pulse = 0; 
    TIM_OC1Init(TIM4, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OCInitStruct.TIM_Pulse = 0; 
    TIM_OC2Init(TIM4, &TIM_OCInitStruct);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OCInitStruct.TIM_Pulse = 0; 
    TIM_OC3Init(TIM4, &TIM_OCInitStruct);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OCInitStruct.TIM_Pulse = 0; 
    TIM_OC4Init(TIM4, &TIM_OCInitStruct);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 

    TIM_Cmd( TIM4, ENABLE );
}

void USART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

void USART1_puts(char* s)
{
    while(*s != '\0') {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s);
        s++;
    }
}

void USART1_int(uint16_t num)
{
    //The range of unsigned integer is from 0 to 65535,which max length is 5 digits. 
    char intArray[5];
    int i;
    for(i = 4; i >= 0 ; i--) {
        intArray[i] = num % 10 + 48;
        num = num / 10;
        if(num == 0)
            break;
    }
    USART1_puts(&intArray[i]);
}

uint16_t myatoi(char *str)
{	
	uint16_t num = 0;
	while (*str != '\0') {
		num *= 10; 
		num += (*str - '0');
		str++;
	}
	return num;
}

void command_detect(char *str)
{
	uint16_t brightness;
	uint16_t channel;
	brightness = myatoi(str);
	channel = brightness / 1000;
	brightness %= 1000;
	switch(channel){
		case 1:
			TIM4->CCR1 = brightness;
			USART1_puts("channel 1 :\t\0");
			USART1_int(brightness);	
			break;
		case 2:
			TIM4->CCR2 = brightness;
			USART1_puts("channel 2 :\t\0");
			USART1_int(brightness);
			break;
		case 3:
			TIM4->CCR3 = brightness;
			USART1_puts("channel 3 :\t\0");
			USART1_int(brightness);
			break;
		case 4:
			TIM4->CCR4 = brightness;
			USART1_puts("channel 4 :\t\0");
			USART1_int(brightness);
			break;
	}
}

void uart1command() {
	char buffer[50];
	int index = 0;
	char c;
	while (1) {
		// Receive character
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
		c = USART_ReceiveData(USART1);
		if (c == '\r' || c == '\n') {	
			buffer[index] = '\0';
			USART1_puts(buffer);
			command_detect(buffer);
			index = 0;
		} else{
			buffer[index++] = c;
		}
		if(index == 50)
			index = 0;
	}
}

void UART1Task(void *pvParameters)
{
	USART1_puts("Hello World!\r\n\0");
	USART1_puts("Just for STM32F429I Discovery verify USART1 with USB TTL Cable\r\n\0");
	uart1command();
}

//Main Function
int main(void)
{
	RCC_Configuration();
	TIM_Configuration();
	GPIO_Configuration();
	USART1_Configuration();

	prvInit();
			
	xTaskCreate(UART1Task, "UART1", 256,
		(void *)NULL, tskIDLE_PRIORITY + 1, (void *)NULL);
	
	vTaskStartScheduler();
}
