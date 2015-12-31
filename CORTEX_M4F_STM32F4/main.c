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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

void prvInit()
{
	//LCD init
	LCD_Init();
	IOE_Config();
	LTDC_Cmd( ENABLE );

	LCD_LayerInit();
	LCD_SetLayer( LCD_FOREGROUND_LAYER );
	LCD_Clear( LCD_COLOR_BLACK );
	LCD_SetTextColor( LCD_COLOR_WHITE );

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
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15; //PD12->LED3 PD13->LED4 PD14->LED5 PD15->LED6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init( GPIOD, &GPIO_InitStructure );

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

    // Let PWM frequency equal 100Hz.
    // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
    // Solving for prescaler gives 240.
    TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInitStruct.TIM_Period = 1680 - 1;   
    TIM_TimeBaseInitStruct.TIM_Prescaler = 500 - 1;  
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;        
    TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );
        
    TIM_OCStructInit( &TIM_OCInitStruct );
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
        
    // Initial duty cycle equals 0%. Value can range from zero to 65535.
    //TIM_Pulse = TIM4_CCR1 register (16 bits)
    TIM_OCInitStruct.TIM_Pulse = 65535; //(0=Always Off, 65535=Always On)
 
    TIM_OC1Init( TIM4, &TIM_OCInitStruct ); // Channel 1  LED
    TIM_OC2Init( TIM4, &TIM_OCInitStruct ); // Channel 2  LED
    TIM_OC3Init( TIM4, &TIM_OCInitStruct ); // Channel 3  LED
    TIM_OC4Init( TIM4, &TIM_OCInitStruct ); // Channel 4  LED
 
    TIM_Cmd( TIM4, ENABLE );
}

void USART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    /* USARTx configuration ------------------------------------------------------*/
    /* USARTx configured as follow:
     *  - BaudRate = 9600 baud
     *  - Word Length = 8 Bits
     *  - One Stop Bit
     *  - No parity
     *  - Hardware flow control disabled (RTS and CTS signals)
     *  - Receive and transmit enabled
     */
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
    while(*s) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s);
        s++;
    }
}

int atoi(const char *str)
{
	int i = 0, num = 0, sign = 0;
	if (str[i] == '-') {
		i++;
		sign = 1;
	}
	while (str[i] >= '0' && str[i] <= '9') {
		num = num * 10 + str[i] - '0';
		i++;
	}
	if (sign)
		num = -num;
	return num;
}

void command_detect(char *str)
{
	unsigned int number;
	number = atoi(str);
	TIM4->CCR1 = number;
}

void uart1command(char *buffer) {
	int index = 0;
	while (1) {
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
		buffer[index] = USART_ReceiveData(USART1);
		if (buffer[index] == 13) {
			USART1_puts("\0");
			buffer[index] = '\0';
			command_detect(buffer);
			break;
		} else if (buffer[index] == 8 || buffer[index] == 127) {
			if (index != 0) {
				USART1_puts("\b");
				USART1_puts(" ");
				USART1_puts("\b");
				index--;
			}
		} else {
			USART1_puts(&buffer[index++]);
		}
		if (index == 50)
			index--;
	}
}

void UART1Task(void *pvParameters)
{
	char buffer[50];
	USART1_puts("Hello World!\r\n");
	USART1_puts("Just for STM32F429I Discovery verify USART1 with USB TTL Cable\r\n");
	while (1) {
//		while (lock);
		uart1command(buffer);
		USART1_puts("\n");

	}

	while (1)
		; // Don't want to exit
}

void PWMTask(void *pvParameters)
{
	volatile int i = 0;
	//int whichpin = *(int *)pvParameters;
	uint16_t whichpin = 0;
	uint16_t brightness = 3000;
	int n = 1;
	while(1){
		
		//if(brightness + n <=0)
		//	whichpin = (whichpin + 1) % 4;
	
		if (((brightness + n) >= 3000) || ((brightness + n) <= 0))
			n = -n; // if  brightness maximum/maximum change direction

		//brightness += n;

		//Light LEDs in turn
		switch(whichpin){
			case 0:
				TIM4->CCR1 = brightness; // set brightness
				break;
			case 1:
				TIM4->CCR2 = brightness; // set brightness
				break;
			case 2:
				TIM4->CCR3 = brightness; // set brightness
				break;
			case 3:
				TIM4->CCR4 = brightness; // set brightness
				break;
		}
		for(i=0;i<10000;i++);  // delay
	}
}

//Main Function
int main(void)
{
	int whichpin[4] = {0,1,2,3};
	
	RCC_Configuration();
	TIM_Configuration();
	GPIO_Configuration();
	USART1_Configuration();

	prvInit();

	xTaskCreate(PWMTask, "PWM1 Task", 256, 
		( void * ) whichpin, tskIDLE_PRIORITY + 1,(void *)NULL);
	xTaskCreate(UART1Task, "UART1", 256,
		(void *) whichpin, tskIDLE_PRIORITY + 2, (void *)NULL);
	
	vTaskStartScheduler();
}

