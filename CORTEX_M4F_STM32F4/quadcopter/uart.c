#include "uart.h"
#include "motor.h"

char buffer[MAX_UART_INPUT];
uint8_t buffer_index = 0;

void UART1_RCC_Configuration()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
}

void UART1_GPIO_Configuration()
{
	GPIO_InitTypeDef GPIO_InitStructure;

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

void UART1_Configuration()
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

	/* enable uart interrupt while receiving char */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);
}

void Init_UART1()
{
	UART1_RCC_Configuration();
	UART1_GPIO_Configuration();
	UART1_Configuration();
	UART1_puts("UART1 Init\r\n\0");
}

void UART1_puts(char* s)
{
	while(*s != '\0') {
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, *s);
		s++;
	}
}

void UART1_int(uint16_t num)
{
	//The range of unsigned integer is from 0 to 65535,which max length is 5 digits.Add \0 for 1 digit. 
	char intArray[6];
	int i;
	for(i = 4; i >= 0 ; i--) {
		intArray[i] = num % 10 + 48;
		num = num / 10;
		if(num == 0)
			break;
	}
	intArray[5] = '\0';
	UART1_puts(&intArray[i]);
}

void USART1_IRQHandler() {
	/*
	 * read a line from uart1
	 */
	UART1_ReadLine();
}

void UART1_ReadLine() {
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	char c = USART_ReceiveData(USART1);
	if (c == '\r' || c == '\n') {
		buffer[buffer_index] = '\0';
		remote_ctrl(buffer);
		buffer_index = 0;
	} else {
		buffer[buffer_index++] = c;
	}
	if (buffer_index == 50)
		buffer_index = 0;
}
