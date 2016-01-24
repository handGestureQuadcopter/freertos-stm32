#include "usart6.h"
#include "usart1.h"
#include "stm32f4xx_conf.h"

char buffer6[100];
char buffer7[100];
uint8_t buffer6_index = 0;
uint8_t buffer7_index = 0;
void USART6_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    /* --------------------------- System Clocks Configuration -----------------*/
    /* USART6 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    /* GPIOA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);   // USART6_TX
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);   // USART6_RX


    /* USARTx configuration ------------------------------------------------------*/
    /* USARTx configured as follow:
     *  - BaudRate = 115200 baud
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
    USART_Init(USART6, &USART_InitStructure);
    USART_Cmd(USART6, ENABLE);

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART6_IRQn);
}

void USART6_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
        USART_SendData(USART6, *s);
        s++;
    }
}

void USART6_IRQHandler() {
	/*
	 * read a line from uart1
	 */
	UART6_ReadLine();
}

void UART6_ReadLine() {
	while (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET);
	char c = USART_ReceiveData(USART6);	
	if (c == '\n') {
		buffer6[buffer6_index++] = '\r';
		buffer6[buffer6_index++] = '\n';
		buffer6[buffer6_index] = '\0';
		USART1_puts(buffer6);
		buffer6_index = 0;
	} else {
		buffer6[buffer6_index++] = c;
	}
	if (buffer6_index == 100)
		buffer6_index = 0;
}
