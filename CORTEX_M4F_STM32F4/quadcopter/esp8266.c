#include "esp8266.h"
#include "uart.h"
#include "shell.h"

char buffer6[MAX_UART_INPUT];
uint8_t buffer6_index = 0;
volatile uint8_t isOK;
char uart_out[32];

xTaskHandle xWifiHandle;

const char RESPONSE_OK[] = "OK";

//void ESP8266Task(void *pvParameters) {
//
//	while (1) {
//		while (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET) {}
//		char c = USART_ReceiveData(USART6);
//		if (c == '\n') {
//			buffer6[buffer6_index++] = '\n';
//			buffer6[buffer6_index] = '\0';
//			if (buffer6[0] == 'O' && buffer6[1] == 'K')
//				set_isOK(TRUE);
//			UART1_puts(buffer6);
//			//		UART1_puts("\r\n\0");
//			buffer6_index = 0;
//			continue;
//		} else {
//			buffer6[buffer6_index++] = c;
//		}
//		if (buffer6_index > MAX_UART_INPUT - 1) {
//			buffer6_index = 0;
//		}
//	}
//
//}

void esp8266_configure() {
//	Enable_TIM2_INTERRUPT();
//	UART6_puts("AT+CIPMUX=1\r\n");
//	delay(3000000);
//	UART6_puts("AT+CIPSERVER=1,9999\r\n");
//	delay(3000000);
}

void Enable_TIM3_INTERRUPT() {
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 168000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}

void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);

	}
}

//uint8_t buffer6_ready() {
//	return (buffer6_index > 0) ? TRUE : FALSE;
//}

uint8_t buffer6_read_respond(const char * rsp) {
	uint8_t rep = FALSE;
//	set_isTimeout(FALSE);
//	UART1_puts("\r\n");
//			UART1_puts("brr ");
//			shell_itoa(get_isTimeout(), uart_out);
//			UART1_puts(uart_out);
	TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
//	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	while (1) {
//		if (buffer6_ready())
//			UART1_puts("good\r\n");

		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
				TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
				break;
			}
//		UART6_ReadLine();

//		UART1_puts("not timeout\r\n");
//		UART1_puts("\r\n");
//				UART1_puts("whil ");
//				shell_itoa(isTimeout, uart_out);
//				UART1_puts(uart_out);
//		if (buffer6_ready()) {
//			UART1_puts("gg\r\n");
//			if (buffer6_search(rsp)) {
//				UART1_puts(rsp);
//				rep = TRUE;
//			}
//		}
	}
	UART1_puts("out\r\n");
	return rep;
}

void set_isOK(uint8_t set) {
	isOK = set;
}

uint8_t get_isOK() {
	return isOK;
}

uint8_t buffer6_search(const char * test) {
//	int i =0;
//	int bufferLen = strlen((const char *)buffer6);
//	if (bufferLen < MAX_UART_INPUT) {
//		if(strstr(buffer6, test))	{
//			UART1_puts(test);
//			return TRUE;
//		}
//	}
//	else {
//		// If the buffer is full, we need to search from the end of the
//		// buffer back to the beginning.
//		int testLen = strlen(test);
//		for (i=0; i< MAX_UART_INPUT; i++) {
//
//		}
//	}
	return FALSE;
}

void sendAT() {
	esp8266_clear_buffer6();
	UART6_puts("AT\r\n");
//	delay(3000000);
//	if (esp8266_read_response(RESPONSE_OK))
//		UART1_puts("\r\nGood");
//	else
//		UART1_puts("\r\nBad");
//	UART1_puts("\r\nGOOD");
//	UART1_puts("zz\r\n");
}

void putbuffer6() {
	UART1_puts("\r\n");
	UART1_puts(buffer6);
//	for (int i = 0; i < buffer6_index+1; i++) {
//		int d = (int) buffer6[i];
//		shell_itoa(d, uart_out);
//		UART1_puts(uart_out);
//		UART1_puts(" ");
//	}
}

//uint8_t ESP8266_Task_Creat() {
//	BaseType_t ret = xTaskCreate(ESP8266Task, "ESP8266", 512, (void * ) NULL,
//			tskIDLE_PRIORITY + 4, &xWifiHandle);
//	if (ret != pdPASS)
//		return 0;
//	return 1;
//}

void UART6_ReadLine() {
	while (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET);
	char c = USART_ReceiveData(USART6);
//	if (c == '\n' && buffer6[buffer6_index-1] == '\r') {
//		buffer6[buffer6_index++] = '\n';
//		buffer6[buffer6_index] = '\0';
//		UART1_puts(buffer6);
//		buffer6_index = 0;
//	} else if (c != '\n'){
		buffer6[buffer6_index++] = c;
//	}
	buffer6_index %= MAX_UART_INPUT;
}

uint8_t esp8266_read_response(const char * rsp) {
	int32_t timeout = 3000000;
	while (--timeout > 0) {
		if (esp8266_buffer6_available()) {
			if (esp8266_search_buffer6(rsp))
				return TRUE;
		}
	}
	return FALSE;
}

uint8_t esp8266_search_buffer6(const char * test) {
	// If our buffer isn't full, just do an strstr
	if (buffer6_index < MAX_UART_INPUT)
		return strstr((const char *)buffer6, test) ? TRUE : FALSE;
//	}
//	else
//	{	//! TODO
//		// If the buffer is full, we need to search from the end of the
//		// buffer back to the beginning.
//		int testLen = strlen(test);
//		for (i=0; i<ESP8266_RX_BUFFER_LEN; i++)
//		{
//
//		}
//	}
	return FALSE;
}

void esp8266_clear_buffer6() {
	memset(buffer6, '\0', MAX_UART_INPUT);
	buffer6_index = 0;
}

uint8_t esp8266_buffer6_available() {
	return (buffer6_index > 0) ? TRUE : FALSE;
}
