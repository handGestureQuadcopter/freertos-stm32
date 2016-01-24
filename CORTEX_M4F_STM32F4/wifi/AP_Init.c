#include "usart1.h"
#include "AP_Init.h"

int OKflag=0;


void AP_Init()
{	
	USART6_puts("AP start\r\n");
	USART1_puts("AT+CIPMUX=1\r\n");
	while(OKflag==0){
	USART6_puts("no work!\r\n");
	}
	USART1_puts("AT+CIPSERVER=1,8888\r\n");
	
	return;
}


