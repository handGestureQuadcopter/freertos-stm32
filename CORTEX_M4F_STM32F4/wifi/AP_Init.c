#include "usart1.h"
#include "AP_Init.h"
#include <string.h>



int AP_Init()
{	
	USART1_puts("AT+CIPMUX=1\r\n");
	
	USART1_puts("0AT+CIPSERVER=1,8888\r\n");
	
	return 1;
}

char receiveString_Parse(char* s)
{
	char* ptr;	
	ptr=strstr(s,':');
	return ptr++;
}
