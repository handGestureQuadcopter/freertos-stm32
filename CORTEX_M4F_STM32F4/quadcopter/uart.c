#include "uart.h"
#include "shell.h"
#include "pid.h"

char buffer[MAX_UART_INPUT];
uint8_t buffer_index = 0;
extern MotorSpeed_t motorspeed;
extern Angle_Data Angle;

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

	USART_InitStructure.USART_BaudRate = 57600;
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

void UART1_ReadLine() 
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	char c = USART_ReceiveData(USART1);
	if (c == '\r' || c == '\n') {
		buffer[buffer_index] = '\0';
		UART1_puts(buffer);
		UART1_puts("\r\n\0");
		remote_ctrl(buffer);
		buffer_index = 0;
	} else {
		buffer[buffer_index++] = c;
	}
	if (buffer_index == 50)
		buffer_index = 0;
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
void remote_ctrl(char *str)
{
	uint16_t command;
	uint16_t protocol;
	uint16_t channel;
	char temp[10];
	command = myatoi(str);
	protocol = command / 1000;
	command %= 1000;	
	switch(protocol){
		case 0:		//reset	
			break;
		case 1:
			command = PULSE(command);
			if(command == MIN_PULSE){
				Reset_MagicNumber();
			}
			motorspeed.motor1_speed = command;
			motorspeed.motor2_speed = command;
			motorspeed.motor3_speed = command;
			motorspeed.motor4_speed = command;
			Change_Speed();
			break;
		case 2:
			if(command == 0){
				UART1_puts("KP  ");
				shell_float2str(getKP(),temp);
				UART1_puts(temp);
				UART1_puts("\r\nKI  ");
				shell_float2str(getKI(),temp);
				UART1_puts(temp);	
				UART1_puts("\r\nKD  ");
				shell_float2str(getKD(),temp);
				UART1_puts(temp);
			}
			else{
				channel = command / 100;
				command %= 100;
				switch(channel){
					case 1:
						setKP((float)command/10);
						break;
					case 2:
						setKI((float)command/10);
						break;
					case 3:
						setKD((float)command/10);
						break;
				}
			}
			break;
		case 3:
			if(command == 0){
				UART1_puts("\r\nSetpointX : ");
				shell_float2str(getSetPointX(),temp);
				UART1_puts(temp);
				UART1_puts("\r\nSetpointY : ");
				shell_float2str(getSetPointY(),temp);
				UART1_puts(temp);
			}
			else{
				UART1_puts("\r\nSetpointX : ");
				shell_float2str(setSetPointX(Angle.Roll),temp);
				UART1_puts(temp);
				UART1_puts("\r\nSetpointY : ");
				shell_float2str(setSetPointY(Angle.Pitch),temp);
				UART1_puts(temp);
			}
			break;
		case 4:
			UART1_puts("\r\nPID 1 2 3 4 : \0");
			UART1_int(motorspeed.magicNumber1);UART1_puts(" ");
			UART1_int(motorspeed.magicNumber2);UART1_puts(" ");
			UART1_int(motorspeed.magicNumber3);UART1_puts(" ");
			UART1_int(motorspeed.magicNumber4);
			break;
		case 5:		//forward
			if(command == 100){
				motorspeed.d_speedup1 += SPEEDUP;
				motorspeed.d_speedup2 += SPEEDUP;
				//setSetPointY(-20.0);
				UART1_puts("Forward\r\n\0");
			}else{
				motorspeed.d_speedup1 -= SPEEDUP;
				motorspeed.d_speedup2 -= SPEEDUP;
				//setSetPointY(-20.0);
				UART1_puts("Hover\r\n\0");
			}
			Change_Speed();
			break;
		case 6:		//left
			if(command == 100){
				motorspeed.d_speedup2 += SPEEDUP;
				motorspeed.d_speedup3 += SPEEDUP;
				//setSetPointX(-20.0);
				UART1_puts("Left\r\n\0");
			}else{
				motorspeed.d_speedup2 -= SPEEDUP;
				motorspeed.d_speedup3 -= SPEEDUP;
				//setSetPointX(-20.0);
				UART1_puts("Hover\r\n\0");
			}
			Change_Speed();
			break;
		case 7:		//backward
			if(command == 100){
				motorspeed.d_speedup3 += SPEEDUP;
				motorspeed.d_speedup4 += SPEEDUP;
				//setSetPointY(20.0);
				UART1_puts("Backward\r\n\0");
			}else{
				motorspeed.d_speedup3 -= SPEEDUP;
				motorspeed.d_speedup4 -= SPEEDUP;
				//setSetPointX(-20.0);
				UART1_puts("Hover\r\n\0");
			}
			Change_Speed();	
			break;
		case 8:		//right
			if(command == 100){
				motorspeed.d_speedup4 += SPEEDUP;
				motorspeed.d_speedup1 += SPEEDUP;
				//setSetPointX(20.0);
				UART1_puts("Right\r\n\0");
			}else{
				motorspeed.d_speedup4 -= SPEEDUP;
				motorspeed.d_speedup1 -= SPEEDUP;
				//setSetPointX(-20.0);
				UART1_puts("Hover\r\n\0");
			}
			Change_Speed();
			break;
	}
}
