#include "motor.h"
#include "uart.h"
#include "stm32f4xx_conf.h" 

MotorSpeed_t motorspeed;

static uint16_t myatoi(char *str)
{	
	uint16_t num = 0;
	while (*str != '\0') {
		num *= 10; 
		num += (*str - '0');
		str++;
	}
	return num;
}

void PWM_RCC_Configuration()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE );
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE );
}

void PWM_TIM_Configuration()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	// Let PWM frequency equal 50Hz.
	TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInitStruct.TIM_Period = PERIOD - 1;   
	TIM_TimeBaseInitStruct.TIM_Prescaler = PRESCALER - 1;  
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;        
	TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );

	TIM_OCStructInit( &TIM_OCInitStruct );
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;

	// Initial duty cycle equals 0%. Value can range from zero to 65535.
	//TIM_Pulse = TIM4_CCR1 register (16 bits)
	TIM_OCInitStruct.TIM_Pulse = MIN_PULSE; 
	TIM_OC1Init(TIM4, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OCInitStruct.TIM_Pulse = MIN_PULSE; 
	TIM_OC2Init(TIM4, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OCInitStruct.TIM_Pulse = MIN_PULSE; 
	TIM_OC3Init(TIM4, &TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OCInitStruct.TIM_Pulse = MIN_PULSE; 
	TIM_OC4Init(TIM4, &TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 

	TIM_Cmd( TIM4, ENABLE );
}

void PWM_GPIO_Configuration()
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
}

void Init_Motor()
{
	PWM_RCC_Configuration();
	PWM_TIM_Configuration();
	PWM_GPIO_Configuration();

	motorspeed.motor1_speed = MIN_PULSE;
	motorspeed.motor2_speed = MIN_PULSE;
	motorspeed.motor3_speed = MIN_PULSE;
	motorspeed.motor4_speed = MIN_PULSE;
	motorspeed.magicNumber1 = 0;
	motorspeed.magicNumber2 = 0;
	motorspeed.magicNumber3 = 0;
	motorspeed.magicNumber4 = 0;

	UART1_puts("Motor Init\r\n\0");
}

void Change_Speed()
{
	taskENTER_CRITICAL();	
	TIM4->CCR1 = MIN_MAX(motorspeed.motor1_speed + motorspeed.magicNumber1);
	TIM4->CCR2 = MIN_MAX(motorspeed.motor2_speed + motorspeed.magicNumber2);
	TIM4->CCR3 = MIN_MAX(motorspeed.motor3_speed + motorspeed.magicNumber3);
	TIM4->CCR4 = MIN_MAX(motorspeed.motor4_speed + motorspeed.magicNumber4);
	taskEXIT_CRITICAL();
	UART1_puts("\r\nPID 1 2 3 4 : \0");
	UART1_int(motorspeed.magicNumber1);UART1_puts(" ");
	UART1_int(motorspeed.magicNumber2);UART1_puts(" ");
	UART1_int(motorspeed.magicNumber3);UART1_puts(" ");
	UART1_int(motorspeed.magicNumber4);
	UART1_puts("\r\nchannel 1 2 3 4 : \0");
	UART1_int(TIM4->CCR1);UART1_puts(" ");
	UART1_int(TIM4->CCR2);UART1_puts(" ");
	UART1_int(TIM4->CCR3);UART1_puts(" ");
	UART1_int(TIM4->CCR4);
}

void remote_ctrl(char *str)
{
	uint16_t speed;
	uint16_t channel;
	speed = myatoi(str);
	if(speed > 100 || speed < 0)
		return;
	UART1_int(speed);
	channel = speed / 1000;
	speed %= 1000;	
	switch(channel){
		case 1:
			speed = PULSE(speed);
			motorspeed.motor1_speed = speed;
			break;
		case 2:
			speed = PULSE(speed);
			motorspeed.motor2_speed = speed;
			break;
		case 3:
			speed = PULSE(speed);
			motorspeed.motor3_speed = speed;
			break;
		case 4:
			speed = PULSE(speed);
			motorspeed.motor4_speed = speed;
			break;
		case 5:
			speed = PULSE(speed);
			motorspeed.magicNumber1 = speed;
			UART1_puts("channel 1 add \0");
			UART1_int(speed);
			UART1_puts("\r\n\0");
			break;
		case 6:
			motorspeed.magicNumber2 = speed;
			UART1_puts("channel 2 add \0");
			UART1_int(speed);
			UART1_puts("\r\n\0");
			break;
		case 7:
			motorspeed.magicNumber3 = speed;
			UART1_puts("channel 3 add \0");
			UART1_int(speed);
			UART1_puts("\r\n\0");
			break;
		case 8:
			motorspeed.magicNumber4 = speed;
			UART1_puts("channel 4 add \0");
			UART1_int(speed);
			UART1_puts("\r\n\0");
			break;
		default:
			if(speed == 0)
				Reset_MagicNumber();
			speed = PULSE(speed);
			motorspeed.motor1_speed = speed;
			motorspeed.motor2_speed = speed;
			motorspeed.motor3_speed = speed;
			motorspeed.motor4_speed = speed;
	}
	Change_Speed();
}
void Reset_MagicNumber()
{	
	motorspeed.magicNumber1 = 0;
	motorspeed.magicNumber2 = 0;
	motorspeed.magicNumber3 = 0;
	motorspeed.magicNumber4 = 0;
}
