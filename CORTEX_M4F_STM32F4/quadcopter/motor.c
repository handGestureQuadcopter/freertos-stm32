#include "motor.h"
#include "uart.h"
#include "stm32f4xx_conf.h" 

MotorSpeed_t motorspeed;

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
	TIM4->CCR1 = MAX(MIN(motorspeed.motor1_speed + motorspeed.magicNumber1));
	TIM4->CCR2 = MAX(MIN(motorspeed.motor2_speed + motorspeed.magicNumber2));
	TIM4->CCR3 = MAX(MIN(motorspeed.motor3_speed + motorspeed.magicNumber3));
	TIM4->CCR4 = MAX(MIN(motorspeed.motor4_speed + motorspeed.magicNumber4));
	taskEXIT_CRITICAL();
/*	UART1_puts("\r\nPID 1 2 3 4 : \0");
	UART1_int(motorspeed.magicNumber1);UART1_puts(" ");
	UART1_int(motorspeed.magicNumber2);UART1_puts(" ");
	UART1_int(motorspeed.magicNumber3);UART1_puts(" ");
	UART1_int(motorspeed.magicNumber4);
	UART1_puts("\r\nchannel 1 2 3 4 : \0");
	UART1_int(TIM4->CCR1);UART1_puts(" ");
	UART1_int(TIM4->CCR2);UART1_puts(" ");
	UART1_int(TIM4->CCR3);UART1_puts(" ");
	UART1_int(TIM4->CCR4);*/
}

void Reset_MagicNumber()
{	
	motorspeed.magicNumber1 = 0;
	motorspeed.magicNumber2 = 0;
	motorspeed.magicNumber3 = 0;
	motorspeed.magicNumber4 = 0;
}
