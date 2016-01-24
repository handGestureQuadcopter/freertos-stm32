#include <stdlib.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#define MAX_PULSE 400
#define MIN_PULSE 240
#define DEGREE 100
#define PULSE_RANGE (MAX_PULSE - MIN_PULSE) 
#define PULSE(Speed) (((Speed) * PULSE_RANGE / DEGREE)  + MIN_PULSE)
#define MIN_MAX(Speed) ((Speed < MIN_PULSE ? MIN_PULSE : Speed) > MAX_PULSE ? MIN_PULSE : Speed)
#define PERIOD 4800
#define PRESCALER 100
#define MAGIC_CHECK(Value) ((Value < -10 ? -10 : Value) > 10 ? 10 : Value)

typedef struct Motor_Speed{
	uint16_t motor1_speed;
	uint16_t motor2_speed;
	uint16_t motor3_speed;
	uint16_t motor4_speed;
	int16_t magicNumber1;
	int16_t magicNumber2;
	int16_t magicNumber3;
	int16_t magicNumber4;
} MotorSpeed_t;

void PWM_TIM_Configuration();
void PWM_GPIO_Configuration();
void PWM_RCC_Configuration();
void Init_Motor();
void Change_Speed();
void remote_ctrl(char *str);
