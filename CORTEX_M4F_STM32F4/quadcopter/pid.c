#include "pid.h"
#include "uart.h"

extern MotorSpeed_t motorspeed;
extern Kalman_Angel_Data K_Data;

float roll, pitch;

uint8_t PID_Task_Creat(TaskHandle_t xHandle) {
	BaseType_t ret = xTaskCreate(PIDTask,"PID",512,(void *) NULL,tskIDLE_PRIORITY + 4, &xHandle);
	if (ret != pdPASS)  
		return 0;
	return 1;   
}

void PIDTask()
{	
	int16_t pre_error_X, pre_error_Y;
	int16_t integral_X, integral_Y;
	int16_t error_X, error_Y;
	int16_t derivative_X, derivative_Y;

	//Intialize	
	pre_error_X = 0;
	integral_X = 0;
	
	pre_error_Y = 0;
	integral_Y = 0;

	//wait for initializing	ESC
	UART1_puts("WAIT FOR ESC\r\n\0");
	vTaskDelay(5000);
	UART1_puts("Start PID\r\n\0");
	
	while(1){
		taskENTER_CRITICAL();
		roll = K_Data.kalAngleX;
		pitch = K_Data.kalAngleY;
		taskEXIT_CRITICAL();

		//x axis for roll	
		error_X = calculateP_X(roll);
		integral_X = calculateI(integral_X, error_X);
		derivative_X = calculateD(error_X, pre_error_X);
		PID_X(error_X,integral_X,derivative_X);
		pre_error_X = error_X;

		//y axis for pitch
		error_Y = calculateP_Y(pitch);
		integral_Y = calculateI(integral_Y, error_Y);
		derivative_Y = calculateD(error_Y, pre_error_Y);
		PID_Y(error_Y,integral_Y,derivative_Y);
		pre_error_Y = error_Y;
	
		Change_Speed();
		vTaskDelay(dt);
	}
}

float calculateP_X(float roll)
{
	float error;
	error = SETPOINT_X - roll;
	return error;
}

float calculateP_Y(float pitch)
{
	float error;
	error = SETPOINT_Y - pitch;
	return error;
}


float calculateI(float integral, float error)
{		
	return (0.66) * (integral + (error * dt));
}

float calculateD(float error, float pre_error)	
{
	return ((error - pre_error) / dt);
}

void PID_X(float error, float integral, float derivative)
{
	float output;
	output = (KP * error) + (KI * integral) + (KD * derivative);
	output = PID_BOUND(output);
	motorspeed.magicNumber1 = MAGIC_CHECK(motorspeed.magicNumber1 + output);
	motorspeed.magicNumber2 = MAGIC_CHECK(motorspeed.magicNumber2 - output);
	motorspeed.magicNumber3 = MAGIC_CHECK(motorspeed.magicNumber3 - output);
	motorspeed.magicNumber4 = MAGIC_CHECK(motorspeed.magicNumber4 + output);
}

void PID_Y(float error, float integral, float derivative)
{
	float output;
	output = (KP * error) + (KI * integral) + (KD * derivative);
	output = PID_BOUND(output);
	motorspeed.magicNumber1 = MAGIC_CHECK(motorspeed.magicNumber1 - output);
	motorspeed.magicNumber2 = MAGIC_CHECK(motorspeed.magicNumber2 - output);
	motorspeed.magicNumber3 = MAGIC_CHECK(motorspeed.magicNumber3 + output);
	motorspeed.magicNumber4 = MAGIC_CHECK(motorspeed.magicNumber4 + output);

}
