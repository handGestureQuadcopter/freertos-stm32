#include "pid.h"
#include "motor.h"

uint16_t PIDTask(uint16_t measured_value)
{	
	uint16_t previous_error = 0;
	uint16_t intergral = 0;
	uint16_t error,derivative;
	uint16_t output;
	while(1){
		error = setpoint-sensordata;
		intergral = intergral + error*DT;
		derivative = (error - previous_error)/DT;
		output = KP*error + KI*intergral + KD*derivative;
		previous_error = error;
		vTaskDelay(DT);
	}
}

int16_t calculateP(const SensorData *sensordata, MotorData *motordata){
	
	int16_t errorX;
	int16_t errorY;	

	errorX = sensordata->pitch;
	errorY = sensordata->roll;
	
	//P control (X axis)
	motordata->speed1 += Kp * errorX;
	motordata->speed2 += Kp * errorX;
	motordata->speed3 -= Kp * errorX;
	motordata->speed4 -= Kp * errorX;

	//P control (Y axis)
	motordata->speed1  -= Kp * errorY;
	motordata->speed2  += Kp * errorY;
	motordata->speed3  += Kp * errorY;
	motordata->speed4  -= Kp * errorY;		
}

int16_t calculateI(int16_t integral, int16_t error)
{
		
}
