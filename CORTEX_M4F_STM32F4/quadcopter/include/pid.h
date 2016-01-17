#include "motor.h"
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"

//PID define
#define dt (100/portTICK_RATE_MS)
#define KP 0.5
#define KI 1
#define KD 0
#define SETPOINT_X 0.5
#define SETPOINT_Y 0.5

void PIDTask();
uint8_t PID_Task_Creat();
float calculateP_X();
float calculateP_Y();
float calculateI(float integral, float error);
float calculateD(float error, float pre_error);
void PID_X(float error, float integral, float derivative);
void PID_Y(float error, float integral, float derivative);
