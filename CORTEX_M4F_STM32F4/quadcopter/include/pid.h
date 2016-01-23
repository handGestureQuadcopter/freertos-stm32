#include "motor.h"
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"

//PID define
#define dt (100/portTICK_RATE_MS)
#define KP 0.12f
#define KI 0
#define KD 50
#define SETPOINT_X 3
#define SETPOINT_Y 0
#define PID_BOUND(Correction) ((Correction > 10 ? 10 : Correction) < -10 ? -10 : Correction )

void PIDTask();
uint8_t PID_Task_Creat(TaskHandle_t xHandle);
float bound_check(float value);
float calculateP_X(float roll);
float calculateP_Y(float pitch);
float calculateI(float integral, float error);
float calculateD(float error, float pre_error);
void PID_X(float error, float integral, float derivative);
void PID_Y(float error, float integral, float derivative);
