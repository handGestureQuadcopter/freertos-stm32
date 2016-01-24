#include "motor.h"
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"

//PID define
#define dt (20/portTICK_RATE_MS)
#define KP 0.05f
#define KI 0
#define KD 1.5
#define SETPOINT_X 0
#define SETPOINT_Y 0
#define UPPER_BOUND(Correction) (Correction > 15 ? 15 : Correction)
#define LOWWER_BOUND(Correction) (Correction < -15 ? -15 : Correction)

void PIDTask();
uint8_t PID_Task_Creat(TaskHandle_t xHandle);
float bound_check(float value);
float calculateP_X(float roll);
float calculateP_Y(float pitch);
float calculateI(float integral, float error);
float calculateD(float error, float pre_error);
void PID_X(float error, float integral, float derivative);
void PID_Y(float error, float integral, float derivative);
