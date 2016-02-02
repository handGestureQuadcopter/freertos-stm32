#include "motor.h"
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"

//PID define
#define dt (20/portTICK_RATE_MS)
#define UPPER_BOUND(Correction) (Correction > 20 ? 20 : Correction)
#define LOWWER_BOUND(Correction) (Correction < 0 ? 0 : Correction)

void PIDTask();
uint8_t PID_Task_Creat();
float bound_check(float value);
float calculateP_X(float roll);
float calculateP_Y(float pitch);
float calculateI(float integral, float error);
float calculateD(float error, float pre_error);
void PID_X(float error, float integral, float derivative);
void PID_Y(float error, float integral, float derivative);
float getKP();
float getKI();
float getKD();
void setKP(float setting);
void setKI(float setting);
void setKD(float setting);
float getSetPointX();
float getSetPointY();
void setSetPointX(float setting);
void setSetPointY(float setting);
