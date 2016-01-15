#include "motor.h"

//PID define
#define DT 100
#define KP 2
#define KI 1
#define KD 1
#define SETPOINT 1

uint16_t PIDTask(uint16_t measured_value)
void calculatePID(const SensorData *sensordata, MotorData *motordata);
