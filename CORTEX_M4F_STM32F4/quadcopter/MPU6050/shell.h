#ifndef _MPU6050_SHELL_H
#define _MPU6050_SHELL_H

#include <stdint.h>
#include <math.h>

uint16_t s_strlen(const char *str);
void reverse(char *str);
void shell_itoa(int16_t n, char *str);
void shell_float2str(float f, char *str);

uint16_t shell_atoi(char *str);
float sqrt1(const float x);

#endif
