#ifndef _MPU6050_I2C_H
#define _MPU6050_I2C_H

#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"

#define MPU6050_I2C_TIMEOUT 50000

void I2C_MPU6050_Init(I2C_TypeDef* I2Cx, int clock_speed);

void I2C1_ER_IRQHandler(void);

int16_t I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint16_t ack);

uint8_t I2C_Stop(I2C_TypeDef* I2Cx);

uint8_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg);

uint8_t I2C_ReadAck(I2C_TypeDef* I2Cx);

uint8_t I2C_ReadNack(I2C_TypeDef* I2Cx);

void I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count);

void I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data);

void I2C_WriteWord(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint16_t data);

void I2C_WriteBytes(I2C_TypeDef* I2Cx, uint8_t address, uint8_t writeAddr, uint8_t length, uint8_t *data);

void I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data);

void I2C_WriteBit(uint8_t address, uint8_t shamt, uint8_t t_or_f);

void I2C_WriteBits(uint8_t address, uint8_t bitStart, uint8_t length, uint8_t data);

void I2C_ReadBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data);

void I2C_ReadBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);

#endif
