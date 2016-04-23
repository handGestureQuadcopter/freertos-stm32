#include "i2c.h"
#include "uart.h"
#include "mpu6050.h"

static uint32_t MPU6050_I2C_Timeout;

void I2C_MPU6050_Init(I2C_TypeDef* I2Cx, int clock_speed) {
	/*
	 *         SCL = PB6
	 *         SDA = PB7
	 */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable IOE_I2C and IOE_I2C_GPIO_PORT & Alternate Function clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOB, ENABLE);

	/* Connect PXx to I2C_SCL*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	/* Connect PXx to I2C_SDA*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

	/* SCL and SDA pins configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; //50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the I2C event priority */
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the I2C peripheral */
	I2C_DeInit(I2Cx);

	I2C_InitTypeDef I2C_InitStructure;

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = clock_speed;

	/* Initialize the I2C peripheral */
	I2C_Init(I2Cx, &I2C_InitStructure);

	I2C_ITConfig(I2Cx, I2C_IT_ERR, ENABLE);
	I2C_Cmd(I2Cx, ENABLE);
}

void I2C1_ER_IRQHandler(void)
{
  /* ACK failure */

  if (I2C_GetITStatus(I2C1, I2C_IT_AF))
  {
    UART1_puts("I2C_IT_AF");
    I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
  }
  else if(I2C_GetITStatus(I2C1, I2C_IT_BERR))
  {
    UART1_puts("I2C_IT_BERR");
  }
  else if(I2C_GetITStatus(I2C1, I2C_IT_ARLO))
  {
    UART1_puts("I2C_IT_ARLO");
  }
    else if(I2C_GetITStatus(I2C1, I2C_IT_OVR))
  {
    UART1_puts("I2C_IT_OVR");
  }
    else if(I2C_GetITStatus(I2C1, I2C_IT_PECERR))
  {
    UART1_puts("I2C_IT_PECERR");
  }
    else if(I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT))
  {
    UART1_puts("I2C_IT_TIMEOUT");
  }
  else if(I2C_GetITStatus(I2C1, I2C_IT_SMBALERT))
  {
    UART1_puts("I2C_IT_SMBALERT");
  }

}

int16_t I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint16_t ack) {
	/* Generate I2C start pulse */
	I2Cx->CR1 |= I2C_CR1_START;

	/* Wait till I2C is busy */
	MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
	while (!(I2Cx->SR1 & I2C_SR1_SB)) {
		if (--MPU6050_I2C_Timeout == 0x00) {
			UART1_puts("\r\nTime OUT Start1");
			return 1;
		}
	}

	/* Enable ack if we select it */
	if (ack) {
		I2Cx->CR1 |= I2C_CR1_ACK;
	}

	/* Send write/read bit */
	if (direction == I2C_Direction_Transmitter) {
		/* Send address with zero last bit */
		I2Cx->DR = address & ~I2C_OAR1_ADD0;

		/* Wait till finished */
		MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
		while (!(I2Cx->SR1 & I2C_SR1_ADDR)) {
			if (--MPU6050_I2C_Timeout == 0x00) {
				UART1_puts("\r\nTime OUT Start2");
				return 1;
			}
		}
	}
	if (direction == I2C_Direction_Receiver) {
		/* Send address with 1 last bit */
		I2Cx->DR = address | I2C_OAR1_ADD0;

		/* Wait till finished */
		MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			if (--MPU6050_I2C_Timeout == 0x00) {
				UART1_puts("\r\nTime OUT Start3");
				return 1;
			}
		}
	}

	/* Read status register to clear ADDR flag */
	I2Cx->SR2;

	/* Return 0, everything ok */
	return 0;
}

uint8_t I2C_Stop(I2C_TypeDef* I2Cx) {
	/* Wait till transmitter not empty */
	MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
	while (((!(I2Cx->SR1 & I2C_SR1_TXE)) || (!(I2Cx->SR1 & I2C_SR1_BTF)))) {
		if (--MPU6050_I2C_Timeout == 0x00) {
			UART1_puts("\r\nTime OUT Stop");
			return 1;
		}
	}

	/* Generate stop */
	I2Cx->CR1 |= I2C_CR1_STOP;

	/* Return 0, everything ok */
	return 0;
}

uint8_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg) {
	uint8_t received_data;
	I2C_Start(I2Cx, address, I2C_Direction_Transmitter, I2C_Ack_Disable);
	I2C_WriteData(I2Cx, reg);
	I2C_Stop(I2Cx);
	I2C_Start(I2Cx, address, I2C_Direction_Receiver, I2C_Ack_Disable);
	received_data = I2C_ReadNack(I2Cx);
	return received_data;
}

uint8_t I2C_ReadAck(I2C_TypeDef* I2Cx) {
	uint8_t data;

	/* Enable ACK */
	I2Cx->CR1 |= I2C_CR1_ACK;

	/* Wait till not received */
	MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if (--MPU6050_I2C_Timeout == 0x00) {
			UART1_puts("\r\nTime OUT ReadAck");
			return 1;
		}
	}

	/* Read data */
	data = I2Cx->DR;

	/* Return data */
	return data;
}

uint8_t I2C_ReadNack(I2C_TypeDef* I2Cx) {
	uint8_t data;

	/* Disable ACK */
	I2Cx->CR1 &= ~I2C_CR1_ACK;

	/* Generate stop */
	I2Cx->CR1 |= I2C_CR1_STOP;

	/* Wait till received */
	MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if (--MPU6050_I2C_Timeout == 0x00) {
			UART1_puts("\r\nTime OUT ReadNack");
			return 1;
		}
	}

	/* Read data */
	data = I2Cx->DR;

	/* Return data */
	return data;
}

void I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	uint8_t i;
	I2C_Start(I2Cx, address, I2C_Direction_Transmitter, I2C_Ack_Enable);
	I2C_WriteData(I2Cx, reg);
	I2C_Stop(I2Cx);
	I2C_Start(I2Cx, address, I2C_Direction_Receiver, I2C_Ack_Enable);
	for (i = 0; i < count; i++) {
		if (i == (count - 1)) {
			/* Last byte */
			data[i] = I2C_ReadNack(I2Cx);
		} else {
			data[i] = I2C_ReadAck(I2Cx);
		}
	}
}

void I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data) {
	I2C_Start(I2Cx, address, I2C_Direction_Transmitter, I2C_Ack_Disable);
	I2C_WriteData(I2Cx, reg);
	I2C_WriteData(I2Cx, data);
	I2C_Stop(I2Cx);
}

void I2C_WriteWord(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint16_t data) {
	I2C_Start(I2Cx, address, I2C_Direction_Transmitter, I2C_Ack_Disable);
	I2C_WriteData(I2Cx, reg);
	I2C_WriteData(I2Cx, (data >> 8));		// send MSB
	I2C_WriteData(I2Cx, (data << 8));		// send LSB
	I2C_Stop(I2Cx);
}

void I2C_WriteBytes(I2C_TypeDef* I2Cx, uint8_t address, uint8_t writeAddr, uint8_t length, uint8_t *data) {
	int i = 0;
	I2C_Start(I2Cx, address, I2C_Direction_Transmitter, I2C_Ack_Disable);
	I2C_WriteData(I2Cx, writeAddr);
	for (i = 0; i < length; i++) {
		I2C_WriteData(I2Cx, data[i]);
	}
	I2C_Stop(I2Cx);
}

void I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data) {
	/* Wait till I2C is not busy anymore */
	MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
	while (!(I2Cx->SR1 & I2C_SR1_TXE) && MPU6050_I2C_Timeout) {
		if (--MPU6050_I2C_Timeout == 0x00) {
			UART1_puts("\r\nTime OUT Writedata");
		}
	}

	/* Send I2C data */
	I2Cx->DR = data;
}

void I2C_WriteBit(uint8_t address, uint8_t shamt, uint8_t t_or_f) {
	uint8_t temp;
	temp = I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, address);
	temp = t_or_f ? (temp | (1 << shamt)) : (temp & ~(1 << shamt));
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, address, temp);
}

void I2C_WriteBits(uint8_t address, uint8_t bitStart, uint8_t length, uint8_t data) {
	uint8_t temp, mask;
	temp = I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, address);
	mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1);
	data &= mask;
	temp &= ~(mask);
	temp |= data;
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, address, temp);
}

void I2C_ReadBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
	uint8_t temp;
	temp = I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, regAddr);
	*data = temp & (1 << bitNum);
}

void I2C_ReadBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
	uint8_t mask, temp;
	temp = I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, regAddr);
	mask = ((1 << length) - 1) << (bitStart - length + 1);
	temp &= mask;
	temp >>= (bitStart - length + 1);
	*data = temp;
}
