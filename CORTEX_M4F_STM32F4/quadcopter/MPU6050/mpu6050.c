#include "mpu6050.h"
#include "kalman.h"
#include "i2c.h"
#include "shell.h"
#include "uart.h"

#define SENSOR_PERIOD_MS 50

#define Square(x) ((x)*(x))
#define Abs(x) ((x < 0) ? (0-x) : x)
#define Average(x, y) ((x + y) / 2)

xTaskHandle xSensorHandle;
static TM_MPU6050_t MPU6050_Data;
float const dt = SENSOR_PERIOD_MS / 1000.0;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
float accX, accY, accZ;
float gyroX, gyroY;
float kalAngleX, kalAngleY;
Kalman_Angel_Data K_Data;

void MPU6050Task(void) {
	char uart_out[32];

	initKalman(&kalmanX);
	initKalman(&kalmanY);
	MPU6050_ReadAccelerometer();

	accX = MPU6050_Data.Accelerometer_X;
	accY = MPU6050_Data.Accelerometer_Y;
	accZ = MPU6050_Data.Accelerometer_Z;

	float roll = atan2(-accY, accZ) * RAD_TO_DEG;
	float pitch = atan(-accX / sqrt1(Square(accY) + Square(accZ))) * RAD_TO_DEG;

	setAngle(&kalmanX, roll); // Set starting angle
	setAngle(&kalmanY, pitch);

	Enable_TIM2_INTERRUPT();
	while (1) {

		/* Read all data from sensor */
		MPU6050_ReadAccGyo();

		accX = MPU6050_Data.Accelerometer_X;
		accY = MPU6050_Data.Accelerometer_Y;
		accZ = MPU6050_Data.Accelerometer_Z;
		gyroX = MPU6050_Data.Gyroscope_X;
		gyroY = MPU6050_Data.Gyroscope_Y;

		float roll = atan2(-accY, accZ) * RAD_TO_DEG;
		float pitch = atan(-accX / sqrt1(Square(accY) + Square(accZ))) * RAD_TO_DEG;

		float gyroXrate = gyroX * MPU6050_Data.Gyro_Mult; // Convert to deg/s
		float gyroYrate = gyroY * MPU6050_Data.Gyro_Mult; // Convert to deg/s

		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			setAngle(&kalmanX, roll);
			kalAngleX = roll;
		} else {
			kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
		}

		if (Abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleY = getAngle(&kalmanY, pitch, gyroYrate, dt);

		taskENTER_CRITICAL();
		K_Data.kalAngleX = kalAngleX;
		K_Data.kalAngleY = kalAngleY;
		taskEXIT_CRITICAL();

//		UART1_puts("\r\n");
//		shell_float2str(kalAngleX, uart_out);
//		UART1_puts(uart_out);
//		UART1_puts(" ");
//		shell_float2str(kalAngleY, uart_out);
//		UART1_puts(uart_out);
		
		vTaskSuspend(xSensorHandle);
	}
}

TM_MPU6050_Result_t MPU6050_Init(TM_MPU6050_Accelerometer_t AccelerometerSensitivity, TM_MPU6050_Gyroscope_t GyroscopeSensitivity) {

	uint8_t temp;

	I2C_MPU6050_Init(MPU6050_I2C, MPU6050_I2C_CLOCK);

	/* Check if device is connected */
	if (!MPU6050_I2C_IsDeviceConnected(MPU6050_I2C_ADDR)) {
		/* Return error */
		return TM_MPU6050_Result_DeviceNotConnected;
	}

	/* Check who I am */
	if (I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_WHO_AM_I) != MPU6050_I_AM) {
		/* Return error */
		return TM_MPU6050_Result_DeviceInvalid;
	}

	// Wakeup MPU6050, PLL with Z axis gyroscope reference
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1, 0x03);

	// Enable low-pass filter 
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_CONFIG, 0x01);

	// Gyroscope sample output rate = 1kH / (1+ 1) 
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_SMPLRT_DIV, 0x01);

	/* Config accelerometer */
	temp = I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG, temp);
	
	/* Config gyroscope */
	temp = I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG, temp);
	
	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case TM_MPU6050_Accelerometer_2G:
			MPU6050_Data.Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2;
			break;
		case TM_MPU6050_Accelerometer_4G:
			MPU6050_Data.Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4;
			break;
		case TM_MPU6050_Accelerometer_8G:
			MPU6050_Data.Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8;
			break;
		case TM_MPU6050_Accelerometer_16G:
			MPU6050_Data.Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16;
			break;
		default:
			break;
	}
	
	switch (GyroscopeSensitivity) {
		case TM_MPU6050_Gyroscope_250s:
			MPU6050_Data.Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250;
			break;
		case TM_MPU6050_Gyroscope_500s:
			MPU6050_Data.Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;
			break;
		case TM_MPU6050_Gyroscope_1000s:
			MPU6050_Data.Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000;
			break;
		case TM_MPU6050_Gyroscope_2000s:
			MPU6050_Data.Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000;
			break;
		default:
			break;
	}

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t MPU6050_ReadAccelerometer() {
	uint8_t data[6];
	
	/* Read accelerometer data */
	I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_ACCEL_XOUT_H, data, 6);
	
	/* Format */
	MPU6050_Data.Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	MPU6050_Data.Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	MPU6050_Data.Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
	
	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t MPU6050_ReadGyroscope() {
	uint8_t data[6];
	
	/* Read gyroscope data */
	I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_GYRO_XOUT_H, data, 6);
	
	/* Format */
	MPU6050_Data.Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	MPU6050_Data.Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	MPU6050_Data.Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t MPU6050_ReadAccGyo() {
	uint8_t data[14];
	
	/* Read full raw data, 14bytes */
	I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_ACCEL_XOUT_H, data, 14);
	
	/* Format accelerometer data */
	MPU6050_Data.Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	MPU6050_Data.Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	MPU6050_Data.Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Format gyroscope data */
	MPU6050_Data.Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	MPU6050_Data.Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	MPU6050_Data.Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

uint8_t MPU6050_I2C_IsDeviceConnected(uint8_t address) {
	uint8_t connected = 0;
	/* Try to start, function will return 0 in case device will send ACK */
	if (!I2C_Start(MPU6050_I2C, address, I2C_Direction_Transmitter, I2C_Ack_Enable)) {
		connected = 1;
	}

	/* STOP I2C */
	I2C_Stop(MPU6050_I2C);

	/* Return status */
	return connected;
}

uint8_t MPU6050_Task_Creat() {
	BaseType_t ret = xTaskCreate(MPU6050Task,
			"MPU6050",
			512,
			(void * ) NULL,
			tskIDLE_PRIORITY + 4,
			&xSensorHandle);
	if (ret != pdPASS)
		return 0;
	return 1;
}

void Init_MPU6050() {
	/* Initialize MPU6050 sensor 0, address = 0xD0, AD0 pin on sensor is low */
	while (MPU6050_Init(TM_MPU6050_Accelerometer_4G, TM_MPU6050_Gyroscope_500s)
			!= TM_MPU6050_Result_Ok) {
		/* Display message to user */
		UART1_puts("\r\nRemote is NOT READY! PLEASE Checkout.");
	}
	UART1_puts("\r\nRemote is ready to use!");
}

void Enable_TIM2_INTERRUPT() {
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = SENSOR_PERIOD_MS - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 168000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, /*TIM_IT_Update*/TIM_FLAG_Update);
		vTaskResume(xSensorHandle);
	}
}
