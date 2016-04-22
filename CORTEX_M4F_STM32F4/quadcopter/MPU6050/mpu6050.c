#include "mpu6050.h"
#include "i2c.h"
#include "shell.h"
#include "uart.h"

#define SENSOR_PERIOD_MS 50

#define Square(x) ((x)*(x))
#define Lowpass(old, new, alpha) ((1.0f - alpha) * old + alpha * new)

xTaskHandle xSensorHandle;
Angle_Data Angle;
static TM_MPU6050_t MPU6050_Data;

float acc_lowpass_gain = 0.03f, gyro_lowpass_gain =0.03f, complementAlpha = 0.0001f;

void MPU6050Task(void) {
	char uart_out[32];

	/* IMU Data */
	float roll = 0.0f, pitch = 0.0f;
	float R_raw, R_true, inv_R_raw, inv_R_true;
	float accX, accY, accZ;
	float scale_accX, scale_accY, scale_accZ;
	float gyroX, gyroY, gyroZ;
	float scale_gyroX, scale_gyroY, scale_gyroZ;
	float delta_gyroX, delta_gyroY, delta_gyroZ;
	float filter_accX = 0.0f, filter_accY = 0.0f, filter_accZ = 1.0f;
	float filter_gyroX = 0.0f, filter_gyroY = 0.0f, filter_gyroZ = 0.0f;
	float predict_X = 0.0f, predict_Y = 0.0f, predict_Z = 1.0f;
	float pre_X = 0.0f, pre_Y = 0.0f, pre_Z = 0.0f;
	float N_Ax_g = 0.0f, N_Ay_g = 0.0f, N_Az_g = 0.0f;
//	float acc_lowpass_gain = 0.03f, gyro_lowpass_gain =0.03f, complementAlpha = 0.0001f;
	float const dt = SENSOR_PERIOD_MS / 1000.0f;

	/* Offset Data. Example of data for current board

		Raw_Axis |  min   | max  |  average(offset) | 1g_scale	|>

		    X	   -4066	4096	15					4081
		    Y	   -4086	4060    -13 				4073
		    Z	   -4256  	3940	-158				4098

		But actual raw_data for 1g in 8g_full_scale setting should be 4096
		So that the modify factor for acc_scale will be 4096/(measured1g_scale) (i.e. scale it to 4096)
	*/
	float offset_accX = 15.0f, offset_accY = -13.0f, offset_accZ = -158.0f;
	float scale_acc_X = 4096.0f / 4081.0f;
	float scale_acc_Y = 4096.0f / 4073.0f;
	float scale_acc_Z = 4096.0f / 4098.0f;

	vTaskDelay(4000);

	/* Gyro offset */
	float offset_gyroX = 0.0f, offset_gyroY = 0.0f, offset_gyroZ = 0.0f;
	for (int i = 0; i < 2000; i++) {

		MPU6050_ReadGyroscope();
		offset_gyroX += ((float) MPU6050_Data.Gyroscope_X / 2000.0f);
		offset_gyroY +=	((float) MPU6050_Data.Gyroscope_Y / 2000.0f);
		offset_gyroZ +=	((float) MPU6050_Data.Gyroscope_Z / 2000.0f);

		delay(2000);
	}

	Enable_TIM2_INTERRUPT();
	while (1) {

		/* Read all data from sensor */
		MPU6050_ReadAccGyo();

		accX = MPU6050_Data.Accelerometer_X;
		accY = MPU6050_Data.Accelerometer_Y;
		accZ = MPU6050_Data.Accelerometer_Z;
		gyroX = MPU6050_Data.Gyroscope_X;
		gyroY = MPU6050_Data.Gyroscope_Y;
		gyroZ = MPU6050_Data.Gyroscope_Z;

		// Convert to scale data
		scale_accX = (accX - offset_accX) * MPU6050_Data.Acce_Mult * scale_acc_X;
		scale_accY = (accY - offset_accY) * MPU6050_Data.Acce_Mult * scale_acc_Y;
		scale_accZ = (accZ - offset_accZ) * MPU6050_Data.Acce_Mult * scale_acc_Z;

		// Convert to degree
		scale_gyroX = -(gyroX - offset_gyroX) * MPU6050_Data.Gyro_Mult;
		scale_gyroY = (gyroY - offset_gyroY) * MPU6050_Data.Gyro_Mult;
		scale_gyroZ = -(gyroZ - offset_gyroZ) * MPU6050_Data.Gyro_Mult;

		filter_accX = Lowpass(filter_accX, scale_accX, acc_lowpass_gain);
		filter_accY = Lowpass(filter_accY, scale_accY, acc_lowpass_gain);
		filter_accZ = Lowpass(filter_accZ, scale_accZ, acc_lowpass_gain);

		filter_gyroX = Lowpass(filter_gyroX, scale_gyroX, gyro_lowpass_gain);
		filter_gyroY = Lowpass(filter_gyroY, scale_gyroY, gyro_lowpass_gain);
		filter_gyroZ = Lowpass(filter_gyroZ, scale_gyroZ, gyro_lowpass_gain);

		R_raw = sqrtf(Square(filter_accX) + Square(filter_accY) + Square(filter_accZ));
		inv_R_raw = 1.0f / R_raw;
		N_Ax_g = filter_accX * inv_R_raw;
		N_Ay_g = filter_accY * inv_R_raw;
		N_Az_g = filter_accZ * inv_R_raw;

//		delta_gyroX = gyroX * dt * DEG_TO_RAD;
//		delta_gyroY = gyroY * dt * DEG_TO_RAD;
//		delta_gyroZ = gyroZ * dt * DEG_TO_RAD;
		delta_gyroX = filter_gyroX * dt * DEG_TO_RAD;
		delta_gyroY = filter_gyroY * dt * DEG_TO_RAD;
		delta_gyroZ = filter_gyroZ * dt * DEG_TO_RAD;

		predict_X = predict_X + (predict_Y * delta_gyroZ);
		predict_Y = - (predict_X * delta_gyroZ) + predict_Y;

		predict_Y = predict_Y + (predict_Z * delta_gyroX);
		predict_Z = - (predict_Y * delta_gyroX) + predict_Z;

		predict_X = predict_X - (predict_Z * delta_gyroY);
		predict_Z = (predict_X * delta_gyroY) + predict_Z;

		pre_X = predict_X;
		pre_Y = predict_Y;
		pre_Z = predict_Z;

		predict_X = Lowpass(pre_X, N_Ax_g, complementAlpha);
		predict_Y = Lowpass(pre_Y, N_Ay_g, complementAlpha);
		predict_Z = Lowpass(pre_Z, N_Az_g, complementAlpha);

		R_true = sqrtf(Square(predict_X) + Square(predict_Y) + Square(predict_Z));
		inv_R_true = 1.0f / R_true;
		predict_X = predict_X * inv_R_true;
		predict_Y = predict_Y * inv_R_true;
		predict_Z = predict_Z * inv_R_true;

		pitch = atanf(predict_Y / predict_Z) * RAD_TO_DEG;
		roll = atanf(-predict_X / sqrtf(Square(predict_Y) + Square(predict_Z))) * RAD_TO_DEG;

		taskENTER_CRITICAL();
		Angle.Roll = roll;
		Angle.Pitch = pitch;
		taskEXIT_CRITICAL();

//		UART1_puts("\r\n");
//		shell_float2str(scale_gyroX, uart_out);
//		UART1_puts(uart_out);
//		UART1_puts(" ");
//		shell_float2str(scale_gyroY, uart_out);
//		UART1_puts(uart_out);
//		UART1_puts(" ");
//		shell_float2str(scale_gyroZ, uart_out);
//		UART1_puts(uart_out);

		UART1_puts("\r\n");
		shell_float2str(roll, uart_out);
		UART1_puts(uart_out);
		UART1_puts(" ");
		shell_float2str(pitch, uart_out);
		UART1_puts(uart_out);
		
		vTaskSuspend(xSensorHandle);
	}
}

TM_MPU6050_Result_t MPU6050_Init(TM_MPU6050_Accelerometer_t AccelerometerSensitivity, TM_MPU6050_Gyroscope_t GyroscopeSensitivity) {

	uint8_t temp;

	I2C_MPU6050_Init(MPU6050_I2C, MPU6050_I2C_CLOCK);

	// reset
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1, 0x80);
	delay(3000000);

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
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1, 0x02);

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
	while (MPU6050_Init(TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_1000s)
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
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
		vTaskResume(xSensorHandle);
	}
}

void set_gain(uint16_t channel, uint16_t command) {
	/* channel 1 for acc_gain
	 * channel 2 for gyro_gain
	 * channel 3 for complement alpha
	 *
	 * command 0 for / 3
	 * command else for * 3
	 */
	char usart_out[32];
	switch (channel) {
		case 1:
			if (command)
				acc_lowpass_gain *= 3;
			else
				acc_lowpass_gain /= 3;
			UART1_puts("/r/n acc_gain :  ");
			shell_float2str(acc_lowpass_gain, usart_out);
			UART1_puts(usart_out);
			break;
		case 2:
			if (command)
				gyro_lowpass_gain *= 3;
			else
				gyro_lowpass_gain /= 3;
			UART1_puts("/r/n gyro_gain :  ");
			shell_float2str(gyro_lowpass_gain, usart_out);
			UART1_puts(usart_out);
			break;
		case 3:
			if (command)
				complementAlpha *=3;
			else
				complementAlpha /= 3;
			UART1_puts("/r/n alpha :  ");
			shell_float2str(complementAlpha, usart_out);
			UART1_puts(usart_out);
			break;
	}
	delay(1000);
}
