#ifndef _MPU6050_H
#define _MPU6050_H 100

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

typedef struct {
	float kalAngleX;
	float kalAngleY; // Calculated angle using a Kalman filter
}Kalman_Angel_Data;

#define RAD_TO_DEG 57.295779513082320876798154814105f

#define wGyro 5

///* Default I2C used */
#define	MPU6050_I2C					I2C1

/* Default I2C clock */
#define MPU6050_I2C_CLOCK			400000

/* Default I2C address */
#define MPU6050_I2C_ADDR			0xD0

#define MUP6050_ADDRESS 			0x68

/* Who I am register value */
#define MPU6050_I_AM				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* Gyro sensitivities in °/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)

/**
 * @}
 */
 
/**
 * @defgroup TM_MPU6050_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  MPU6050 result enumeration	
 */
typedef enum {
	TM_MPU6050_Result_Ok = 0x00,          /*!< Everything OK */
	TM_MPU6050_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
	TM_MPU6050_Result_DeviceInvalid       /*!< Connected device with address is not MPU6050 */
} TM_MPU6050_Result_t;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum {
	TM_MPU6050_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	TM_MPU6050_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	TM_MPU6050_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	TM_MPU6050_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} TM_MPU6050_Accelerometer_t;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	TM_MPU6050_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	TM_MPU6050_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	TM_MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	TM_MPU6050_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} TM_MPU6050_Gyroscope_t;

/**
 * @brief  Main MPU6050 structure
 */
typedef struct {
	/* Private */
	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
	/* Public */
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
} TM_MPU6050_t;

/**
 * @}
 */

void MPU6050Task(void);

/**
 * @defgroup TM_MPU6050_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes MPU6050 and I2C peripheral
 * @param  *DataStruct: Pointer to empty @ref TM_MPU6050_t structure
 * @param   DeviceNumber: MPU6050 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be TM_MPU6050_Device_0,
 *          but if AD0 pin is high, then you should use TM_MPU6050_Device_1
 *          
 *          Parameter can be a value of @ref TM_MPU6050_Device_t enumeration
 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref TM_MPU6050_Accelerometer_t enumeration
 * @param  GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref TM_MPU6050_Gyroscope_t enumeration
 * @retval Status:
 *            - TM_MPU6050_Result_t: Everything OK
 *            - Other member: in other cases
 */
TM_MPU6050_Result_t MPU6050_Init(TM_MPU6050_Accelerometer_t AccelerometerSensitivity, TM_MPU6050_Gyroscope_t GyroscopeSensitivity);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
TM_MPU6050_Result_t MPU6050_ReadAccelerometer();

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
TM_MPU6050_Result_t MPU6050_ReadGyroscope();

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
TM_MPU6050_Result_t MPU6050_ReadAccGyo();

/**
 * @}
 */

uint8_t MPU6050_I2C_IsDeviceConnected(uint8_t address);

uint8_t MPU6050_Task_Creat();

void Init_MPU6050();
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
