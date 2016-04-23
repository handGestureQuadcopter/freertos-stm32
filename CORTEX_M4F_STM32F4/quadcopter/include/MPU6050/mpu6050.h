#ifndef _MPU6050_H
#define _MPU6050_H

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

typedef struct {
	float Roll;
	float Pitch; // Calculated angle using a Kalman filter
}Angle_Data;

#define RAD_TO_DEG 57.2957795130823208768f

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

extern uint8_t *MPUdmpPacketBuffer;
extern const uint16_t MPUdmpPacketSize; // no variable-length arrays at runtime in C
extern uint8_t MPUdevAddr;
extern uint8_t MPUbuffer[14];
extern uint16_t MPUfifoCount;// count of all bytes currently in FIFO
extern uint8_t MPUfifoBuffer[64];// FIFO storage buffer
extern uint8_t MPUverifyBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];

#define wGyro 5

///* Default I2C used */
#define	MPU6050_I2C					I2C1

/* Default I2C clock */
#define MPU6050_I2C_CLOCK			100000

/* Default I2C address */
#define MPU6050_I2C_ADDR			0xD0

#define MUP6050_ADDRESS 			0x68

/* Who I am register value */
#define MPU6050_I_AM				0x68

/* MPU6050 registers */
#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
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
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0


#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0


#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

/* Gyro sensitivities in /s */
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

//static uint8_t MPUverifyBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
 
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
 * @}
 */

void MPU6050Task(void *pvParameters);

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
 * @}
 */

uint8_t MPU6050_I2C_IsDeviceConnected(uint8_t address);

uint8_t MPU6050_Task_Creat();

void Init_MPU6050();

void MPUsetSlaveAddress(uint8_t num, uint8_t address);

void MPUsetI2CMasterModeEnabled(uint8_t enabled);

void MPUsetIntEnabled(uint8_t enabled);

void MPUresetFIFO();

uint16_t MPUgetFIFOCount();

void MPUsetMotionDetectionThreshold(uint8_t threshold);

void MPUsetZeroMotionDetectionThreshold(uint8_t threshold);

void MPUsetMotionDetectionDuration(uint8_t duration);

void MPUsetZeroMotionDetectionDuration(uint8_t duration);

void MPUsetFIFOEnabled(uint8_t enabled);

uint8_t MPUgetIntStatus();

void MPUresetI2CMaster();

uint8_t MPUgetFIFOByte();

void MPUgetFIFOBytes(uint8_t *data, uint8_t length);

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

	// XG_OFFS_TC register
	uint8_t MPUgetOTPBankValid(void);
	void MPUsetOTPBankValid(uint8_t enabled);
	int8_t MPUgetXGyroOffset(void);
	void MPUsetXGyroOffset(int8_t offset);

	// YG_OFFS_TC register
	int8_t MPUgetYGyroOffset(void);
	void MPUsetYGyroOffset(int8_t offset);

	// ZG_OFFS_TC register
	int8_t MPUgetZGyroOffset(void);
	void MPUsetZGyroOffset(int8_t offset);

	// X_FINE_GAIN register
	int8_t MPUgetXFineGain(void);
	void MPUsetXFineGain(int8_t gain);

	// Y_FINE_GAIN register
	int8_t MPUgetYFineGain(void);
	void MPUsetYFineGain(int8_t gain);

	// Z_FINE_GAIN register
	int8_t MPUgetZFineGain(void);
	void MPUsetZFineGain(int8_t gain);

	// XA_OFFS_* registers
	int16_t MPUgetXAccelOffset(void);
	void MPUsetXAccelOffset(int16_t offset);

	// YA_OFFS_* register
	int16_t MPUgetYAccelOffset(void);
	void MPUsetYAccelOffset(int16_t offset);

	// ZA_OFFS_* register
	int16_t MPUgetZAccelOffset(void);
	void MPUsetZAccelOffset(int16_t offset);

	// XG_OFFS_USR* registers
	int16_t MPUgetXGyroOffsetUser(void);
	void MPUsetXGyroOffsetUser(int16_t offset);

	// YG_OFFS_USR* register
	int16_t MPUgetYGyroOffsetUser(void);
	void MPUsetYGyroOffsetUser(int16_t offset);

	// ZG_OFFS_USR* register
	int16_t MPUgetZGyroOffsetUser(void);
	void MPUsetZGyroOffsetUser(int16_t offset);

	// INT_ENABLE register (DMP functions)
	uint8_t MPUgetIntPLLReadyEnabled(void);
	void MPUsetIntPLLReadyEnabled(uint8_t enabled);
	uint8_t MPUgetIntDMPEnabled(void);
	void MPUsetIntDMPEnabled(uint8_t enabled);

	// DMP_INT_STATUS
	uint8_t MPUgetDMPInt5Status(void);
	uint8_t MPUgetDMPInt4Status(void);
	uint8_t MPUgetDMPInt3Status(void);
	uint8_t MPUgetDMPInt2Status(void);
	uint8_t MPUgetDMPInt1Status(void);
	uint8_t MPUgetDMPInt0Status(void);

	// INT_STATUS register (DMP functions)
	uint8_t MPUgetIntPLLReadyStatus(void);
	uint8_t MPUgetIntDMPStatus(void);

	// USER_CTRL register (DMP functions)
	uint8_t MPUgetDMPEnabled(void);
	void MPUsetDMPEnabled(uint8_t enabled);
	void MPUresetDMP(void);

	// BANK_SEL register
	void MPUsetMemoryBank(uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank);

	// MEM_START_ADDR register
	void MPUsetMemoryStartAddress(uint8_t address);

	// MEM_R_W register
	uint8_t MPUreadMemoryByte(void);
	void MPUwriteMemoryByte(uint8_t data);
	void MPUreadMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
	uint8_t MPUwriteMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify, uint8_t useProgMem);
	uint8_t MPUwriteProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify);

	uint8_t MPUwriteDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, uint8_t useProgMem);
	uint8_t MPUwriteProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

	// DMP_CFG_1 register
	uint8_t MPUgetDMPConfig1(void);
	void MPUsetDMPConfig1(uint8_t config);

	// DMP_CFG_2 register
	uint8_t MPUgetDMPConfig2(void);
	void MPUsetDMPConfig2(uint8_t config);
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
