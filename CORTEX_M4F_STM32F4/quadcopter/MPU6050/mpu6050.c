#include "mpu6050.h"
#include "i2c.h"
#include "shell.h"
#include "uart.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define SENSOR_PERIOD_MS 10

xTaskHandle xSensorHandle;
Angle_Data Angle;

uint8_t MPUbuffer[14];
uint8_t *MPUdmpPacketBuffer;
const uint16_t MPUdmpPacketSize = 42;
uint8_t MPUverifyBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
uint8_t MPUdevAddr;
uint8_t MPUbuffer[14];
uint16_t MPUfifoCount;     	// count of all bytes currently in FIFO
uint8_t  MPUfifoBuffer[64];	// FIFO storage buffer

void MPU6050Task(void *pvParameters) {
	char uart_out[32];

	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorFloat gravity;
	float roll, pitch;
	float attitude[3];         // [yaw, roll, pitch]

	MPUdmpInitialize();
	MPUsetDMPEnabled(TRUE);

	mpuIntStatus = MPUgetIntStatus();
	packetSize = MPUdmpGetFIFOPacketSize();

	while (1) {

		mpuIntStatus = MPUgetIntStatus();
		fifoCount = MPUgetFIFOCount();
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			MPUresetFIFO();
//			UART1_puts("\r\nFIFO overflow!\r\n");
		} else if (mpuIntStatus & 0x02) {
			while (fifoCount < packetSize)
				fifoCount = MPUgetFIFOCount();
			MPUgetFIFOBytes(fifoBuffer, packetSize);
			fifoCount -= packetSize;

			MPUdmpGetQuaternion(&q, fifoBuffer);
			MPUdmpGetGravityVect(&gravity, &q);
			MPUdmpGetYawPitchRoll(attitude, &q, &gravity);
			roll = attitude[1] * RAD_TO_DEG;
			pitch = -attitude[2] * RAD_TO_DEG;
		}

		taskENTER_CRITICAL();
		Angle.Roll = roll;
		Angle.Pitch = pitch;
		taskEXIT_CRITICAL();

		//UART1_puts("\r\n");
		//shell_float2str(roll, uart_out);
		//UART1_puts(uart_out);
		//UART1_puts(" ");
		//shell_float2str(pitch, uart_out);
		//UART1_puts(uart_out);

		vTaskDelay(SENSOR_PERIOD_MS / portTICK_PERIOD_MS);
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

	// PLL with X axis gyroscope reference
	I2C_WriteBits(MPU6050_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);

	// Disable Sleep
	I2C_WriteBit(MPU6050_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);

	/* Config accelerometer */
	temp = I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG, temp);
	
	/* Config gyroscope */
	temp = I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG, temp);

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

/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU6050_RA_I2C_SLV0_ADDR
 */
void MPUsetSlaveAddress(uint8_t num, uint8_t address) {
    if (num > 3) return;
    I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_I2C_SLV0_ADDR + num*3, address);
}

/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
 */
void MPUsetI2CMasterModeEnabled(uint8_t enabled) {
    I2C_WriteBit(MPU6050_PWR_MGMT_1, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
void MPUsetIntEnabled(uint8_t enabled) {
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_INT_ENABLE, enabled);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void MPUresetFIFO() {
	I2C_WriteBit(MPU6050_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t MPUgetFIFOCount() {
	I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_FIFO_COUNTH, MPUbuffer, 2);
    return (((uint16_t)MPUbuffer[0]) << 8) | MPUbuffer[1];
}

/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
void MPUsetMotionDetectionThreshold(uint8_t threshold) {
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_MOTION_THRESH, threshold);
}

/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
void MPUsetZeroMotionDetectionThreshold(uint8_t threshold) {
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_ZRMOT_THR, threshold);
}

/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
void MPUsetMotionDetectionDuration(uint8_t duration) {
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_MOT_DUR, duration);
}

/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
void MPUsetZeroMotionDetectionDuration(uint8_t duration) {
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_ZRMOT_DUR, duration);
}

/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void MPUsetFIFOEnabled(uint8_t enabled) {
    I2C_WriteBit(MPU6050_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
uint8_t MPUgetIntStatus() {
    return I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_INT_STATUS);
}

/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_RESET_BIT
 */
void MPUresetI2CMaster() {
    I2C_WriteBit(MPU6050_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, 1);
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return Byte from FIFO buffer
 */
uint8_t MPUgetFIFOByte() {
    return I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_FIFO_R_W);
}
void MPUgetFIFOBytes(uint8_t *data, uint8_t length) {
	I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_FIFO_R_W, data, length);
}

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register
uint8_t MPUgetOTPBankValid() {
    I2C_ReadBit(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, MPUbuffer);
    return MPUbuffer[0];
}

void MPUsetOTPBankValid(uint8_t enabled) {
    I2C_WriteBit(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}

int8_t MPUgetXGyroOffset() {
    I2C_ReadBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, MPUbuffer);
    return MPUbuffer[0];
}

void MPUsetXGyroOffset(int8_t offset) {
    I2C_WriteBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// YG_OFFS_TC register
int8_t MPUgetYGyroOffset() {
    I2C_ReadBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, MPUbuffer);
    return MPUbuffer[0];
}
void MPUsetYGyroOffset(int8_t offset) {
    I2C_WriteBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register
int8_t MPUgetZGyroOffset() {
    I2C_ReadBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, MPUbuffer);
    return MPUbuffer[0];
}
void MPUsetZGyroOffset(int8_t offset) {
    I2C_WriteBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// X_FINE_GAIN register
int8_t MPUgetXFineGain() {
    return I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_X_FINE_GAIN);
}
void MPUsetXFineGain(int8_t gain) {
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_X_FINE_GAIN, gain);
}

// Y_FINE_GAIN register
int8_t MPUgetYFineGain() {
	return I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_Y_FINE_GAIN);
}
void MPUsetYFineGain(int8_t gain) {
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_Y_FINE_GAIN, gain);
}

// Z_FINE_GAIN register
int8_t MPUgetZFineGain() {
	return I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_Z_FINE_GAIN);
}
void MPUsetZFineGain(int8_t gain) {
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_Z_FINE_GAIN, gain);
}

// XA_OFFS_* registers
int16_t MPUgetXAccelOffset() {
	I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_XA_OFFS_H, MPUbuffer, 2);
    return (((int16_t)MPUbuffer[0]) << 8) | MPUbuffer[1];
}
void MPUsetXAccelOffset(int16_t offset) {
	I2C_WriteWord(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_XA_OFFS_H, offset);
}

// YA_OFFS_* register

int16_t MPUgetYAccelOffset() {
	I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_YA_OFFS_H, MPUbuffer, 2);
    return (((int16_t)MPUbuffer[0]) << 8) | MPUbuffer[1];
}
void MPUsetYAccelOffset(int16_t offset) {
	I2C_WriteWord(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_YA_OFFS_H, offset);
}

// ZA_OFFS_* register

int16_t MPUgetZAccelOffset() {
	I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_ZA_OFFS_H, MPUbuffer, 2);
    return (((int16_t)MPUbuffer[0]) << 8) | MPUbuffer[1];
}
void MPUsetZAccelOffset(int16_t offset) {
	I2C_WriteWord(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_ZA_OFFS_H, offset);
}

// XG_OFFS_USR* registers

int16_t MPUgetXGyroOffsetUser() {
	I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_XG_OFFS_USRH, MPUbuffer, 2);
    return (((int16_t)MPUbuffer[0]) << 8) | MPUbuffer[1];
}
void MPUsetXGyroOffsetUser(int16_t offset) {
	I2C_WriteWord(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_XG_OFFS_USRH, offset);
}

// YG_OFFS_USR* register

int16_t MPUgetYGyroOffsetUser() {
	I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_YG_OFFS_USRH, MPUbuffer, 2);
    return (((int16_t)MPUbuffer[0]) << 8) | MPUbuffer[1];
}
void MPUsetYGyroOffsetUser(int16_t offset) {
	I2C_WriteWord(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_YG_OFFS_USRH, offset);
}

// ZG_OFFS_USR* register

int16_t MPUgetZGyroOffsetUser() {
	I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_ZG_OFFS_USRH, MPUbuffer, 2);
    return (((int16_t)MPUbuffer[0]) << 8) | MPUbuffer[1];
}
void MPUsetZGyroOffsetUser(int16_t offset) {
	I2C_WriteWord(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_ZG_OFFS_USRH, offset);
}

// INT_ENABLE register (DMP functions)

uint8_t MPUgetIntPLLReadyEnabled() {
    I2C_ReadBit(MPU6050_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, MPUbuffer);
    return MPUbuffer[0];
}
void MPUsetIntPLLReadyEnabled(uint8_t enabled) {
    I2C_WriteBit(MPU6050_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, enabled);
}
uint8_t MPUgetIntDMPEnabled() {
    I2C_ReadBit(MPU6050_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, MPUbuffer);
    return MPUbuffer[0];
}
void MPUsetIntDMPEnabled(uint8_t enabled) {
    I2C_WriteBit(MPU6050_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, enabled);
}

// DMP_INT_STATUS

uint8_t MPUgetDMPInt5Status() {
    I2C_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_5_BIT, MPUbuffer);
    return MPUbuffer[0];
}
uint8_t MPUgetDMPInt4Status() {
    I2C_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_4_BIT, MPUbuffer);
    return MPUbuffer[0];
}
uint8_t MPUgetDMPInt3Status() {
    I2C_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_3_BIT, MPUbuffer);
    return MPUbuffer[0];
}
uint8_t MPUgetDMPInt2Status() {
    I2C_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_2_BIT, MPUbuffer);
    return MPUbuffer[0];
}
uint8_t MPUgetDMPInt1Status() {
    I2C_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_1_BIT, MPUbuffer);
    return MPUbuffer[0];
}
uint8_t MPUgetDMPInt0Status() {
    I2C_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_0_BIT, MPUbuffer);
    return MPUbuffer[0];
}

// INT_STATUS register (DMP functions)

uint8_t MPUgetIntPLLReadyStatus() {
    I2C_ReadBit(MPU6050_INT_STATUS, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, MPUbuffer);
    return MPUbuffer[0];
}
uint8_t MPUgetIntDMPStatus() {
	I2C_ReadBit(MPU6050_INT_STATUS, MPU6050_INTERRUPT_DMP_INT_BIT, MPUbuffer);
    return MPUbuffer[0];
}

// USER_CTRL register (DMP functions)

uint8_t MPUgetDMPEnabled() {
	I2C_ReadBit(MPU6050_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, MPUbuffer);
    return MPUbuffer[0];
}
void MPUsetDMPEnabled(uint8_t enabled) {
    I2C_WriteBit(MPU6050_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}
void MPUresetDMP() {
    I2C_WriteBit(MPU6050_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1);
}

// BANK_SEL register
void MPUsetMemoryBank(uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank) {
    bank &= 0x1F;
    if (userBank == 1) bank |= 0x20;
    if (prefetchEnabled == 1) bank |= 0x40;
    I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_BANK_SEL, bank);
}

// MEM_START_ADDR register
void MPUsetMemoryStartAddress(uint8_t address) {
    I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_MEM_START_ADDR, address);
}

// MEM_R_W register

uint8_t MPUreadMemoryByte() {
    return I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_MEM_R_W);
}
void MPUwriteMemoryByte(uint8_t data) {
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_MEM_R_W, data);
}
void MPUreadMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank,	uint8_t address) {
	uint8_t chunkSize;
	uint16_t i;
	MPUsetMemoryBank(bank, 0, 0);
	MPUsetMemoryStartAddress(address);

	for (i = 0; i < dataSize;) {
		// determine correct chunk size according to bank position and data size
		chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize)
			chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address)
			chunkSize = 256 - address;

		// read the chunk of data as specified
		I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_MEM_R_W, data + i, chunkSize);

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address += chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize) {
			if (address == 0)
				bank++;
			MPUsetMemoryBank(bank, 0, 0);
			MPUsetMemoryStartAddress(address);
		}
	}
}

uint8_t MPUwriteMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank,
		uint8_t address, uint8_t verify, uint8_t useProgMem) {
	/*
	 verifyBuffer and progBuffer malloc/free has been removed, ProgMem support removed
	 */
	uint8_t chunkSize;
	uint16_t i;
	MPUsetMemoryBank(bank, 0, 0);
	MPUsetMemoryStartAddress(address);
	for (i = 0; i < dataSize;) {
		// determine correct chunk size according to bank position and data size
		chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize)
			chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address)
			chunkSize = 256 - address;

		// write the chunk of data as specified
//		MPUprogBuffer = (uint8_t *) data + i;

		I2C_WriteBytes(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_MEM_R_W, chunkSize, (uint8_t *) data + i);

		// verify data if needed
		if (verify) {
			MPUsetMemoryBank(bank, 0, 0);
			MPUsetMemoryStartAddress(address);
			I2C_ReadMulti(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_MEM_R_W, MPUverifyBuffer, chunkSize);
			if (memcmp((uint8_t *) data + i, MPUverifyBuffer, chunkSize) != 0) {
				return 0; // uh oh.
			}
		}

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address += chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize) {
			if (address == 0)
				bank++;
			MPUsetMemoryBank(bank, 0, 0);
			MPUsetMemoryStartAddress(address);
		}
	}
	return 1;
}
uint8_t MPUwriteProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify) {
    return MPUwriteMemoryBlock(data, dataSize, bank, address, verify, 1);
}
uint8_t MPUwriteDMPConfigurationSet(const uint8_t *data, uint16_t dataSize,	uint8_t useProgMem) {
	uint8_t success, special;
	uint16_t i;

	// config set data is a long string of blocks with the following structure:
	// [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
	uint8_t bank, offset, length;
	for (i = 0; i < dataSize;) {
		bank = data[i++];
		offset = data[i++];
		length = data[i++];

		// write data or perform special action
		if (length > 0) {
			//           MPUprogBuffer = (uint8_t *)data + i;
			/* too few arguments in function call? added FALSE at the end */
			success = MPUwriteMemoryBlock((uint8_t *) data + i, length, bank,
					offset, 1, 1);
			i += length;
		} else {
			// special instruction
			// NOTE: this kind of behavior (what and when to do certain things)
			// is totally undocumented. This code is in here based on observed
			// behavior only, and exactly why (or even whether) it has to be here
			// is anybody's guess for now.
			special = data[i++];
			/*Serial.print("Special command code ");
			 Serial.print(special, HEX);
			 Serial.println(" found...");*/
			if (special == 0x01) {
				// enable DMP-related interrupts

				//setIntZeroMotionEnabled(TRUE);
				//setIntFIFOBufferOverflowEnabled(TRUE);
				//setIntDMPEnabled(TRUE);
				I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_INT_ENABLE, 0x32);

				success = 1;
			} else {
				// unknown special command
				success = 0;
			}
		}

		if (!success) {
			return 0; // uh oh
		}
	}
	return 1;
}
uint8_t MPUwriteProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize) {
    return MPUwriteDMPConfigurationSet(data, dataSize, 1);
}

// DMP_CFG_1 register

uint8_t MPUgetDMPConfig1() {
    return I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_DMP_CFG_1);
}
void MPUsetDMPConfig1(uint8_t config) {
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_DMP_CFG_1, config);
}

// DMP_CFG_2 register

uint8_t MPUgetDMPConfig2() {
    return I2C_Read(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_DMP_CFG_2);
}
void MPUsetDMPConfig2(uint8_t config) {
	I2C_Write(MPU6050_I2C, MPU6050_I2C_ADDR, MPU6050_RA_DMP_CFG_2, config);
}
