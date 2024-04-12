/*
 * MPU6050.h
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "stm32f4xx.h"

// device address
#define MPU6050ADDR 		  (0x68<<1)
//Read and write bits for I2C
#define WriteMask   			0x00
#define ReadMask    			0x01
//Register addresses
#define MPU6050_CONFIG_REG 		0x1A
#define MPU6050_INT_PIN_CFG_REG 0x37
#define MPU6050_GYRO_CONFIG_REG 0x1B  //left as default +-250 dps
#define MPU6050_ACCEL_CONFIG	0x1C  //left as default +-2g
#define MPU6050_INT_ENABLE_REG  0x38
#define MPU6050_PWR_MGMT_1_REG  0x6B
#define WHO_AM_I_REG            0x75

// data start address
#define MPU6050_DATA_START_ADD  0x3B

// Conversion constants for default gyro and acc settings
#define MPU6050_ACC_RAW_TO_MPS2 0.00059875482f
#define MPU6050_GYR_RAW_TO_RPS  0.00013323124f

#define ACCEL_CALI_SLOPE_X 0.007172811f
#define CALI_INTERCEPT_X  0.538545852f
#define ACCEL_CALI_SLOPE_Y 0.005398384f
#define CALI_INTERCEPT_Y  -0.095500155f
#define ACCEL_CALI_SLOPE_Z 0.002774847f
#define CALI_INTERCEPT_Z   0.441896957f

#define LPF_ACC_ALPHA 0.50f
#define LPF_GYR_ALPHA 0.1f
// structure for MPU6050 data

typedef struct{

	I2C_HandleTypeDef *i2cHandle;

	// Accelerometer data (m/s^2) [x y z]
	float acc_mps2[3];

	// Gyroscope data (rad/s) [x y z ]
	float gyr_rps[3];

	// Temperature
	float temp_degC;

	// DMA rx buffer
	uint8_t rxData[14];
	// DMA flags
  volatile uint8_t rxFlag;
  volatile uint8_t dataReadyFlag;

	// Calibration offsets
	float gyroOffsets[3];
	float accelOffsets[3];
	uint8_t numCalibrationMeas;
} MPU6050;

uint8_t checkMPU6050Ready();
uint8_t i2cWriteRegMPU6050(uint16_t devAddr, uint16_t RegAddr, uint8_t* data);
uint8_t MPU6050init(MPU6050* imu, I2C_HandleTypeDef* i2cHandle);
uint8_t MPU6050readDataDMA(MPU6050* imu);
void MPU6050convertRawData(MPU6050* imu);
void MPU6050filterRawData(MPU6050* imu);
void MPU6050Calibration(MPU6050* imu);
void calibrateGyro(MPU6050* imu);
void calibrateAccel(MPU6050* imu);


#endif /* INC_MPU6050_H_ */
