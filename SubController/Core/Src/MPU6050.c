#include "MPU6050.h"
extern I2C_HandleTypeDef hi2c1;

uint8_t checkMPU6050Ready()
{
	if((HAL_I2C_IsDeviceReady(&hi2c1,MPU6050ADDR+0,1,100)==HAL_OK))
	{
		return 1;
	}
	return 0;
}
uint8_t i2cWriteRegMPU6050(uint16_t devAddr, uint16_t RegAddr, uint8_t* data){

	return(HAL_I2C_Mem_Write(&hi2c1,devAddr,RegAddr, 1, data ,1,HAL_MAX_DELAY)==HAL_OK);

}

uint8_t MPU6050init(MPU6050* imu, I2C_HandleTypeDef* i2cHandle)
{
	imu->i2cHandle=i2cHandle;

	for(int i=0;i<3;i++)
	{
		imu->acc_mps2[i]=0;
		imu->accelOffsets[i]=0;
	}


	for(int i=0;i<3;i++)
	{
		imu->gyr_rps[i]=0;
		imu->gyroOffsets[i]=0;
	}

	for(int i=0; i<14;i++)
		imu->rxData[i]=0;

	imu->rxFlag=0;
	imu->dataReadyFlag=0;
	imu->temp_degC=0;
	imu->numCalibrationMeas=0;



	uint8_t numErrors=0;
	//Disable FSYNC, enable digital LPF ( fs=1kHz, bandwidth: acc=94 Hz, gyr=98Hz )
	uint8_t data=0x02;
	if(i2cWriteRegMPU6050(MPU6050ADDR,MPU6050_CONFIG_REG,&data)==0)
		numErrors+=1;

	// Enable Interrupt, clear status bits on any read operation
	data=0x10;
	if(i2cWriteRegMPU6050(MPU6050ADDR,MPU6050_INT_PIN_CFG_REG,&data)==0)
		numErrors+=1;

	data=0x01;
	if(i2cWriteRegMPU6050(MPU6050ADDR,MPU6050_INT_ENABLE_REG,&data)==0)
		numErrors+=1;

	// Wake-up MPU6050
	data=0x00;
	if(i2cWriteRegMPU6050(MPU6050ADDR,MPU6050_PWR_MGMT_1_REG,&data)==0)
		numErrors+=1;

	return numErrors==0;

}

uint8_t MPU6050readDataDMA(MPU6050* imu)
{
	HAL_StatusTypeDef readStatus;
	readStatus=HAL_I2C_Mem_Read_DMA(imu->i2cHandle,MPU6050ADDR,MPU6050_DATA_START_ADD,1,imu->rxData,14);
	imu->dataReadyFlag=0;
	if(readStatus==HAL_OK)return 1;
	else if(readStatus==HAL_ERROR)return 2;
	else if (readStatus==HAL_BUSY)return 3;
	else return 0;
}

void MPU6050convertRawData(MPU6050* imu)
{
	int16_t acc[3]={( (imu->rxData[0]<<8)|imu->rxData[1] ),( (imu->rxData[2]<<8)|imu->rxData[3] ),( (imu->rxData[4]<<8)|imu->rxData[5] )};
	//int16_t temperature=(imu->rxData[6]<<8)|imu->rxData[7];
	int16_t gyr[3]={( (imu->rxData[8]<<8)|imu->rxData[9] ),( (imu->rxData[10]<<8)|imu->rxData[11] ),( (imu->rxData[12]<<8)|imu->rxData[13] )};

	// Before accel calibration
	imu->acc_mps2[0]=MPU6050_ACC_RAW_TO_MPS2*acc[0];
	imu->acc_mps2[1]=MPU6050_ACC_RAW_TO_MPS2*acc[1];
	imu->acc_mps2[2]=MPU6050_ACC_RAW_TO_MPS2*acc[2];

	// After accel calibration
	imu->accelOffsets[0]=ACCEL_CALI_SLOPE_X*imu->acc_mps2[0]+CALI_INTERCEPT_X; // offset calculated from linear regression
	imu->acc_mps2[0]=imu->acc_mps2[0]-imu->accelOffsets[0];

	imu->accelOffsets[1]=ACCEL_CALI_SLOPE_Y*imu->acc_mps2[1]+CALI_INTERCEPT_Y; // offset calculated from linear regression
	imu->acc_mps2[1]=imu->acc_mps2[1]-imu->accelOffsets[1];

	imu->accelOffsets[2]=ACCEL_CALI_SLOPE_Z*imu->acc_mps2[2]+CALI_INTERCEPT_Z; // offset calculated from linear regression
	imu->acc_mps2[2]=imu->acc_mps2[2]-imu->accelOffsets[2];


	imu->gyr_rps[0]=MPU6050_GYR_RAW_TO_RPS*gyr[0];
	imu->gyr_rps[1]=MPU6050_GYR_RAW_TO_RPS*gyr[1];
	imu->gyr_rps[2]=MPU6050_GYR_RAW_TO_RPS*gyr[2];

}

void MPU6050filterRawData(MPU6050* imu)
{

	static float accPrev[3] = {0.0f}; // Static array to store previous accelerometer values
	static float gyrPrev[3] = {0.0f}; // Static array to store previous gyroscope values

	imu->acc_mps2[0]=LPF_ACC_ALPHA*accPrev[0]+(1.0f-LPF_ACC_ALPHA)*imu->acc_mps2[0];
	imu->acc_mps2[1]=LPF_ACC_ALPHA*accPrev[1]+(1.0f-LPF_ACC_ALPHA)*imu->acc_mps2[1];
	imu->acc_mps2[2]=LPF_ACC_ALPHA*accPrev[2]+(1.0f-LPF_ACC_ALPHA)*imu->acc_mps2[2];

	imu->gyr_rps[0]=LPF_GYR_ALPHA*accPrev[0]+(1.0f-LPF_GYR_ALPHA)*imu->gyr_rps[0];
	imu->gyr_rps[1]=LPF_GYR_ALPHA*accPrev[1]+(1.0f-LPF_GYR_ALPHA)*imu->gyr_rps[1];
	imu->gyr_rps[2]=LPF_GYR_ALPHA*accPrev[2]+(1.0f-LPF_GYR_ALPHA)*imu->gyr_rps[2];

	accPrev[0] = imu->acc_mps2[0];
	accPrev[1] = imu->acc_mps2[1];
	accPrev[2] = imu->acc_mps2[2];

	gyrPrev[0] = imu->gyr_rps[0];
	gyrPrev[1] = imu->gyr_rps[1];
	gyrPrev[2] = imu->gyr_rps[2];

}

