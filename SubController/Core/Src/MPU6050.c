#include "MPU6050.h"
#include "stm32f4xx.h"
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
		imu->acc_mps2[i]=0;

	for(int i=0;i<3;i++)
		imu->gyr_rps[i]=0;

	for(int i=0; i<14;i++)
		imu->rxData[i]=0;

	imu->rxFlag=0;
	imu->dataReadyFlag=0;
	imu->temp_degC=0;

	uint8_t numErrors=0;
	//Disable FSYNC, enable digital LPF ( fs=1kHz, bandwidth: acc=94 Hz, gyr=98Hz )
	uint8_t data=0x02;
	if(i2cWriteRegMPU6050(MPU6050ADDR+WriteMask,MPU6050_CONFIG_REG,&data)==0)
		numErrors+=1;

	// Enable Interrupt, clear status bits on any read operation
	data=0x10;
	if(i2cWriteRegMPU6050(MPU6050ADDR+WriteMask,MPU6050_INT_PIN_CFG_REG,&data)==0)
		numErrors+=1;

	data=0x01;
	if(i2cWriteRegMPU6050(MPU6050ADDR+WriteMask,MPU6050_INT_ENABLE_REG,&data)==0)
		numErrors+=1;

	// Wake-up MPU6050
	data=0x00;
	if(i2cWriteRegMPU6050(MPU6050ADDR+WriteMask,MPU6050_PWR_MGMT_1_REG,&data)==0)
		numErrors+=1;

	return numErrors==0;

}

uint8_t MPU6050readDataDMA(MPU6050* imu)
{
	HAL_StatusTypeDef readStatus;
	readStatus=HAL_I2C_Mem_Read_DMA(imu->i2cHandle,MPU6050ADDR|0x01,MPU6050_DATA_START_ADD,1,imu->rxData,14);
	imu->rxFlag=(readStatus==HAL_OK);
	imu->dataReadyFlag=0;
	return imu->rxFlag;
}

void MPU6050convertRawData(MPU6050* imu)
{
	int16_t acc[3]={( (imu->rxData[0]<<8)|imu->rxData[1] ),( (imu->rxData[2]<<8)|imu->rxData[3] ),( (imu->rxData[4]<<8)|imu->rxData[5] )};
	//int16_t temperature=(imu->rxData[6]<<8)|imu->rxData[7];
	int16_t gyr[3]={( (imu->rxData[8]<<8)|imu->rxData[9] ),( (imu->rxData[10]<<8)|imu->rxData[11] ),( (imu->rxData[12]<<8)|imu->rxData[13] )};

	imu->acc_mps2[0]=MPU6050_ACC_RAW_TO_MPS2*acc[0];
	imu->acc_mps2[1]=MPU6050_ACC_RAW_TO_MPS2*acc[1];
	imu->acc_mps2[2]=MPU6050_ACC_RAW_TO_MPS2*acc[2];

	imu->gyr_rps[0]=MPU6050_GYR_RAW_TO_RPS*gyr[0];
	imu->gyr_rps[1]=MPU6050_GYR_RAW_TO_RPS*gyr[1];
	imu->gyr_rps[2]=MPU6050_GYR_RAW_TO_RPS*gyr[2];

}


