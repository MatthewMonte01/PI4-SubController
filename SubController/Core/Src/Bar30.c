#include "Bar30.h"
#include "stm32f4xx.h"
extern I2C_HandleTypeDef hi2c2;
HAL_StatusTypeDef halStatus;
void Bar30init(Bar30* sensor, I2C_HandleTypeDef* i2cHandle)
{
	sensor->i2cHandle=i2cHandle;

	for(int i=0; i<7;i++)
	{
		sensor->calibrationResult[i]=0;
	}
	sensor->rxFlag=0;
	sensor->conversionCompleteFlag=0;
	sensor->dT=0;
	sensor->actualPressure=0;
	sensor->actualTemperature=0;
	sensor->rawPressure=0;
	sensor->rawTemperature=0;
}



uint8_t Bar30reset(Bar30* sensor)
{
	uint8_t cmdByte=RESET_CMD;
	uint8_t numBytes=1;
	halStatus=(HAL_I2C_Master_Transmit(sensor->i2cHandle, BAR30_I2C_ADD, &cmdByte, numBytes, HAL_MAX_DELAY));
	return halStatus==HAL_OK;
}



uint8_t Bar30getCalibration(Bar30* sensor)
{
	uint8_t errorCount=0;
	uint8_t numTxBytes=1;
	uint8_t numRxBytes=2;
	uint8_t rxBuffer[2];
	uint8_t cmdByte;

	for (int i=0;i<7;i++)
	{
		cmdByte=READ_PROM_CMD_1+2*i;
		halStatus=HAL_I2C_Master_Transmit(sensor->i2cHandle, BAR30_I2C_ADD, &cmdByte, numTxBytes, HAL_MAX_DELAY);
		if(halStatus!=HAL_OK)
		{
			errorCount+=1;
		}
		halStatus==HAL_I2C_Master_Receive(sensor->i2cHandle, BAR30_I2C_ADD, &rxBuffer[0], numRxBytes, HAL_MAX_DELAY);
		if(halStatus!=HAL_OK)
		{
			errorCount+=1;
		}
		sensor->calibrationResult[i]=(uint16_t)((rxBuffer[0] << 8) | rxBuffer[1]);
	}
	return errorCount==0;

}



uint8_t Bar30getData(Bar30* sensor)
{
	uint8_t conversionGood=1;
	halStatus=initiatePressureConversion(sensor);
	if(halStatus!=HAL_OK)
		conversionGood=0;
	HAL_Delay(3);
	halStatus=readPressureData(sensor);
	if(halStatus!=HAL_OK)
		conversionGood=0;

	halStatus=initiateTemperatureConversion(sensor);
	if(halStatus!=HAL_OK)
		conversionGood=0;
	HAL_Delay(3);
	halStatus=readTemperatureData(sensor);
	if(halStatus!=HAL_OK)
		conversionGood=0;
	calculateTemperature(sensor);
	calculatePressure(sensor);
	calculateDepth(sensor);
	return conversionGood;

}

HAL_StatusTypeDef initiatePressureConversion(Bar30* sensor)
{
	uint8_t cmdByte=PRESSURE_CONVERSION_CMD;
	halStatus=HAL_I2C_Master_Transmit(sensor->i2cHandle,BAR30_I2C_ADD,&cmdByte,1,HAL_MAX_DELAY);
	return halStatus;


}
HAL_StatusTypeDef initiateTemperatureConversion(Bar30* sensor)
{
	uint8_t cmdByte=TEMP_CONVERSION_CMD;
	halStatus= HAL_I2C_Master_Transmit(sensor->i2cHandle,BAR30_I2C_ADD,&cmdByte,1,HAL_MAX_DELAY);
	return halStatus;

}
HAL_StatusTypeDef readPressureData(Bar30* sensor){
	uint8_t cmdByte=READ_RESULT_CMD;
	uint8_t rxBuffer[3];
	halStatus= HAL_I2C_Master_Transmit(sensor->i2cHandle,BAR30_I2C_ADD,&cmdByte,1,HAL_MAX_DELAY);
	halStatus=HAL_I2C_Master_Receive(sensor->i2cHandle, BAR30_I2C_ADD, rxBuffer, 3, HAL_MAX_DELAY);
	sensor->rawPressure|=((uint32_t)rxBuffer[0]<<16);
	sensor->rawPressure|=((uint32_t)rxBuffer[1]<<8);
	sensor->rawPressure|=rxBuffer[2];
	return halStatus;

}

HAL_StatusTypeDef readTemperatureData(Bar30* sensor)
{
	uint8_t cmdByte=READ_RESULT_CMD;
	uint8_t rxBuffer[3];
	halStatus= HAL_I2C_Master_Transmit(sensor->i2cHandle,BAR30_I2C_ADD,&cmdByte,1,HAL_MAX_DELAY);
	halStatus=HAL_I2C_Master_Receive(sensor->i2cHandle, BAR30_I2C_ADD, rxBuffer, 3, HAL_MAX_DELAY);
	sensor->rawTemperature|=((uint32_t)rxBuffer[0]<<16);
	sensor->rawTemperature|=((uint32_t)rxBuffer[1]<<8);
	sensor->rawTemperature|=rxBuffer[2];


	return halStatus;
}


uint8_t Bar30CRCcheck(uint16_t* promData)
{
	uint8_t cnt;
	uint32_t remainder=0;
	uint8_t nbit;
	promData[0]=((promData[0])&0x0FFF);
	for(cnt=0;cnt<16;cnt++)
	{
		if(cnt%2==1)
			remainder^=(uint16_t)((promData[cnt>>1])& 0x00FF);
		else
			remainder=(uint16_t)(promData[cnt>>1]>>8);
		for(nbit=8;nbit>0;nbit--)
		{
			if(remainder&(0x8000))
				remainder=(remainder<<1)^0x3000;
			else
				remainder=(remainder<<1);
		}
	}
	remainder=((remainder>>12)&0x000F);
	return ((remainder^0x00)==0);



}

void calculateTemperature(Bar30* sensor)
{
	int32_t temperature=0;
	sensor->dT=sensor->rawTemperature-((uint32_t)(sensor->calibrationResult[5])*256l);
	temperature=2000l+(int64_t)(sensor->dT)*sensor->calibrationResult[6]/8388608LL;
	sensor->actualTemperature=temperature;


}

void calculatePressure(Bar30* sensor)
{
	int32_t dT=sensor->dT;
	int64_t offset=0;
	int64_t sensitivity=0;
	offset=((int64_t)sensor->calibrationResult[2]*65536l) + ((int64_t)(sensor->calibrationResult[4] * dT)/128l);
	sensitivity = (((int64_t)sensor->calibrationResult[1]*32768l)) + (((int64_t)(dT * sensor->calibrationResult[3]))/256l);
	sensor->actualPressure=(sensor->rawPressure*sensitivity/(2097152l)-offset)/(8192l);
	sensor->pressure_mbar=(float)(sensor->actualPressure)/10.0f; // result in millibar


}
float calculateDepth(Bar30* sensor)
{
	sensor->depth=(sensor->pressure_mbar*100.0f-101300.0f)/(FLUID_DENSITY*9.80665f);
	return sensor->depth;
}






