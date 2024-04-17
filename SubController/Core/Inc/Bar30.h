#include <stdint.h>
#include "stm32f4xx.h"

#ifndef INC_BAR30_H_
#define INC_BAR30_H_

#define BAR30_I2C_ADD 			    0x76<<1


#define RESET_CMD 				    0x1E
#define PRESSURE_CONVERSION_CMD     0x44
#define TEMP_CONVERSION_CMD         0x54
#define READ_RESULT_CMD             0x00

#define READ_PROM_CMD_1              0xA0
#define READ_PROM_CMD_2              0xA2
#define READ_PROM_CMD_3              0xA4
#define READ_PROM_CMD_4              0xA6
#define READ_PROM_CMD_5              0xA8
#define READ_PROM_CMD_6              0xAA
#define READ_PROM_CMD_7              0xAC

# define FLUID_DENSITY               992.72f // fluid density of pool water in Kg/m^3


typedef struct
{
	I2C_HandleTypeDef *i2cHandle;
	uint32_t rawPressure;
	uint32_t rawTemperature;
	int32_t actualPressure;
	float pressure_mbar;
	int32_t actualTemperature;
	uint16_t calibrationResult[7]; // [CRC C1 C2 C3 C4 C5 C6 C7]
	uint8_t rxFlag;
	uint8_t conversionCompleteFlag;
	int32_t dT;
	float depth;
} Bar30;

uint8_t Bar30reset(Bar30* sensor);
void Bar30init(Bar30* sensor, I2C_HandleTypeDef* i2cHandle);
uint8_t Bar30getCalibration(Bar30* sensor);
uint8_t Bar30getData(Bar30* sensor);
uint8_t Bar30CRCcheck(uint16_t* promData);
void calculateTemperature(Bar30* sensor);
void calculatePressure(Bar30* sensor);
float calculateDepth(Bar30* sensor);
HAL_StatusTypeDef initiatePressureConversion(Bar30* sensor);
HAL_StatusTypeDef initiateTemperatureConversion(Bar30* sensor);
HAL_StatusTypeDef readPressureData(Bar30* sensor);
HAL_StatusTypeDef readTemperatureData(Bar30* sensor);




#endif /* INC_BAR30_H_ */

