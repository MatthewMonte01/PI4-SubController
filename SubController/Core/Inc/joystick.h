

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

#define JOYSTICK_MIN_THRESHOLD 0.6f
#define JOYSTICK_MAX_THRESHOLD 2.5f



extern ADC_HandleTypeDef hadc1;
typedef struct
{
	uint32_t joystickData[2]; // [1]-> horizontal output, [2]->verticalOutput
	float joystickVoltage[2];
}Joystick;

//void readJoystick(Joystick* stick)
//{
//	HAL_ADC_Start_DMA(&hadc1,stick->joystickData,2);
//	convertRaw2Voltage(stick);
//
//}

//void convertRaw2Voltage( Joystick* stick)
//{
//	stick->joystickVoltage[0]=stick->joystickData[0]*3.3/4095.0f;
//	stick->joystickVoltage[1]=stick->joystickData[1]*3.3/4095.0f;
//
//
//}



#endif /* INC_JOYSTICK_H_ */
