/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"
#include "MPU6050.h"
#include "joystick.h"
#include "Bar30.h"
#include "LCDcommunication.h"
#include "KalmanRollPitch.h"
#include "servos.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// SD card variables
SPI_HandleTypeDef hspi2;
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
char buffer[100];


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId DefaultTaskHandle;
osThreadId LEDscreenTaskHandle;
osThreadId controlLoopTaskHandle;
osThreadId SDcardTaskHandle;
osThreadId pressureSensorTHandle;
osThreadId KalmanPredictHandle;
osThreadId KalmanUpdateHandle;
osMessageQId senderHandle;
osMessageQId receiverHandle;
/* USER CODE BEGIN PV */

Joystick joystick;

EKF ekf;
MPU6050 mpu6050;
Bar30 pressureSensor;

uint8_t readCount=0;
uint8_t message=0;
uint8_t mpuIntCount=0;

float* accelx=&mpu6050.acc_mps2[0];
float* accely=&mpu6050.acc_mps2[1];
float* accelz=&mpu6050.acc_mps2[2];

float rollSpeed;
float pitchSpeed;
float yawSpeed;



// screen buffers and states
uint8_t uartRxBuffer[8];
uint8_t uartTxBuffer[8];
uint8_t* txbufptr=&uartTxBuffer[0];
uint8_t transferControlByte=0x00;
static HAL_StatusTypeDef txStatus;

State currentState=IDLE_STATE;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
static void MX_USART3_UART_Init(void);
void defaultTask(void const * argument);
void sendDataToScreen(void const * argument);
void updateControlLoop(void const * argument);
void recordSDdata(void const * argument);
void getBar30Data(void const * argument);
void kalmanPredict(void const * argument);
void kalmanUpdate(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin==INT_MPU6050_Pin) //interrupt called at 1kHz
	{
		mpuIntCount=mpuIntCount+1;
		if(mpuIntCount==20){ // downsample MPU6050 data at 50 Hz
			message=MPU6050readDataDMA(&mpu6050);
			//printf("error message: %i\r\n:",message);
			mpuIntCount=0;
		}
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
	if(hi2c->Instance==mpu6050.i2cHandle->Instance)
	{
		mpu6050.rxFlag=0;
		MPU6050convertRawData(&mpu6050);
		MPU6050filterRawData(&mpu6050);

	}

}

void transmit_uart(char* string){
	uint8_t len=strlen(string);
	HAL_UART_Transmit(&huart2,(uint8_t*)string,len,200);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
//	if(hadc->Instance==ADC1){
//		printf("Joystick x %.2f\r\n",joystick.joystickVoltage[0]);
//		printf("Joystick y %.2f\r\n",joystick.joystickVoltage[1]);
//	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART3)
	{
		if(currentState==IDLE_STATE)
		{
			if(transferControlByte==0xFF)
				currentState=SEND_ACK_STATE;
		}

		else if(currentState==SENDING_DATA_STATE)
		{
			currentState==IDLE_STATE;
		}
		else
			return;
	}
	transferControlByte=0x00;




}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{


}

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start servo PWM ( pulse duration from 0.9ms to 2.1ms with 1.5 as center ) duty cycle varies from 4.5% to 7.5%
  //( CCR can go from 425 to 1175 )
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

  htim3.Instance->CCR1=SERVO_CENTER_PWM;
  htim3.Instance->CCR2=SERVO_CENTER_PWM;
  htim3.Instance->CCR3=SERVO_CENTER_PWM;
  htim3.Instance->CCR4=SERVO_CENTER_PWM;

  // EKF initialisation
  float KalmanQ[2]={KALMAN_Q,KALMAN_Q};
  float KalmanR[3]={KALMAN_R,KALMAN_R,KALMAN_R};
  float KalmanP[2]={KALMAN_P_INIT,KALMAN_P_INIT};
  EKF_Init(&ekf,KalmanP,KalmanQ,KalmanR);


  // Start DMA streams
  HAL_ADC_Start_DMA(&hadc1,joystick.joystickData,2);
  HAL_TIM_Base_Start(&htim2);
  HAL_UART_Receive_DMA(&huart3,uartRxBuffer,8);


  // sensor initialization
  uint8_t deviceReady;
  deviceReady=checkMPU6050Ready();
  uint8_t configGood=MPU6050init(&mpu6050,&hi2c1 );
  uint8_t rec;
  HAL_StatusTypeDef stat;
  stat=HAL_I2C_Mem_Read (&hi2c1,MPU6050ADDR,WHO_AM_I_REG,I2C_MEMADD_SIZE_8BIT,&rec,1,100);
  Bar30init(&pressureSensor, &hi2c2);
  uint8_t i2cGood=0;
  i2cGood=Bar30reset(&pressureSensor);
  printf("Reset Command Result : %i\r\n",i2cGood );
  i2cGood=Bar30getCalibration(&pressureSensor);
  printf("Calibration phase result : %i\r\n",i2cGood );
  i2cGood=Bar30CRCcheck(pressureSensor.calibrationResult);

  HAL_UART_Receive_DMA(&huart3,&transferControlByte,1);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of sender */
  osMessageQDef(sender, 32, uint16_t);
  senderHandle = osMessageCreate(osMessageQ(sender), NULL);

  /* definition and creation of receiver */
  osMessageQDef(receiver, 16, uint16_t);
  receiverHandle = osMessageCreate(osMessageQ(receiver), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DefaultTask */
  osThreadDef(DefaultTask, defaultTask, osPriorityIdle, 0, 128);
  DefaultTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);

  /* definition and creation of LEDscreenTask */
  osThreadDef(LEDscreenTask, sendDataToScreen, osPriorityNormal, 0, 128);
  LEDscreenTaskHandle = osThreadCreate(osThread(LEDscreenTask), NULL);

  /* definition and creation of controlLoopTask */
  osThreadDef(controlLoopTask, updateControlLoop, osPriorityHigh, 0, 128);
  controlLoopTaskHandle = osThreadCreate(osThread(controlLoopTask), NULL);

  /* definition and creation of SDcardTask */
  osThreadDef(SDcardTask, recordSDdata, osPriorityAboveNormal, 0, 2048);
  SDcardTaskHandle = osThreadCreate(osThread(SDcardTask), NULL);

  /* definition and creation of pressureSensorT */
  osThreadDef(pressureSensorT, getBar30Data, osPriorityNormal, 0, 256);
  pressureSensorTHandle = osThreadCreate(osThread(pressureSensorT), NULL);

  /* definition and creation of KalmanPredict */
  osThreadDef(KalmanPredict, kalmanPredict, osPriorityNormal, 0, 256);
  KalmanPredictHandle = osThreadCreate(osThread(KalmanPredict), NULL);

  /* definition and creation of KalmanUpdate */
  osThreadDef(KalmanUpdate, kalmanUpdate, osPriorityIdle, 0, 256);
  KalmanUpdateHandle = osThreadCreate(osThread(KalmanUpdate), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //readJoystick(&joystick);

	  //sendDataToScreen();
	  //uartReceiveFlag=0;
//	  i2cGood=Bar30getData(&pressureSensor);
//
//	  HAL_Delay(10);
//	  printf("Data Conversion Result : %i\r\n",i2cGood );
//	  printf("Ambient presure in mbar is: %.2f\r\n",(float)(pressureSensor.actualPressure)/100.0f);



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 168;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_MPU6050_Pin */
  GPIO_InitStruct.Pin = INT_MPU6050_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_MPU6050_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
* @brief Retargets the C library printf function to the USART.
* @param None
* @retval None
*/
PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the USART2 and Loop until the end
of transmission */
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
return ch;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_defaultTask */
/**
  * @brief  Function implementing the DefaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_defaultTask */
void defaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_sendDataToScreen */
/**
* @brief Function implementing the LEDscreenTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sendDataToScreen */
void sendDataToScreen(void const * argument)
{
  /* USER CODE BEGIN sendDataToScreen */




  /* Infinite loop */
  for(;;)
  {
	  switch(currentState){
		  case IDLE_STATE:
			  // request byte is received in background via cicular DMA
			  break;

		  case SEND_ACK_STATE:
			  uint8_t ackByte=0x10;
			  HAL_UART_Transmit(&huart3,&ackByte,1,100);
			  break;


		  case SENDING_DATA_STATE:
			  uartTxBuffer[0]=0xAA;
			  uartTxBuffer[1]=0xBB;
			  uartTxBuffer[2]=0xCC;
			  uartTxBuffer[3]=0xDD;
			  uartTxBuffer[4]=0xEE;
			  uartTxBuffer[5]=0xFF;
			  uartTxBuffer[6]=0x00;
			  uartTxBuffer[7]=0x01;
			  txStatus=HAL_UART_Transmit(&huart3,txbufptr,8,10);
			  break;

	  }


	  osDelay(50000); //
  }


  /* USER CODE END sendDataToScreen */
}

/* USER CODE BEGIN Header_updateControlLoop */
/**
* @brief Function implementing the controlLoopTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_updateControlLoop */
void updateControlLoop(void const * argument)
{
  /* USER CODE BEGIN updateControlLoop */

		static uint8_t horizontalCommand=0;
		static uint8_t verticalCommand=0;

	  /* Infinite loop */
	  for(;;)
	  {
		readJoystick(&joystick);
		if(joystick.joystickVoltage[0]<JOYSTICK_MIN_THRESHOLD ) // go left
		{
			if(verticalCommand=1)
				neutralRudders();
			turnLeft();
			horizontalCommand=1;
			verticalCommand=0;

		}
		else if(joystick.joystickVoltage[0]>JOYSTICK_MAX_THRESHOLD)//go Right
		{
			if(verticalCommand=1)
				neutralRudders();
			turnRight();
			horizontalCommand=1;
			verticalCommand=0;
		}

		else if (joystick.joystickVoltage[1]<JOYSTICK_MIN_THRESHOLD)//dive
		{
			if(horizontalCommand=1)
				neutralRudders();
			dive();
			verticalCommand=1;
			horizontalCommand=0;
		}
		else if(joystick.joystickVoltage[1]>JOYSTICK_MAX_THRESHOLD)//surface
		{
			if(horizontalCommand=1)
				neutralRudders();
			surface();
			verticalCommand=1;
			horizontalCommand=0;
		}


	    osDelay(20); // update control loop every 20 ms
	  }
  /* USER CODE END updateControlLoop */
}

/* USER CODE BEGIN Header_recordSDdata */
/**
* @brief Function implementing the SDcardTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_recordSDdata */
void recordSDdata(void const * argument)
{
  /* USER CODE BEGIN recordSDdata */
  /* Infinite loop */
  for(;;)
  {

	    // Waiting for the Micro SD module to initialize
	  	printf("SD card thread is called!");

	  	fres = f_mount(&fs, "", 0);
	  	if (fres == FR_OK) {
	  		printf("Micro SD card is mounted successfully!\n");
	  	} else if (fres != FR_OK) {
	  		printf("Micro SD card's mount error!\n");
	  	}

	  	// FA_OPEN_APPEND opens file if it exists and if not then creates it,
	  	// the pointer is set at the end of the file for appending
	  	fres = f_open(&fil, "accel.txt", FA_OPEN_APPEND | FA_WRITE | FA_READ);
	  	if (fres == FR_OK) {
	  		printf("File opened for reading and checking the free space.\n");
	  	} else if (fres != FR_OK) {
	  		printf("File was not opened for reading and checking the free space!\n");
	  	}

	  	fres = f_getfree("", &fre_clust, &pfs);
	  	totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	  	freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);
	  	char mSz[12];
	  	sprintf(mSz, "%lu", freeSpace);
	  	if (fres == FR_OK) {
	  		printf("The free space is: ");
	  		printf(mSz);
	  		printf("\n");
	  	} else if (fres != FR_OK) {
	  		printf("The free space could not be determined!\n");
	  	}

//	  	for (uint8_t i = 0; i < 10; i++) {
//	  		f_puts("NEW BOARD TEST.\n", &fil);
//	  	}
	  	char accDataString[50];
	  	sprintf(accDataString, "ax=%3f, ay=%3f, az=%3f\n", mpu6050.acc_mps2[0],  mpu6050.acc_mps2[1],  mpu6050.acc_mps2[2]);
	  	f_puts(accDataString, &fil);

	  	fres = f_close(&fil);
	  	if (fres == FR_OK) {
	  		printf("The file is closed.\n");
	  	} else if (fres != FR_OK) {
	  		printf("The file was not closed.\n");
	  	}

//	  	// Open file to read
//	  	fres = f_open(&fil, "hi.txt", FA_READ);
//	  	if (fres == FR_OK) {
//	  		printf("File opened for reading.\n");
//	  	} else if (fres != FR_OK) {
//	  		printf("File was not opened for reading!\n");
//	  	}
//
//	  	while (f_gets(buffer, sizeof(buffer), &fil)) {
//	  		char mRd[100];
//	  		sprintf(mRd, "%s", buffer);
//	  		printf(mRd);
//
//	  	}

//	  	 //Close file
//	  	fres = f_close(&fil);
//	  	if (fres == FR_OK) {
//	  		printf("The file is closed.\n");
//	  	} else if (fres != FR_OK) {
//	  		printf("The file was not closed.\n");
//	  	}

	  	f_mount(NULL, "", 1);
	  	if (fres == FR_OK) {
	  		printf("The Micro SD card is unmounted!\n");
	  	} else if (fres != FR_OK) {
	  		printf("The Micro SD was not unmounted!");
	  	}
    osDelay(5000); // write SD card data every 5 seconds
  }
  /* USER CODE END recordSDdata */
}

/* USER CODE BEGIN Header_getBar30Data */
/**
* @brief Function implementing the pressureSensorT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_getBar30Data */
void getBar30Data(void const * argument)
{
  /* USER CODE BEGIN getBar30Data */
  /* Infinite loop */
	static uint8_t i2cGood=1;
  for(;;)
  {
	 i2cGood=Bar30getData(&pressureSensor);
    osDelay(250);
  }
  /* USER CODE END getBar30Data */
}

/* USER CODE BEGIN Header_kalmanPredict */
/**
* @brief Function implementing the KalmanPredict thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_kalmanPredict */
void kalmanPredict(void const * argument)
{
  /* USER CODE BEGIN kalmanPredict */
  /* Infinite loop */
  for(;;)
  {
	EKF_Predict(&ekf, mpu6050.gyr_rps[0], mpu6050.gyr_rps[1], mpu6050.gyr_rps[2], 0.02f);
    osDelay(20);
  }
  /* USER CODE END kalmanPredict */
}

/* USER CODE BEGIN Header_kalmanUpdate */
/**
* @brief Function implementing the KalmanUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_kalmanUpdate */
void kalmanUpdate(void const * argument)
{
  /* USER CODE BEGIN kalmanUpdate */
  /* Infinite loop */
  for(;;)
  {
	EKF_Update(&ekf, mpu6050.acc_mps2[0], mpu6050.acc_mps2[1], mpu6050.acc_mps2[2]);
    osDelay(200);
  }
  /* USER CODE END kalmanUpdate */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
