/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "ILI9488.h"
#include "xpt2046.h"
#include "stdio.h"

#include "008_Open_Sans_Bold.h"
#include "009_Open_Sans_Bold.h"
#include "010_Open_Sans_Bold.h"
#include "012_Open_Sans_Bold.h"
#include "014_Open_Sans_Bold.h"
#include "016_Open_Sans_Bold.h"
#include "018_Open_Sans_Bold.h"
#include "020_Open_Sans_Bold.h"
#include "022_Open_Sans_Bold.h"
#include "024_Open_Sans_Bold.h"
#include "026_Open_Sans_Bold.h"
#include "028_Open_Sans_Bold.h"
#include "036_Open_Sans_Bold.h"
#include "048_Open_Sans_Bold.h"
#include "072_Open_Sans_Bold.h"
#include "096_Open_Sans_Bold.h"
#include "112_Open_Sans_Bold.h"
#include "128_Open_Sans_Bold.h"

#define _Open_Sans_Bold_8      &Open_Sans_Bold_8
#define _Open_Sans_Bold_9      &Open_Sans_Bold_9
#define _Open_Sans_Bold_10     &Open_Sans_Bold_10
#define _Open_Sans_Bold_11     &Open_Sans_Bold_11
#define _Open_Sans_Bold_12      &Open_Sans_Bold_12
#define _Open_Sans_Bold_14      &Open_Sans_Bold_14
#define _Open_Sans_Bold_16      &Open_Sans_Bold_16
#define _Open_Sans_Bold_18      &Open_Sans_Bold_18
#define _Open_Sans_Bold_20      &Open_Sans_Bold_20
#define _Open_Sans_Bold_22      &Open_Sans_Bold_22
#define _Open_Sans_Bold_24      &Open_Sans_Bold_24
#define _Open_Sans_Bold_26      &Open_Sans_Bold_26
#define _Open_Sans_Bold_28      &Open_Sans_Bold_28
#define _Open_Sans_Bold_36      &Open_Sans_Bold_36
#define _Open_Sans_Bold_48      &Open_Sans_Bold_48
#define _Open_Sans_Bold_72      &Open_Sans_Bold_72
#define _Open_Sans_Bold_96      &Open_Sans_Bold_96
#define _Open_Sans_Bold_112      &Open_Sans_Bold_112
#define _Open_Sans_Bold_128      &Open_Sans_Bold_128

volatile uint8_t SPI1_TX_completed_flag = 1;

char buf1[20];



#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5
#define BWHITE 6
//#define Aqua 7

#define BYELLOW 8


uint16_t AA[315]={315,314,313,312,311,310,309,308,307,306,305,304,303,302,301,300,299,298,297,296,
		295,294,293,292,291,290,289,288,287,286,285,284,283,282,281,280,279,278,277,276,275,274,
		273,272,271,270,269,268,267,266,265,264,263,262,261,260,259,258,257,256,255,254,253,252,
		251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,236,235,234,233,232,231,230,
		229,228,227,226,225,224,223,222,221,220,219,218,217,216,215,214,213,212,211,210,209,208,
		207,206,205,204,203,202,201,200,199,198,197,196,195,194,193,192,191,190,189,188,187,186,
		185,184,183,182,181,180,179,178,177,176,175,174,173,172,171,170,169,168,167,166,165,164,
		163,162,161,160,159,158,157,156,155,154,153,152,151,150,149,148,147,146,145,144,143,142,
		141,140,139,138,137,136,135,134,133,132,131,130,129,128,127,126,125,124,123,122,121,120,
		119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,
		96,95,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,
		66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,
		36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,
		4,3,2,1};

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t RxBuffer[25];
uint8_t rxComplete=0;
uint8_t firstReception=1;
float depth;
float velocity;
float roll;
float pitch;
float yaw;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	rxComplete=1;
	firstReception=0;
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	SPI1_TX_completed_flag = 1;
}
/* USER CODE END 0 */
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
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1,RxBuffer,25);
  HAL_Delay(1000);
  ILI9488_Init();

 //   HAL_Delay(1000);
    setRotation(1);



    ILI9341_Fill_Screen(WHITE);

    uint16_t touchx = 0, touchy = 0;


    	  	  ILI9341_Fill_Screen(GREEN);
    	  	  ILI9341_Fill_Screen(BLUE);
    	  	  ILI9341_Fill_Screen(RED);
    	  	  ILI9341_Fill_Screen(ILI9488_MAGENTA);
    	  	  ILI9341_Fill_Screen(ILI9488_YELLOW);
    	  	  ILI9341_Fill_Screen(ILI9488_WHITE);
    	  	  ILI9341_Fill_Screen(ILI9488_ORANGE);
    	  	  ILI9341_Fill_Screen(ILI9488_OLIVE);

    	  	  ILI9341_Fill_Screen(ILI9488_GREEN);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_BLUE);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(RED);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_MAGENTA);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_YELLOW);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_WHITE);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, ILI9488_BLUE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, ILI9488_BLUE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_ORANGE);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_OLIVE);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);

    	  	  ILI9341_Fill_Screen(GREEN);
    	  	  ILI9341_Fill_Screen(BLUE);
    	  	  ILI9341_Fill_Screen(RED);
    	  	  ILI9341_Fill_Screen(ILI9488_MAGENTA);
    	  	  ILI9341_Fill_Screen(ILI9488_YELLOW);
    	  	  ILI9341_Fill_Screen(ILI9488_WHITE);
    	  	  ILI9341_Fill_Screen(ILI9488_ORANGE);
    	  	  ILI9341_Fill_Screen(ILI9488_OLIVE);

    	  	  ILI9341_Fill_Screen(ILI9488_GREEN);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_BLUE);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(RED);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_MAGENTA);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_YELLOW);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_WHITE);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, ILI9488_BLUE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, ILI9488_BLUE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_ORANGE);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);
    	  	  ILI9341_Fill_Screen(ILI9488_OLIVE);LCD_Font(20, 100, "ISMAIL", _Open_Sans_Bold_112  , 1, WHITE);LCD_Font(10, 250, "STM32", _Open_Sans_Bold_112  , 1, WHITE);HAL_Delay(1000);

    	  //    CODE FOR TOUCH


    	  	  char buf[20],buf1[20];
    //	  	  LCD_Font(25, 25, buf1, _Open_Sans_Bold_24  , 1, WHITE);
    //	  	  touchX = getX();
    //	  	  touchY = getY();
    //	  	  sprintf(buf,"x=%3d y=%3d",touchX,touchY);
    //	  	  sprintf(buf1,"x=%3d y=%3d",touchX,touchY);
    //	  	  LCD_Font(25, 25, buf, _Open_Sans_Bold_24  , 1, BLACK);
    //	  	  fillRect(touchX, touchY,2,2, RED);
    	  	ILI9341_Fill_Screen(WHITE);

    LCD_Font(165, 25, "Slider 1", _Open_Sans_Bold_24, 1, BLACK);
    LCD_Font(165, 80, "Slider 2", _Open_Sans_Bold_24, 1, BLACK);
    LCD_Font(165, 145, "Slider 3", _Open_Sans_Bold_24, 1, BLACK);

    LCD_Rect_Round(165, 30, 280, 20, 5,1, BLUE);
    LCD_Rect_Round(165, 90, 280, 20, 5,1, RED);
    LCD_Rect_Round(165, 150, 280, 20, 5,1, BLACK);


    LCD_Rect_Round_Fill(165, 30, 140, 20, 5, BLUE);
    LCD_Rect_Round_Fill(165, 90, 70, 20, 5, RED);
    LCD_Rect_Round_Fill(165, 150, 210, 20, 5, GREEN);



    LCD_Rect_Round_Fill(190, 200, 100, 50, 5, GREEN_D);

    LCD_Rect_Round_Fill(315, 200, 100, 50, 5, GREEN_D);

    LCD_Font(200, 230, "LED ON", _Open_Sans_Bold_20, 1, WHITE);

    LCD_Font(320, 230, "LED OFF", _Open_Sans_Bold_20, 1, WHITE);

    LCD_Rect_Round_Fill(190, 260, 100, 50, 5, GREEN_D);

    LCD_Rect_Round_Fill(315, 260, 100, 50, 5, GREEN_D);

    LCD_Font(200, 290, "LED ON", _Open_Sans_Bold_20, 1, WHITE);

    LCD_Font(320, 290, "LED OFF", _Open_Sans_Bold_20, 1, WHITE);



    ringMeter1(500, 0, 1020, 25, 30, 60,15,YELLOW, RED2RED);
    ringMeter1(750, 0, 1020, 25, 180, 60,15,YELLOW, BLUE2BLUE);

 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(rxComplete==1)
	  {
		  int i;
		      for (i = 0; i < 25; i++) {
		          if (RxBuffer[i] == 'D' && i + 4 < 25) {

		              *((uint8_t*)&depth + 0) = RxBuffer[i + 1];
		              *((uint8_t*)&depth + 1) = RxBuffer[i + 2];
		              *((uint8_t*)&depth + 2) = RxBuffer[i + 3];
		              *((uint8_t*)&depth + 3) = RxBuffer[i + 4];

		          }
		          else if (RxBuffer[i] == 'R' && i + 4 < 25) {

		              *((uint8_t*)&roll + 0) = RxBuffer[i + 1];
		              *((uint8_t*)&roll + 1) = RxBuffer[i + 2];
		              *((uint8_t*)&roll + 2) = RxBuffer[i + 3];
		              *((uint8_t*)&roll + 3) = RxBuffer[i + 4];

		          }
		          else if (RxBuffer[i] == 'P' && i + 4 < 25) {

		              *((uint8_t*)&pitch + 0) = RxBuffer[i + 1];
		              *((uint8_t*)&pitch + 1) = RxBuffer[i + 2];
		              *((uint8_t*)&pitch + 2) = RxBuffer[i + 3];
		              *((uint8_t*)&pitch + 3) = RxBuffer[i + 4];

		          }

		          else if (RxBuffer[i] == 'V' && i + 4 < 25) {

					  *((uint8_t*)&velocity + 0) = RxBuffer[i + 1];
					  *((uint8_t*)&velocity + 1) = RxBuffer[i + 2];
					  *((uint8_t*)&velocity + 2) = RxBuffer[i + 3];
					  *((uint8_t*)&velocity + 3) = RxBuffer[i + 4];

		         		          }
		          else if (RxBuffer[i] == 'Y' && i + 4 < 25) {

		              *((uint8_t*)&yaw + 0) = RxBuffer[i + 1];
		              *((uint8_t*)&yaw + 1) = RxBuffer[i + 2];
		              *((uint8_t*)&yaw + 2) = RxBuffer[i + 3];
		              *((uint8_t*)&yaw + 3) = RxBuffer[i + 4];

		          }
		      }
	  }
	  rxComplete=0;

	  LCD_Font(0, 18, buf1, _Open_Sans_Bold_24  , 1, WHITE);
	 	  	  touchx = getX();
	 	  	  touchy = getY();
	 	  	  sprintf(buf,"x=%3d y=%3d",touchx,touchy);
	 	  	  sprintf(buf1,"x=%3d y=%3d",touchx,touchy);
	 	  	  LCD_Font(0, 18, buf, _Open_Sans_Bold_24  , 1, BLACK);


	 	  	  	if(touchx >=155 && touchx<=460 && touchy>=20 && touchy<=30)
	 	  		{

	 	  		   LCD_Rect_Round_Fill(165, 30, 280, 20, 5, WHITE);
	 	  		   LCD_Rect_Round(165, 30, 280, 20, 5,1, BLUE);
	 	  		   LCD_Rect_Round_Fill(165, 30, touchx-155, 20, 5, BLUE);

	 	  		 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, AA[touchx-150]);

	 	  		ringMeter1(touchx*2, 0, 1020, 25, 30, 60,15,YELLOW, RED2RED);

	 	  		}

	 	  	  	if(touchx >=155 && touchx<=460 && touchy>=80 && touchy<=100)
	 	  		{

	 	  		   LCD_Rect_Round_Fill(165, 90, 280, 20, 5, WHITE);
	 	  		   LCD_Rect_Round(165, 90, 280, 20, 5,1, RED);
	 	  		   LCD_Rect_Round_Fill(165, 90, touchx-155, 20, 5, RED);

	 	  		 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, AA[touchx-150]);

	 	  		ringMeter1(touchx*2, 0, 1020, 25, 180, 60,15,YELLOW, BLUE2BLUE);

	 	  		}


	 	  	  	if(touchx >=155 && touchx<=460 && touchy>=140 && touchy<=160)
	 	  		{

	 	  		   LCD_Rect_Round_Fill(165, 150, 280, 20, 5, WHITE);
	 	  		   LCD_Rect_Round(165, 150, 280, 20, 5,1, GREEN);
	 	  		   LCD_Rect_Round_Fill(165, 150, touchx-155, 20, 5, GREEN);

	 	  		 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, AA[touchx-150]);


	 	  		}


	 	  	  	if(touchx >=180 && touchx<=290 && touchy>=190 && touchy<=240)
	 	  		{
	 	  	  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	 	  		}

	 	  	  	if(touchx >=180 && touchx<=290 && touchy>=250 && touchy<=295)
	 	  		{
	 	  	  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	 	  		}


	 	  	  	if(touchx >=325 && touchx<=435 && touchy>=190 && touchy<=240)
	 	  		{
	 	  	  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	 	  		}

	 	  	  	if(touchx >=325 && touchx<=435 && touchy>=250 && touchy<=300)
	 	  		{
	 	  	  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	 	  		}



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 336;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_CS_Pin|TFT_RST_Pin|TFT_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TP_IRQ_GPIO_Port, TP_IRQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_CS_Pin TP_CS_Pin */
  GPIO_InitStruct.Pin = TFT_CS_Pin|TP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_RST_Pin TFT_DC_Pin */
  GPIO_InitStruct.Pin = TFT_RST_Pin|TFT_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_IRQ_Pin */
  GPIO_InitStruct.Pin = TP_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
