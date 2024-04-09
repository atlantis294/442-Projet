/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "dma2d.h"
#include "fatfs.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stdio.h"
#include "stdlib.h"
#include "bitmaps.h"
#include "fatfs_storage.h"
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

/* USER CODE BEGIN PV */
uint8_t *uwInternelBuffer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

ADC_ChannelConfTypeDef sConfig = {0};
uint8_t caractere_recu;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_UART7_Init();
  MX_SDMMC1_SD_Init();
  MX_ADC3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+ BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4);
  BSP_LCD_DisplayOn();
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SelectLayer(1);
  BSP_LCD_Clear(00);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_SetBackColor(00);
  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

  uint8_t fatfs;
  //pb configuration fatfs
  fatfs=FATFS_LinkDriver(&SD_Driver, SDPath);

	char image[200]={};
  char text[50]={};
	FIL file;
	uwInternelBuffer = (uint8_t*) 0xC0260000;
	unsigned int byteRead;
	TCHAR pathfile[] = "mapENS.bmp";
    FRESULT res;
	res=f_open(&file, pathfile, FA_READ); // on ne prend que le fichier
	f_read(&file, (TCHAR*) image, 200, &byteRead);
	f_close(&file);

  sprintf(text, "file: %d  fatfs: %d      ",res,fatfs);
	BSP_LCD_DisplayStringAtLine(8,(uint8_t*) text);

	//Entete début
	//Largeur
	image[18] = 0xE0;
	image[19] = 0x01;
	image[20] = 0x00;
	image[21] = 0x00;
	//hauteur
	image[22] = 0x10;
	image[23] = 0x01;
	image[24] = 0x00;
	image[25] = 0x00;



  uint8_t Test[25]="Fin init\r\n";
  HAL_UART_Transmit(&huart1,Test,sizeof(Test),10);
  HAL_UART_Receive_IT(&huart1,&caractere_recu,1);
  //HAL_RTC_GetDate()
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  	//HAL_RTC_GetTime()
	  	 // Bouton + LED
	  	//HAL_GPIO_WritePin(LED13_GPIO_Port,LED13_Pin,HAL_GPIO_ReadPin(BP1_GPIO_Port,BP1_Pin));
		//HAL_GPIO_WritePin(LED14_GPIO_Port,LED14_Pin,HAL_GPIO_ReadPin(BP2_GPIO_Port,BP2_Pin));
		//sprintf(text,"BP1 : %d",HAL_GPIO_ReadPin(BP1_GPIO_Port,BP1_Pin));
		//BSP_LCD_DisplayStringAtLine(5,(uint8_t*) text);
//	  	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
//	  	sprintf(text,"BP : %d",HAL_GPIO_ReadPin(BP1_GPIO_Port,BP1_Pin));
//	  	BSP_LCD_DisplayStringAtLine(1,(uint8_t*) text);

	  	// Potar 1
		//sConfig.Channel = ADC_CHANNEL_6;
		//HAL_ADC_ConfigChannel(&hadc3, &sConfig);
		//HAL_ADC_Start(&hadc3);
		//while(HAL_ADC_PollForConversion(&hadc3, 100)!=HAL_OK);
		//potr = HAL_ADC_GetValue(&hadc3);

//
//	  	RTC_DateTypeDef sDate;
//	  	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
//	  	sprintf(text,"Date actuelle: %02d-%02d-%02d", sDate.Year, sDate.Month, sDate.Date);
//	  	BSP_LCD_DisplayStringAtLine(2,(uint8_t*) text);
//
//	  	RTC_TimeTypeDef sTime; // Structure pour stocker l'heure
//		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
//		sprintf(text2,"Heure actuelle: %02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
//		BSP_LCD_DisplayStringAtLine(3,(uint8_t*) text2);
//
//		HAL_UART_Receive(&huart1,&caractere_recu,1,1);
//		if(caractere_recu=='a') {
//			strcat(text2,"\r\n");
//			HAL_UART_Transmit(&huart1,text2,sizeof(text2),10);
//			caractere_recu=0;
//		}
//
//		sConfig.Channel = ADC_CHANNEL_8;
//		HAL_ADC_ConfigChannel(&hadc3, &sConfig);
//		HAL_ADC_Start(&hadc3);
//		while(HAL_ADC_PollForConversion(&hadc3, 100)!=HAL_OK);
//		joystick_v = HAL_ADC_GetValue(&hadc3);
//
//		HAL_ADC_Start(&hadc1);
//		while(HAL_ADC_PollForConversion(&hadc1, 100)!=HAL_OK);
//		joystick_h = HAL_ADC_GetValue(&hadc1);
//
//		BSP_LCD_SetTextColor(LCD_COLOR_RED);
//		sprintf(text,"POTL : %4u joy_v : %4u joy_h : %4u",(uint16_t)potl,(uint16_t)joystick_v,(uint16_t)joystick_h);
//		BSP_LCD_DisplayStringAtLine(9,(uint8_t*) text);
//
//		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
//		BSP_TS_GetState(&TS_State);
//
//		sprintf(text,"                     ");
//		BSP_LCD_DisplayStringAtLine(5,(uint8_t*) text);
//		if(TS_State.touchDetected){
//			sprintf(text,"   Essai de l'ASPIQUE");
//			BSP_LCD_DisplayStringAtLine(5,(uint8_t*) text);
//		  BSP_LCD_FillCircle(TS_State.touchX[0],TS_State.touchY[0],4);
//	  	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// ==============================================================================================================

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (caractere_recu == 'a')
		HAL_GPIO_WritePin(LED16_GPIO_Port, LED16_Pin, 1);
	if (caractere_recu == 'e')
		HAL_GPIO_WritePin(LED16_GPIO_Port, LED16_Pin, 0);
	HAL_UART_Receive_IT(&huart1, &caractere_recu, 1);

}


//===============================================================================================================

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
