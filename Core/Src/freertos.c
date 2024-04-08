/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "rtc.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern uint8_t caractere_recu;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Tache1Handle;
osThreadId Tache2Handle;
osThreadId DisplayHandle;
osMessageQId QueueTimeHandle;
osMessageQId QueueSerieHandle;
osMutexId Mutex_AffichageHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void tache1_fonction(void const * argument);
void tache2_fonction(void const * argument);
void Display_fonction(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of Mutex_Affichage */
  osMutexDef(Mutex_Affichage);
  Mutex_AffichageHandle = osMutexCreate(osMutex(Mutex_Affichage));

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
  /* definition and creation of QueueTime */
  osMessageQDef(QueueTime, 10, RTC_TimeTypeDef);
  QueueTimeHandle = osMessageCreate(osMessageQ(QueueTime), NULL);

  /* definition and creation of QueueSerie */
  osMessageQDef(QueueSerie, 4, uint8_t);
  QueueSerieHandle = osMessageCreate(osMessageQ(QueueSerie), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Tache1 */
  osThreadDef(Tache1, tache1_fonction, osPriorityLow, 0, 1024);
  Tache1Handle = osThreadCreate(osThread(Tache1), NULL);

  /* definition and creation of Tache2 */
  osThreadDef(Tache2, tache2_fonction, osPriorityHigh, 0, 1024);
  Tache2Handle = osThreadCreate(osThread(Tache2), NULL);

  /* definition and creation of Display */
  osThreadDef(Display, Display_fonction, osPriorityNormal, 0, 1024);
  DisplayHandle = osThreadCreate(osThread(Display), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_tache1_fonction */
/**
* @brief Function implementing the Tache1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tache1_fonction */
void tache1_fonction(void const * argument)
{
  /* USER CODE BEGIN tache1_fonction */
	RTC_TimeTypeDef sTime; // Structure pour stocker l'heure
	RTC_DateTypeDef sDate;
	/* Infinite loop */
	for (;;) {

		HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, 1);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		xQueueSend(QueueTimeHandle, &sTime, 0);
		HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, 0);
		osDelay(50);
	}
  /* USER CODE END tache1_fonction */
}

/* USER CODE BEGIN Header_tache2_fonction */
/**
* @brief Function implementing the Tache2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tache2_fonction */
void tache2_fonction(void const * argument)
{
  /* USER CODE BEGIN tache2_fonction */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, 1);

		//xQueueSend(QueueSerieHandle, &valLed, 0);
		//HAL_GPIO_WritePin(LED16_GPIO_Port, LED16_Pin, valLed);

		HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, 0);
		osDelay(1000);

	}
  /* USER CODE END tache2_fonction */
}

/* USER CODE BEGIN Header_Display_fonction */
/**
* @brief Function implementing the Display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Display_fonction */
void Display_fonction(void const * argument)
{
  /* USER CODE BEGIN Display_fonction */
	/* Infinite loop */
	char text[50] = { }, text2[50] = { };
	uint8_t valLed = 0;
	RTC_TimeTypeDef sTime; // Structure pour stocker l'heure
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, 1); ////||(xQueueReceive(QueueSerieHandle, &valLed, 0)))
		if (xQueueReceive(QueueTimeHandle, &sTime, 0)) {
			sprintf(text2, "Heure actuelle: %02d:%02d:%02d", sTime.Hours,
					sTime.Minutes, sTime.Seconds);
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			BSP_LCD_DisplayStringAtLine(3, (uint8_t*) text2);

		}
		if (xQueueReceive(QueueSerieHandle, &valLed, 0)) {
			sprintf(text, "LED 16: %d", valLed);
			BSP_LCD_DisplayStringAtLine(2, (uint8_t*) text);
		}
		HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, 0);
		osDelay(100);
	}
  /* USER CODE END Display_fonction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

