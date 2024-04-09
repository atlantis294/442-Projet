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
osThreadId deplacementHandle;
osThreadId DisplayHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void deplacement_fonction(void const * argument);
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

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of deplacement */
  osThreadDef(deplacement, deplacement_fonction, osPriorityHigh, 0, 1024);
  deplacementHandle = osThreadCreate(osThread(deplacement), NULL);

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

/* USER CODE BEGIN Header_deplacement_fonction */
/**
* @brief Function implementing the deplacement thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_deplacement_fonction */
void deplacement_fonction(void const * argument)
{
  /* USER CODE BEGIN deplacement_fonction */
  TS_StateTypeDef TS_State,TS_State0,TS_State1;
  uint8_t deplacement=0,dx,dy;
  char text[50]={};
  /* Infinite loop */
  for(;;)
  {
    BSP_TS_GetState(&TS_State);
    if(TS_State.touchDetected){
      if (deplacement==0){
		    TS_State0=TS_State;
        deplacement=1;
	  	}
      else{
        TS_State1=TS_State;
    	}
    }
    else {
        deplacement=0;
        dx=TS_State1.touchX[0]-TS_State0.touchX[0];
        dy=TS_State1.touchY[0]-TS_State0.touchY[0];
        sprintf(text,"dx %d dy %d",dx,dy);
        BSP_LCD_DisplayStringAtLine(1, (uint8_t*) text);
    }
    osDelay(10);
  }
  /* USER CODE END deplacement_fonction */
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
	for (;;) {
		HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, 1); 
		HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, 0);
		osDelay(500);
	}
  /* USER CODE END Display_fonction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

