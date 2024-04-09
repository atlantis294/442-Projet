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
typedef struct
{
 	int16_t dx;
  int16_t dy;
}
mouvement;
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
osMessageQId DeplacementQueueHandle;
osMutexId Mutex_AffichageHandle;

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
  /* definition and creation of DeplacementQueue */
  osMessageQDef(DeplacementQueue, 16, mouvement);
  DeplacementQueueHandle = osMessageCreate(osMessageQ(DeplacementQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of deplacement */
  osThreadDef(deplacement, deplacement_fonction, osPriorityRealtime, 0, 1024);
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
  TS_StateTypeDef TS_State;
  int16_t touch,x,y,x0,y0;
  mouvement deplacement;
  char text[50]={};
  /* Infinite loop */
  for(;;)
  {
    BSP_TS_GetState(&TS_State);
    if(TS_State.touchDetected){
      sprintf(text,"x %d y %d            ",TS_State.touchX[0],TS_State.touchY[0]);
      BSP_LCD_DisplayStringAtLine(4, (uint8_t*) text);
      if (touch==0){
		    x0=TS_State.touchX[0];
        y0=TS_State.touchY[0];
        touch=1;
	  	}
      else{
        x=TS_State.touchX[0];
        y=TS_State.touchY[0];
    	}
    }
    else {
        touch=0;
        deplacement.dx=x0-x;
        deplacement.dy=y0-y;
        sprintf(text,"dx %d dy %d            ",deplacement.dx,deplacement.dy);
        BSP_LCD_DisplayStringAtLine(1, (uint8_t*) text);
        sprintf(text,"f: x %d y %d            ",x,y);
        BSP_LCD_DisplayStringAtLine(2, (uint8_t*) text);
        sprintf(text,"d: x %d y %d            ",x0,y0);
        BSP_LCD_DisplayStringAtLine(3, (uint8_t*) text);
        xQueueSend(DeplacementQueueHandle, &deplacement, 0);
    }
    osDelay(50);
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
  mouvement deplacement;
  char image[32778];
  int16_t x=0,y=0;
  FabriquerEntete(image);
	/* Infinite loop */
	for (;;) {
    if (xQueueReceive(DeplacementQueueHandle, &deplacement, 0)) {
      HAL_GPIO_TogglePin(LED13_GPIO_Port, LED13_Pin);
      x+=deplacement.dx;
      y+=deplacement.dy;
      if (x<0) x=0;
      if (x>227) x=1227;//1707-480
      if (y<0) y=0;
      if (y>428) y=428;//700-272
      RemplirImage(x,y,image);
      BSP_LCD_DrawBitmap(0, 0, (uint8_t*) image);
    }
		osDelay(100);
	}
  /* USER CODE END Display_fonction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void RemplirImage(uint16_t x, uint16_t y, char* image){
  int16_t largeur=480,hauteur=272,h=700,l=1707,offset=138;
  uint16_t i,j,k,index=138;
  FIL file;
  uint8_t* uwInternelBuffer;
	uwInternelBuffer = (uint8_t*) 0xC0260000;
  unsigned int byteRead;
	TCHAR pathfile[]="mapENS.bmp";
  char data[480];

  f_open(&file,pathfile,FA_READ); // on ne prend que le fichier
  for(i=0;i<largeur;i++){//Chaque ligne
    k=offset+2*(j*(l+1)+x);
    f_lseek(&file,offset+2*(j*(l+1)+x));
    f_read(&file, (TCHAR*) data, 480, &byteRead);
    for(j=0;j<hauteur;j++){
      image[index]=data[j];
      index++;      
    }
  }
	f_close(&file);
}

void FabriquerEntete(char* image){
  char entete[138];
  FIL file;
  uint8_t* uwInternelBuffer;
  uwInternelBuffer = (uint8_t*) 0xC0260000;
  unsigned int byteRead;
  TCHAR pathfile[]="mapENS.bmp";
  f_open(&file,pathfile,FA_READ); // on ne prend que le fichier
  f_read(&file, (TCHAR*) entete, 138, &byteRead);
  for(int i=0;i<138;i++){
    image[i]=entete[i];
  }
  image[18]=0xE0
  image[19]=0x01
  image[20]=0x00
  image[21]=0x00
  image[22]=0x10
  image[23]=0x01
  image[24]=0x00
  image[25]=0x00
  f_close(&file);
}

/* USER CODE END Application */

