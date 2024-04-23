/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "fatfs.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "main.h"
#include "fatfs_storage.h"


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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t caractere_recu;
int State_demarrage = 0;
uint8_t workBuffer[2 * _MAX_SS];
char *pDirectoryFiles[MAX_BMP_FILES];
uint8_t ubNumberOfFiles = 0;
uint32_t uwBmplen = 0;

uint8_t *uwInternelBuffer; //Buffer pour la mémoire SDRAM
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId task_Affich_PicHandle;
osThreadId tackdeplacementHandle;
osMessageQId DeplacementQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Affichage_Pic(void const * argument);
void deplacement_fonction(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

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

  /* Create the queue(s) */
  /* definition and creation of DeplacementQueue */
  osMessageQDef(DeplacementQueue, 16, mouvement);
  DeplacementQueueHandle = osMessageCreate(osMessageQ(DeplacementQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of task_Affich_Pic */
  osThreadDef(task_Affich_Pic, Affichage_Pic, osPriorityNormal, 0, 2048);
  task_Affich_PicHandle = osThreadCreate(osThread(task_Affich_Pic), NULL);

  /* definition and creation of tackdeplacement */
  osThreadDef(tackdeplacement, deplacement_fonction, osPriorityIdle, 0, 128);
  tackdeplacementHandle = osThreadCreate(osThread(tackdeplacement), NULL);

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
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Affichage_Pic */
/**
* @brief Function implementing the task_Affich_Pic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Affichage_Pic */
void Affichage_Pic(void const * argument)
{
  /* USER CODE BEGIN Affichage_Pic */
		FATFS fs0;
		FIL file;
		BYTE buffer[99];
		UINT br;

//		uint8_t str[30];
		uwInternelBuffer = (uint8_t*) 0xC0260000;
//		uint16_t MessageFromIHM;

		/*##-1- Link the SD Card disk I/O driver ###################################*/
		if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0) {
			/*##-2- Initialize the Directory Files pointers (heap) ###################*/
			f_mount(&fs0, "", 0);
			f_open(&file,"mapENS.bmp",FA_READ);
			f_read(&file,(TCHAR *)buffer,sizeof buffer,&br);
			f_close(&file);
		} else {
			/* FatFs Initialization Error */
			Error_Handler();
		}
  /* USER CODE END Affichage_Pic */
}

/* USER CODE BEGIN Header_deplacement_fonction */
/**
* @brief Function implementing the tackdeplacement thread.
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

 void RemplirImage(uint16_t x, uint16_t y, char* image){
   int16_t largeur=480,hauteur=272,h=700,l=1707,offset=138;
   uint16_t i,j,k,index=138;
   FIL file;
//   uint8_t* uwInternelBuffer;
 	uwInternelBuffer = (uint8_t*) 0xC0260000;
   unsigned int byteRead;
 	TCHAR pathfile[]="mapENS.bmp";
   char data[480];

   f_open(&file,pathfile,FA_READ); // on ne prend que le fichier
   for(i=0;i<largeur;i++){//Chaque ligne
     k=offset+2*(j*(l+1)+x);
     f_lseek(&file,k);
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
//   uint8_t* uwInternelBuffer;
   uwInternelBuffer = (uint8_t*) 0xC0260000;
   unsigned int byteRead;
   TCHAR pathfile[]="mapENS.bmp";
   f_open(&file,pathfile,FA_READ); // on ne prend que le fichier
   f_read(&file, (TCHAR*) entete, 138, &byteRead);
   for(int i=0;i<138;i++){
     image[i]=entete[i];
   }
   image[18]=0xE0;
   image[19]=0x01;
   image[20]=0x00;
   image[21]=0x00;
   image[22]=0x10;
   image[23]=0x01;
   image[24]=0x00;
   image[25]=0x00;
   f_close(&file);
 }

/* USER CODE END Application */

