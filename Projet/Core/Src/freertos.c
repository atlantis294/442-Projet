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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int State_demarrage = 0;
uint8_t workBuffer[2 * _MAX_SS];
char *pDirectoryFiles[MAX_BMP_FILES];
uint8_t ubNumberOfFiles = 0;
uint32_t uwBmplen = 0;

uint8_t *uwInternelBuffer; //Buffer pour la mémoire SDRAM
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId task_IHMHandle;
osThreadId task_Affich_PicHandle;
osThreadId task_DemarrageHandle;
osThreadId myTask05Handle;
osMessageQId QueueTS2PICHandle;
osMessageQId QueueActivationHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Interface_HM(void const * argument);
void Affichage_Pic(void const * argument);
void Demarrage(void const * argument);
void StartTask05(void const * argument);

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
  /* definition and creation of QueueTS2PIC */
  osMessageQDef(QueueTS2PIC, 2, uint16_t);
  QueueTS2PICHandle = osMessageCreate(osMessageQ(QueueTS2PIC), NULL);

  /* definition and creation of QueueActivation */
  osMessageQDef(QueueActivation, 2, uint16_t);
  QueueActivationHandle = osMessageCreate(osMessageQ(QueueActivation), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of task_IHM */
  osThreadDef(task_IHM, Interface_HM, osPriorityAboveNormal, 0, 512);
  task_IHMHandle = osThreadCreate(osThread(task_IHM), NULL);

  /* definition and creation of task_Affich_Pic */
  osThreadDef(task_Affich_Pic, Affichage_Pic, osPriorityNormal, 0, 2048);
  task_Affich_PicHandle = osThreadCreate(osThread(task_Affich_Pic), NULL);

  /* definition and creation of task_Demarrage */
  osThreadDef(task_Demarrage, Demarrage, osPriorityLow, 0, 128);
  task_DemarrageHandle = osThreadCreate(osThread(task_Demarrage), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, StartTask05, osPriorityNormal, 0, 1024);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

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

/* USER CODE BEGIN Header_Interface_HM */
/**
* @brief Function implementing the task_IHM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Interface_HM */
void Interface_HM(void const * argument)
{
	 /* USER CODE BEGIN Interface_HM */
		TS_StateTypeDef TS_State;
		uint16_t MessageTS;

		/* Infinite loop */
		for (;;) {
			osDelay(300); //Période de 200ms
			BSP_TS_GetState(&TS_State);
			if (TS_State.touchDetected) {
				HAL_GPIO_TogglePin(LED12_GPIO_Port, LED12_Pin);
				MessageTS = TS_State.touchX[0];
				xQueueSend(QueueTS2PICHandle, &MessageTS, 0);
			}
		}
	  /* USER CODE END Interface_HM */
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
		FIL music_file;
		BYTE buffer[99];
		FRESULT open_good, read_good,close_good;
		UINT br;


		uint32_t counter = 0, transparency = 0;
		uint8_t str[30];
		uwInternelBuffer = (uint8_t*) 0xC0260000;
		uint16_t MessageFromIHM;





		const TickType_t Delai_Atttente = 5000; //5s d'attente maximun
		/*##-1- Link the SD Card disk I/O driver ###################################*/
		if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0) {
			/*##-2- Initialize the Directory Files pointers (heap) ###################*/
			for (counter = 0; counter < MAX_BMP_FILES; counter++) {
				pDirectoryFiles[counter] = malloc(MAX_BMP_FILE_NAME);
				if (pDirectoryFiles[counter] == NULL) {
					/* Set the Text Color */
					BSP_LCD_SetTextColor(LCD_COLOR_RED);

					BSP_LCD_DisplayStringAtLine(8,
							(uint8_t*) "  Cannot allocate memory ");

					while (1) {
					}
				}
			}
			f_mount(&fs0, "", 0);
			open_good = f_open(&music_file,"sha.wav",FA_READ);
			read_good = f_read(&music_file,(TCHAR *)buffer,sizeof buffer,&br);
			close_good = f_close(&music_file);

			/* Get the BMP file names on root directory */
			ubNumberOfFiles = Storage_GetDirectoryBitmapFiles("/Media",
					pDirectoryFiles);

			if (ubNumberOfFiles == 0) {
				for (counter = 0; counter < MAX_BMP_FILES; counter++) {
					free(pDirectoryFiles[counter]);
				}
				BSP_LCD_DisplayStringAtLine(4,
						(uint8_t*) "  Pas de fichiers Bitmap ou pas de carte SD inseree... Redemarrer      ");
				while (1) {
				}
			}
		} else {
			/* FatFs Initialization Error */
			Error_Handler();
		}

		/* Infinite loop */
		for (;;) {

			//Période de 200ms
			osDelay(200);

			counter = 0;
			while ((counter) < ubNumberOfFiles) {
				/* Step1 : Display on Foreground layer -------------------------------*/
				/* Format the string */
				sprintf((char*) str, "Media/%-11.11s", pDirectoryFiles[counter]);

				if (Storage_CheckBitmapFile((const char*) str, &uwBmplen) == 0) {
					/* Format the string */
					sprintf((char*) str, "Media/%-11.11s",
							pDirectoryFiles[counter]);

					/* Set LCD foreground Layer */
					BSP_LCD_SelectLayer(1);

					/* Open a file and copy its content to an internal buffer */
					Storage_OpenReadFile(uwInternelBuffer, (const char*) str);

					/* Write bmp file on LCD frame buffer */
					BSP_LCD_DrawBitmap(0, 0, uwInternelBuffer);
					BSP_LCD_DisplayStringAtLine(3, str);


					/* Configure the transparency for background layer : Increase the transparency */
					for (transparency = 0; transparency < 255; (transparency++)) {
						BSP_LCD_SetTransparency(1, transparency);

						/* Insert a delay of display */
						HAL_Delay(2);
					}

					/* Attente d'un appui sur l'écran tactile */
	//				while (HAL_GPIO_ReadPin(BP1_GPIO_Port, BP1_Pin) == 1) {
	//				}
					xQueueReceive(QueueTS2PICHandle, &MessageFromIHM,
							Delai_Atttente);
					if (MessageFromIHM < 240) {
						if (counter == 0) {
							counter = ubNumberOfFiles;
						}
						counter -= 2; //Le counter++ ce fait en aval au cas où il n'y a pas d'appui sur l'écran
					} else if (MessageFromIHM >= 240) {
						counter += 0;
					}

					/* Configure the transparency for foreground layer : decrease the transparency */
					for (transparency = 255; transparency > 0; transparency--) {
						BSP_LCD_SetTransparency(1, transparency);

						/* Insert a delay of display */
						HAL_Delay(2);
					}

					/* Clear the Foreground Layer */
					BSP_LCD_Clear(LCD_COLOR_WHITE);

					/* Jump to the next image */
					counter++;
					/* Step2 : Display on Background layer -----------------------------*/
					/* Format the string */
					sprintf((char*) str, "Media/%-11.11s",
							pDirectoryFiles[counter]);

					if ((Storage_CheckBitmapFile((const char*) str, &uwBmplen) == 0)
							|| (counter < (ubNumberOfFiles))) {
						/* Connect the Output Buffer to LCD Background Layer  */
						BSP_LCD_SelectLayer(0);

						/* Format the string */
						sprintf((char*) str, "Media/%-11.11s",
								pDirectoryFiles[counter]);

						/* Open a file and copy its content to an internal buffer */
						Storage_OpenReadFile(uwInternelBuffer, (const char*) str);

						/* Write bmp file on LCD frame buffer */
						BSP_LCD_DrawBitmap(0, 0, uwInternelBuffer);
						BSP_LCD_DisplayStringAtLine(3, str);


						/* Configure the transparency for background layer : decrease the transparency */
						for (transparency = 0; transparency < 255;
								(transparency++)) {
							BSP_LCD_SetTransparency(0, transparency);

							/* Insert a delay of display */
							HAL_Delay(2);
						}

						/* Attente d'un appui sur l'écran tactile */
	//					while (HAL_GPIO_ReadPin(BP1_GPIO_Port, BP1_Pin) == 1) {
	//					}
						xQueueReceive(QueueTS2PICHandle, &MessageFromIHM,
								Delai_Atttente);
						if (MessageFromIHM < 240) {
							if (counter == 0) {
								counter = ubNumberOfFiles;
							} //On définit un cycle
							counter -= 2; //Le counter++ ce fait en aval au cas où il n'y a pas d'appui sur l'écran
						} else if (MessageFromIHM >= 240) {
							counter += 0;
						}

						/* Step3 : -------------------------------------------------------*/
						/* Configure the transparency for background layer : Increase the transparency */
						for (transparency = 255; transparency > 0; transparency--) {
							BSP_LCD_SetTransparency(0, transparency);

							/* Insert a delay of display */
							HAL_Delay(2);
						}

						/* Clear the Background Layer */
						BSP_LCD_Clear(LCD_COLOR_WHITE);

						/* Next image */
						counter++;
					} else if (Storage_CheckBitmapFile((const char*) str, &uwBmplen)
							== 0) {
						/* Set the Text Color */
						BSP_LCD_SetTextColor(LCD_COLOR_LIGHTRED);

						BSP_LCD_DisplayStringAtLine(7, (uint8_t*) str);
						BSP_LCD_DisplayStringAtLine(8,
								(uint8_t*) "    File type not supported. ");
						while (1) {
						}
					}
				}
			}
		}
	/* USER CODE END Affichage_Pic */
}

/* USER CODE BEGIN Header_Demarrage */
/**
* @brief Function implementing the task_Demarrage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Demarrage */
void Demarrage(void const * argument)
{
	 /* USER CODE BEGIN Demarrage */
	//	int etat_demarrage=0;

		FIL music_file;
		BYTE buffer[4096];
		FRESULT fr;

		FATFS fs0;

		UINT br;
		if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0) {
			f_mount(&fs0,"",0);
			f_open(&music_file,"/son/sha.wav",FA_READ);
			f_read(&music_file,buffer,sizeof buffer,&br);
			f_close(&music_file);
		}

		/* Infinite loop */
		for (;;) {

			osDelay(100);
		}
	  /* USER CODE END Demarrage */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

