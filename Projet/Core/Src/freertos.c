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
#include "adc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	int32_t dx;
	int32_t dy;
}
mouvement;

typedef struct
{
	uint16_t x;
	uint16_t y;
}
position;
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
char image[32778];	//32778	130700
position toucher[1000]; //480*2
uint8_t remplissage=0;
int32_t x=0,y=0;

uint8_t *uwInternelBuffer; //Buffer pour la mémoire SDRAM
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId deplacement_tasHandle;
osThreadId DisplayHandle;
osThreadId Joystick_taskHandle;
osMessageQId DeplacementQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void RemplirImage(int32_t x, int32_t y, char* image);
void FabriquerEntete(char* image);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void deplacement_function(void const * argument);
void Display_fonction(void const * argument);
void joystick_function(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of deplacement_tas */
  osThreadDef(deplacement_tas, deplacement_function, osPriorityRealtime, 0, 1024);
  deplacement_tasHandle = osThreadCreate(osThread(deplacement_tas), NULL);

  /* definition and creation of Display */
  osThreadDef(Display, Display_fonction, osPriorityIdle, 0, 4096);
  DisplayHandle = osThreadCreate(osThread(Display), NULL);

  /* definition and creation of Joystick_task */
  osThreadDef(Joystick_task, joystick_function, osPriorityHigh, 0, 1024);
  Joystick_taskHandle = osThreadCreate(osThread(Joystick_task), NULL);

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

/* USER CODE BEGIN Header_deplacement_function */
/**
* @brief Function implementing the deplacement_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_deplacement_function */
void deplacement_function(void const * argument)
{
  /* USER CODE BEGIN deplacement_fonction */
	TS_StateTypeDef TS_State;
	int32_t x1=0,y1=0,x0=0,y0=0;
	uint8_t verrouillage=0,touch=0;
	mouvement deplacement;
	char text[50]={};
	position point;

	/* Infinite loop */
	for(;;)
	{
		BSP_TS_GetState(&TS_State);
		if(TS_State.touchDetected){
			// sprintf(text,"x %d y %d            ",TS_State.touchX[0],TS_State.touchY[0]);
			// BSP_LCD_DisplayStringAtLine(4, (uint8_t*) text);
			if ((TS_State.touchX[0]<50)&& (TS_State.touchY[0]<50)){
				if (verrouillage==1) verrouillage=0;
				else verrouillage=1;
				osDelay(500);
			}
			else if ((TS_State.touchX[0]<50)&& (TS_State.touchY[0]>250)){
				remplissage=0;
				deplacement.dx=0;
				deplacement.dy=0;
				xQueueSend(DeplacementQueueHandle, &deplacement, 0);
				osDelay(300);
			}
			else if (verrouillage!=0){
				BSP_LCD_FillCircle(TS_State.touchX[0], TS_State.touchY[0], 2);
				point.x=TS_State.touchX[0]+x;
				point.y=272+34-TS_State.touchY[0]+y;
				toucher[remplissage]=point;
				remplissage++;
				if (remplissage==1000)remplissage=0;
			}
			else{
				if (touch==0){
					x0=TS_State.touchX[0];
					y0=TS_State.touchY[0];
					touch=1;
				}
				else{
					x1=TS_State.touchX[0];
					y1=TS_State.touchY[0];
				}
			}
		}
		else if(touch!=0) {
			touch=0;
			deplacement.dx=x0-x1;
			deplacement.dy=y1-y0;
			//			sprintf(text,"dx %d dy %d            ",deplacement.dx,deplacement.dy);
			//			BSP_LCD_DisplayStringAtLine(1, (uint8_t*) text);
			//			sprintf(text,"f: x %d y %d            ",x,y);
			//			BSP_LCD_DisplayStringAtLine(2, (uint8_t*) text);
			//			sprintf(text,"d: x %d y %d            ",x0,y0);
			//			BSP_LCD_DisplayStringAtLine(3, (uint8_t*) text);
			xQueueSend(DeplacementQueueHandle, &deplacement, 0);
		}
		//if (verrouillage!=0)BSP_LCD_DisplayStringAtLine(1, (uint8_t*) "verrouillage");
		HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin,verrouillage);
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
	mouvement deplacement;
	uint16_t hauteur=272,index=0;
	//char image[32778];	//32778

	char text[50]={};
	position point;
	FabriquerEntete(image);
	for (int i=0;i<=8;i++){
		RemplirImage(x,y+34*i,image);
		BSP_LCD_DrawBitmap(0, hauteur-34*i, (uint8_t*) image);
	}
	/* Infinite loop */
	for (;;) {
		if (xQueueReceive(DeplacementQueueHandle, &deplacement, 0)) {
			HAL_GPIO_TogglePin(LED13_GPIO_Port, LED13_Pin);
			x+=deplacement.dx;
			y+=deplacement.dy;
			if (x<0) x=0;
			if (x>1227) x=1227;//1707-480
			if (y<0) y=0;
			if (y>394) y=394;//700-272
			for (int i=0;i<=8;i++){
				RemplirImage(x,y+34*i,image);
				BSP_LCD_DrawBitmap(0, hauteur-34*i, (uint8_t*) image);
			}
			sprintf(text,"x=%4ld y=%4ld",x,y);
			BSP_LCD_DisplayStringAtLine(0, (uint8_t*) text);
			for (index=0;index<remplissage;index++){
				point=toucher[index];
				if ((x+5<point.x)&&(point.x<x+480-5) && (y+5<point.y)&&(point.y<y+306-5)){
					BSP_LCD_FillCircle(point.x-x,306-point.y+y,2);
				}
			}
		}
		osDelay(200);
	}
  /* USER CODE END Display_fonction */
}

/* USER CODE BEGIN Header_joystick_function */
/**
* @brief Function implementing the Joystick_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_joystick_function */
void joystick_function(void const * argument)
{
  /* USER CODE BEGIN joystick_function */
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	sConfig.Channel = ADC_CHANNEL_8;
	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	sConfig.Channel = ADC_CHANNEL_0;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	int32_t joystick_h, joystick_v;
	char text[50]={};
	mouvement deplacement;
	/* Infinite loop */
	for(;;)
	{

		HAL_ADC_Start(&hadc3);
		while (HAL_ADC_PollForConversion(&hadc3, 100) != HAL_OK)
			;
		joystick_v = HAL_ADC_GetValue(&hadc3);

		HAL_ADC_Start(&hadc1);
		while (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK)
			;
		joystick_h = HAL_ADC_GetValue(&hadc1);

		deplacement.dx=-(joystick_h-2048)/8;
		deplacement.dy=(joystick_v-2048)/8;
		if ((abs(deplacement.dx)>100)||(abs(deplacement.dy)>100)){
			xQueueSend(DeplacementQueueHandle, &deplacement, 0);
			osDelay(500);
		}

//		sprintf(text, "joy_v : %4ld joy_h : %ld", deplacement.dy, deplacement.dx);
//		BSP_LCD_DisplayStringAtLine(9, (uint8_t*) text);
		osDelay(100);
	}
  /* USER CODE END joystick_function */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void RemplirImage(int32_t x, int32_t y, char* image){
	int16_t largeur=480,hauteur=272,h=700,l=1707,offset=image[10];
	uint32_t k,index=offset,j;
	FIL file;
	char text[50]={};
	//   uint8_t* uwInternelBuffer;
	//	uwInternelBuffer = (uint8_t*) 0xC0260000;
//	unsigned int byteRead;
//	TCHAR pathfile[]="mapENS.bmp";
//	BYTE data[480];
	FATFS fs0;
	UINT br;
	FRESULT val0,val1,val2,val3;

//	if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0) {
		/*##-2- Initialize the Directory Files pointers (heap) ###################*/
		val0 = f_mount(&fs0, "", 0);
		val1 = f_open(&file,"mapENS.bmp",FA_READ);
		for(j=y;j<y+34;j++){//Chaque ligne  		y+hauteur
//			if (j==34)

			k=offset+2*(j*(l+1)+x);
//			sprintf(text,"j: %lu k %lu   %lu         ",j,k,f_size(&file));
//			BSP_LCD_DisplayStringAtLine(6, (uint8_t*) text);

			val2=f_lseek(&file,k);
			val3=f_read(&file,(TCHAR *)image+index,2*largeur,&br);
			// for(i=0;i<2*largeur;i++){
			// 	image[index]=data[i];
			index=index+2*largeur;
			// }
		}
		//			f_read(&file,(TCHAR *)entete,sizeof entete,&br);
		f_close(&file);
//	} else {
//		/* FatFs Initialization Error */
//		Error_Handler();
//	}

	//	f_open(&file,pathfile,FA_READ); // on ne prend que le fichier
	//	for(i=0;i<largeur;i++){//Chaque ligne
	//		k=offset+2*(j*(l+1)+x);
	//		f_lseek(&file,k);
	//		f_read(&file, (TCHAR*) data, 480, &byteRead);
	//		for(j=0;j<hauteur;j++){
	//			image[index]=data[j];
	//			index++;
	//		}
	//	}
	//	f_close(&file);
}

void FabriquerEntete(char* image){
	BYTE entete[138];
	FIL file;
	FATFS fs0;
	UINT br;
//	uint8_t* uwInternelBuffer;

//	uwInternelBuffer = (uint8_t*) 0xC0260000;
	//unsigned int byteRead;
	//TCHAR pathfile[]="mapENS.bmp";


	if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0) {
		/*##-2- Initialize the Directory Files pointers (heap) ###################*/
		f_mount(&fs0, "", 0);
		f_open(&file,"mapENS.bmp",FA_READ);
		f_read(&file,(TCHAR *)entete,sizeof entete,&br);
		f_close(&file);
	} else {
		/* FatFs Initialization Error */
		Error_Handler();
	}

	//f_open(&file,pathfile,FA_READ); // on ne prend que le fichier
	//f_read(&file, (TCHAR*) entete, 138, &byteRead);
	for(int i=0;i<138;i++){
		image[i]=entete[i];
	}
	image[18]=0xE0;
	image[19]=0x01;
	image[20]=0x00;
	image[21]=0x00;

	image[22]=0x22;
	image[23]=0x00;
//	image[22]=0x10;
//	image[23]=0x01;
	image[24]=0x00;
	image[25]=0x00;
	//f_close(&file);
}

/* USER CODE END Application */

