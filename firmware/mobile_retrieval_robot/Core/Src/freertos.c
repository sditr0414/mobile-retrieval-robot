/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Variables */
/* Definitions for bluetoothTask */
osThreadId_t bluetoothTaskHandle;
const osThreadAttr_t bluetoothTask_attributes = {
  .name = "bluetoothTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for armTask */
osThreadId_t armTaskHandle;
const osThreadAttr_t armTask_attributes = {
  .name = "armTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for driveTask */
osThreadId_t driveTaskHandle;
const osThreadAttr_t driveTask_attributes = {
  .name = "driveTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for armQueue */
osMessageQueueId_t armQueueHandle;
const osMessageQueueAttr_t armQueue_attributes = {
  .name = "armQueue"
};
/* Definitions for driveQueue */
osMessageQueueId_t driveQueueHandle;
const osMessageQueueAttr_t driveQueue_attributes = {
  .name = "driveQueue"
};
/* Definitions for i2cMutex */
osMutexId_t i2cMutexHandle;
const osMutexAttr_t i2cMutex_attributes = {
  .name = "i2cMutex"
};
/* Definitions for flashMutex */
osMutexId_t flashMutexHandle;
const osMutexAttr_t flashMutex_attributes = {
  .name = "flashMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBluetoothTask(void *argument);
void StartArmTask(void *argument);
void StartDriveTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
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

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of i2cMutex */
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);

  /* creation of flashMutex */
  flashMutexHandle = osMutexNew(&flashMutex_attributes);

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
  /* creation of armQueue */
  armQueueHandle = osMessageQueueNew (8, sizeof(uint16_t), &armQueue_attributes);

  /* creation of driveQueue */
  driveQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &driveQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of bluetoothTask */
  bluetoothTaskHandle = osThreadNew(StartBluetoothTask, NULL, &bluetoothTask_attributes);

  /* creation of armTask */
  armTaskHandle = osThreadNew(StartArmTask, NULL, &armTask_attributes);

  /* creation of driveTask */
  driveTaskHandle = osThreadNew(StartDriveTask, NULL, &driveTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBluetoothTask */
/**
  * @brief  Function implementing the BluetoothTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBluetoothTask */
void StartBluetoothTask(void *argument)
{
  /* USER CODE BEGIN StartBluetoothTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBluetoothTask */
}

/* USER CODE BEGIN Header_StartArmTask */
/**
* @brief Function implementing the ArmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartArmTask */
void StartArmTask(void *argument)
{
  /* USER CODE BEGIN StartArmTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartArmTask */
}

/* USER CODE BEGIN Header_StartDriveTask */
/**
* @brief Function implementing the DriveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDriveTask */
void StartDriveTask(void *argument)
{
  /* USER CODE BEGIN StartDriveTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDriveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

