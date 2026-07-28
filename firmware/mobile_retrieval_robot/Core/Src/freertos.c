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
#include "cmsis_os.h"
#include "main.h"
#include "task.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "bluetooth.h"
#include "drive_4wd.h"
#include "i2c.h"
#include "robot_arm.h"
#include "teaching_storage.h"
#include "tim.h"
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ARM_MONITOR_INTERVAL_MS (200U)

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
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for armTask */
osThreadId_t armTaskHandle;
const osThreadAttr_t armTask_attributes = {
    .name = "armTask",
    .stack_size = 384 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for driveTask */
osThreadId_t driveTaskHandle;
const osThreadAttr_t driveTask_attributes = {
    .name = "driveTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for armQueue */
osMessageQueueId_t armQueueHandle;
const osMessageQueueAttr_t armQueue_attributes = {.name = "armQueue"};
/* Definitions for driveQueue */
osMessageQueueId_t driveQueueHandle;
const osMessageQueueAttr_t driveQueue_attributes = {.name = "driveQueue"};
/* Definitions for i2cMutex */
osMutexId_t i2cMutexHandle;
const osMutexAttr_t i2cMutex_attributes = {.name = "i2cMutex"};
/* Definitions for flashMutex */
osMutexId_t flashMutexHandle;
const osMutexAttr_t flashMutex_attributes = {.name = "flashMutex"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void PrintArmStatus(void);

/* USER CODE END FunctionPrototypes */

void StartBluetoothTask(void *argument);
void StartArmTask(void *argument);
void StartDriveTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationMallocFailedHook(void);
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name);

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void) {
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
  Error_Handler();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name) {
  /* 태스크 정보는 디버거에서 확인하고 모든 출력을 안전하게 정지한다. */
  (void)task;
  (void)task_name;
  Error_Handler();
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
  armQueueHandle =
      osMessageQueueNew(8,
                        sizeof(BluetoothArmCommand),
                        &armQueue_attributes);

  /* creation of driveQueue */
  driveQueueHandle =
      osMessageQueueNew(4,
                        sizeof(BluetoothDriveCommand),
                        &driveQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of bluetoothTask */
  bluetoothTaskHandle =
      osThreadNew(StartBluetoothTask, NULL, &bluetoothTask_attributes);

  /* creation of armTask */
  armTaskHandle = osThreadNew(StartArmTask, NULL, &armTask_attributes);

  /* creation of driveTask */
  driveTaskHandle = osThreadNew(StartDriveTask, NULL, &driveTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* RTOS 객체 생성 실패 상태로 장치를 구동하지 않는다. */
  if ((i2cMutexHandle == NULL) ||
      (flashMutexHandle == NULL) ||
      (armQueueHandle == NULL) ||
      (driveQueueHandle == NULL) ||
      (bluetoothTaskHandle == NULL) ||
      (armTaskHandle == NULL) ||
      (driveTaskHandle == NULL)) {
    Error_Handler();
  }
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
void StartBluetoothTask(void *argument) {
  /* USER CODE BEGIN StartBluetoothTask */
  (void)argument;

  if (Bluetooth_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }

  /* DMA에 새로 들어온 바이트만 짧은 주기로 파싱한다. */
  for (;;) {
    Bluetooth_Process(armQueueHandle, driveQueueHandle);
    osDelay(5U);
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
void StartArmTask(void *argument) {
  /* USER CODE BEGIN StartArmTask */
  BluetoothArmCommand command;
  const TeachingSequence *sequence;
  HAL_StatusTypeDef status;
  RobotArmResult arm_result;
  uint8_t angles[3];

  (void)argument;
  TeachingStorage_Init();

  if (RobotArm_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  for (;;) {
    if (osMessageQueueGet(armQueueHandle,
                          &command,
                          NULL,
                          10U) == osOK) {
      if (command.mode == BLUETOOTH_MODE_ARM) {
        if (command.data[6] == BLUETOOTH_ARM_ENABLE_HOME) {
          arm_result = RobotArm_EnableHome(command.data);
        } else if (command.data[6] == BLUETOOTH_ARM_DISABLE) {
          RobotArm_Disable();
          arm_result = ROBOT_ARM_OK;
        } else {
          arm_result = RobotArm_SetPose(command.data);
        }

        /* 활성화/비활성화와 실패한 이동은 앱에서 상태를 알 수 있게 응답한다. */
        if ((command.data[6] != BLUETOOTH_ARM_MOVE) ||
            (arm_result != ROBOT_ARM_OK)) {
          (void)Bluetooth_SendArmAck(
              command.data[6],
              (uint8_t)(arm_result == ROBOT_ARM_OK),
              (uint8_t)arm_result);
        }
      } else if (command.mode == BLUETOOTH_MODE_TEACHING) {
        status = HAL_ERROR;

        if (command.data[0] == 2U) {
          sequence = TeachingStorage_Get(command.data[1]);
          status = (RobotArm_Play(sequence) == ROBOT_ARM_OK)
                       ? HAL_OK
                       : HAL_ERROR;
        } else if (command.data[0] == 3U) {
          RobotArm_Stop();
          Drive4WD_SetStorageInhibit(1U);
          if (osMutexAcquire(flashMutexHandle, osWaitForever) == osOK) {
            status = TeachingStorage_Reset(command.data[1]);
            (void)osMutexRelease(flashMutexHandle);
          }
          Drive4WD_SetStorageInhibit(0U);
        } else if ((command.data[0] == 4U) &&
                   (command.data[1] == 1U)) {
          /* 재생 중인 RAM 시퀀스를 업로드가 바꾸지 않게 먼저 중지한다. */
          RobotArm_Stop();
          status = TeachingStorage_BeginUpload(command.data[2],
                                                command.data[3]);
        } else if ((command.data[0] == 4U) &&
                   ((command.data[1] == 2U) ||
                    (command.data[1] == 3U))) {
          angles[0] = command.data[4];
          angles[1] = command.data[5];
          angles[2] = command.data[6];
          status = TeachingStorage_WriteHalf(
              command.data[2],
              command.data[3],
              (uint8_t)(command.data[1] == 3U),
              angles);
        } else if ((command.data[0] == 4U) &&
                   (command.data[1] == 4U)) {
          RobotArm_Stop();
          Drive4WD_SetStorageInhibit(1U);
          if (osMutexAcquire(flashMutexHandle, osWaitForever) == osOK) {
            status = TeachingStorage_Commit(command.data[2]);
            (void)osMutexRelease(flashMutexHandle);
          }
          Drive4WD_SetStorageInhibit(0U);
          (void)Bluetooth_SendTeachingAck(
              command.data[2],
              (uint8_t)(status == HAL_OK));
        }
      }
    }

    RobotArm_Update();
    PrintArmStatus();
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
void StartDriveTask(void *argument) {
  /* USER CODE BEGIN StartDriveTask */
  BluetoothDriveCommand command;

  (void)argument;
  if (Drive4WD_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  for (;;) {
    if (osMessageQueueGet(driveQueueHandle,
                          &command,
                          NULL,
                          20U) == osOK) {
      Drive4WD_Apply(&command);
    }
    Drive4WD_CheckTimeout();
  }
  /* USER CODE END StartDriveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void PrintArmStatus(void) {
  static int16_t previous_values[ROBOT_ARM_JOINT_COUNT];
  static uint8_t previous_enabled = 2U;
  static uint32_t previous_tick;
  int16_t values[ROBOT_ARM_JOINT_COUNT];
  uint16_t pulses[ROBOT_ARM_JOINT_COUNT];
  uint8_t enabled;
  uint32_t now;
  int length;
  char message[192];

  enabled = RobotArm_GetStatus(values, pulses);
  now = HAL_GetTick();

  if ((enabled == previous_enabled) &&
      (memcmp(values, previous_values, sizeof(values)) == 0)) {
    return;
  }

  /* 빠른 관절 이동 중에는 UART 출력 횟수를 제한한다. */
  if ((previous_enabled != 2U) &&
      ((now - previous_tick) < ARM_MONITOR_INTERVAL_MS)) {
    return;
  }

  if (enabled == 0U) {
    length = snprintf(message, sizeof(message), "ARM OFF\r\n");
  } else {
    length = snprintf(
        message,
        sizeof(message),
        "ARM OUT B=%+d/%uus S=%+d/%uus E=%+d/%uus "
        "WT=%+d/%uus WR=%+d/%uus G=%d%%/%uus\r\n",
        values[0], pulses[0],
        values[1], pulses[1],
        values[2], pulses[2],
        values[3], pulses[3],
        values[4], pulses[4],
        values[5], pulses[5]);
  }

  if (length > 0) {
    uint16_t transmit_length = (length < (int)sizeof(message))
                                   ? (uint16_t)length
                                   : (uint16_t)(sizeof(message) - 1U);
    (void)HAL_UART_Transmit(&huart2,
                            (uint8_t *)message,
                            transmit_length,
                            20U);
  }

  memcpy(previous_values, values, sizeof(previous_values));
  previous_enabled = enabled;
  previous_tick = now;
}

/* USER CODE END Application */
