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
#include <string.h>

#include "bluetooth.h"
#include "drive_4wd.h"
#include "i2c.h"
#include "imu.h"
#include "robot_arm.h"
#include "servo_driver.h"
#include "teaching_storage.h"
#include "tim.h"
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IMU_RETRY_INTERVAL_MS  (1000U)
#define ARM_DRIVER_RETRY_INTERVAL_MS (1000U)
#define SETTINGS_SERIALIZED_SIZE (53U)
#define SETTINGS_WRITE_PART_SIZE (10U)
#define SETTINGS_READ_PART_SIZE  (8U)
#define SETTINGS_WRITE_PART_COUNT (6U)
#define SETTINGS_READ_PART_COUNT  (7U)
#define SETTINGS_TRAVEL_POSE_OFFSET (48U)
#define PID_KP_MAX_MILLI          (2550L)
#define PID_GAIN_MAX_MILLI        (100000L)
#define PID_KP_STEP_MILLI         (10L)
#define STATUS_INTERVAL_MS        (250U)
#define TRAVEL_SEQUENCE_ID        (9U)
#define TEACHING_NAME_UPLOAD_PART_SIZE (8U)
#define TEACHING_NAME_READ_PART_SIZE   (7U)

#define SETTINGS_RESULT_OK         (0U)
#define SETTINGS_RESULT_INCOMPLETE (1U)
#define SETTINGS_RESULT_CRC        (2U)
#define SETTINGS_RESULT_INVALID    (3U)
#define SETTINGS_RESULT_ARM_ACTIVE (4U)
#define SETTINGS_RESULT_FLASH      (5U)
#define SETTINGS_RESULT_PID_ACTIVE (6U)
#define SETTINGS_RESULT_PID_INVALID (7U)

#define PID_TOGGLE_RESULT_OK          (0U)
#define PID_TOGGLE_RESULT_UNAVAILABLE (1U)
#define PID_TOGGLE_RESULT_INVALID_GAINS (2U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static osThreadId_t imuTaskHandle;
static const osThreadAttr_t imuTask_attributes = {
    .name = "imuTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

static osThreadId_t statusTaskHandle;
static const osThreadAttr_t statusTask_attributes = {
    .name = "statusTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

static uint8_t settings_upload[SETTINGS_SERIALIZED_SIZE];
static uint8_t settings_upload_parts;

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

static void StartImuTask(void *argument);
static void StartStatusTask(void *argument);
static void HandleSettingsCommand(const BluetoothArmCommand *command);
static void SerializeSettings(const RobotSettings *settings, uint8_t *data);
static void DeserializeSettings(const uint8_t *data, RobotSettings *settings);
static uint16_t CalculateSettingsCrc(const uint8_t *data, uint8_t length);

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
  ServoDriver_SetI2CMutex(i2cMutexHandle);

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
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);
  statusTaskHandle = osThreadNew(StartStatusTask, NULL, &statusTask_attributes);

  /* RTOS 객체 생성 실패 상태로 장치를 구동하지 않는다. */
  if ((i2cMutexHandle == NULL) ||
      (flashMutexHandle == NULL) ||
      (armQueueHandle == NULL) ||
      (driveQueueHandle == NULL) ||
      (bluetoothTaskHandle == NULL) ||
      (armTaskHandle == NULL) ||
      (driveTaskHandle == NULL) ||
      (imuTaskHandle == NULL) ||
      (statusTaskHandle == NULL)) {
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
  RobotSettings settings;
  uint32_t arm_retry_tick;
  uint8_t angles[3];
  uint8_t name_part[TEACHING_NAME_UPLOAD_PART_SIZE];
  uint8_t name_response[TEACHING_NAME_READ_PART_SIZE];
  uint8_t name_offset;
  uint8_t name_part_index;
  uint8_t name_part_length;
  uint8_t waypoint_index;
  uint8_t arm_driver_ready;
  uint8_t active_arm_transaction;
  uint8_t active_arm_command;
  uint8_t teaching_play_sequence;
  uint8_t preview_joint;
  uint8_t preview_request_id;
  uint16_t preview_pulse_us;
  RobotArmResult preview_result;

  (void)argument;
  TeachingStorage_Init();

  /*
   * PCA9685 전원이 MCU보다 늦게 들어와도 Bluetooth와 설정 조회는 유지한다.
   * 초기화에 실패한 동안에는 로봇팔 명령에 드라이버 오류를 응답하고 재시도한다.
   */
  arm_driver_ready =
      (uint8_t)(RobotArm_Init(&hi2c1) == HAL_OK);
  arm_retry_tick = HAL_GetTick();
  active_arm_transaction = 0U;
  active_arm_command = BLUETOOTH_ARM_MOVE;
  teaching_play_sequence = 0U;
  TeachingStorage_GetSettings(&settings);
  (void)Drive4WD_SetPidGains(settings.pid_kp_milli,
                             settings.pid_ki_milli,
                             settings.pid_kd_milli);
  if (RobotArm_SetCalibrations(settings.servo_pulse_us) != ROBOT_ARM_OK) {
    /* 손상된 보정값은 안전한 펌웨어 기본값으로 바꿔 앱 조회값과 맞춘다. */
    TeachingStorage_ResetServoCalibrations();
    TeachingStorage_GetSettings(&settings);
    if (RobotArm_SetCalibrations(settings.servo_pulse_us) != ROBOT_ARM_OK) {
      Error_Handler();
    }
  }

  for (;;) {
    if ((arm_driver_ready == 0U) &&
        ((HAL_GetTick() - arm_retry_tick) >=
         ARM_DRIVER_RETRY_INTERVAL_MS)) {
      arm_retry_tick = HAL_GetTick();
      if (RobotArm_Init(&hi2c1) == HAL_OK) {
        TeachingStorage_GetSettings(&settings);
        if (RobotArm_SetCalibrations(settings.servo_pulse_us) ==
            ROBOT_ARM_OK) {
          arm_driver_ready = 1U;
        }
      }
    }

    if (osMessageQueueGet(armQueueHandle,
                          &command,
                          NULL,
                          5U) == osOK) {
      if (command.mode == BLUETOOTH_MODE_ARM) {
        if (teaching_play_sequence != 0U) {
          (void)Bluetooth_SendTeachingAck(
              2U, teaching_play_sequence, 0U, 0U);
          (void)RobotArm_TakeSpeedWarning();
          teaching_play_sequence = 0U;
        }
        if (command.data[6] == BLUETOOTH_ARM_DISABLE) {
          arm_result = RobotArm_Disable();
          Drive4WD_SetArmMotionInhibit(0U);
        } else {
          /* 관절을 움직이기 전에 차량을 멈추고 주행 명령을 차단한다. */
          Drive4WD_SetArmMotionInhibit(1U);

          if (arm_driver_ready == 0U) {
            arm_result = ROBOT_ARM_DRIVER_ERROR;
          } else if (RobotArm_SetSpeedPercent(command.data[8]) !=
                     ROBOT_ARM_OK) {
            arm_result = ROBOT_ARM_INVALID;
          } else if (command.data[6] == BLUETOOTH_ARM_ENABLE_ORIGIN) {
            arm_result = RobotArm_EnableOrigin(command.data);
          } else if (command.data[6] ==
                     BLUETOOTH_ARM_TRAVEL_KEEP_GRIPPER) {
            sequence = TeachingStorage_Get(TRAVEL_SEQUENCE_ID);
            arm_result = RobotArm_Play(sequence, 1U);
          } else {
            arm_result = RobotArm_SetPose(command.data);
          }

          if (arm_result != ROBOT_ARM_OK) {
            Drive4WD_SetArmMotionInhibit(0U);
          }
        }

        if ((arm_result == ROBOT_ARM_OK) &&
            (command.data[6] != BLUETOOTH_ARM_DISABLE)) {
          active_arm_transaction = command.data[7];
          active_arm_command =
              (command.data[6] == BLUETOOTH_ARM_TRAVEL_KEEP_GRIPPER)
                  ? BLUETOOTH_ARM_TRAVEL_KEEP_GRIPPER
                  : BLUETOOTH_ARM_MOVE;
        } else if (command.data[6] == BLUETOOTH_ARM_DISABLE) {
          active_arm_transaction = 0U;
          active_arm_command = BLUETOOTH_ARM_MOVE;
        }

        /* 활성화/비활성화와 실패한 이동은 앱에서 상태를 알 수 있게 응답한다. */
        if (((command.data[6] != BLUETOOTH_ARM_MOVE) &&
             (command.data[6] != BLUETOOTH_ARM_TRAVEL_KEEP_GRIPPER)) ||
            (arm_result != ROBOT_ARM_OK)) {
          (void)Bluetooth_SendArmAck(
              command.data[6],
              (uint8_t)(arm_result == ROBOT_ARM_OK),
              (uint8_t)arm_result,
              command.data[7]);
        }
      } else if (command.mode == BLUETOOTH_MODE_TEACHING) {
        status = HAL_ERROR;

        if ((command.data[0] == 2U) ||
            (command.data[0] == BLUETOOTH_TEACHING_PREVIEW)) {
          Drive4WD_SetArmMotionInhibit(1U);
          sequence = (command.data[0] == BLUETOOTH_TEACHING_PREVIEW)
                         ? TeachingStorage_GetUpload(command.data[1])
                         : TeachingStorage_Get(command.data[1]);
          status = ((RobotArm_SetSpeedPercent(command.data[2]) ==
                     ROBOT_ARM_OK) &&
                    (RobotArm_Play(
                         sequence,
                         (uint8_t)(command.data[1] == TRAVEL_SEQUENCE_ID)) ==
                     ROBOT_ARM_OK))
                       ? HAL_OK
                       : HAL_ERROR;
          if (status == HAL_OK) {
            teaching_play_sequence = command.data[1];
          } else {
            Drive4WD_SetArmMotionInhibit(0U);
            (void)Bluetooth_SendTeachingAck(
                2U, command.data[1], 0U, 0U);
          }
        } else if (command.data[0] == 3U) {
          RobotArm_Stop();
          (void)RobotArm_TakeSpeedWarning();
          Drive4WD_SetStorageInhibit(1U);
          if (osMutexAcquire(flashMutexHandle, osWaitForever) == osOK) {
            status = TeachingStorage_Reset(command.data[1]);
            (void)osMutexRelease(flashMutexHandle);
          }
          Drive4WD_SetStorageInhibit(0U);
          (void)Bluetooth_SendTeachingAck(
              3U,
              command.data[1],
              (uint8_t)(status == HAL_OK),
              0U);
        } else if ((command.data[0] == 4U) &&
                   (command.data[1] == 1U)) {
          /* 재생 중인 RAM 시퀀스를 업로드가 바꾸지 않게 먼저 중지한다. */
          RobotArm_Stop();
          (void)RobotArm_TakeSpeedWarning();
          status = TeachingStorage_BeginUpload(command.data[2],
                                                command.data[3],
                                                command.data[4]);
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
                   (command.data[1] == 5U)) {
          memcpy(name_part,
                 &command.data[4],
                 TEACHING_NAME_UPLOAD_PART_SIZE);
          status = TeachingStorage_WriteNamePart(command.data[2],
                                                 command.data[3],
                                                 name_part);
        } else if ((command.data[0] == 4U) &&
                   (command.data[1] == 4U)) {
          RobotArm_Stop();
          (void)RobotArm_TakeSpeedWarning();
          Drive4WD_SetStorageInhibit(1U);
          if (osMutexAcquire(flashMutexHandle, osWaitForever) == osOK) {
            status = TeachingStorage_Commit(command.data[2]);
            (void)osMutexRelease(flashMutexHandle);
          }
          Drive4WD_SetStorageInhibit(0U);
          (void)Bluetooth_SendTeachingAck(
              4U,
              command.data[2],
              (uint8_t)(status == HAL_OK),
              0U);
        } else if (command.data[0] == BLUETOOTH_TEACHING_GET_NAME) {
          sequence = TeachingStorage_Get(command.data[1]);
          if (sequence != NULL) {
            for (name_part_index = 0U;
                 (uint8_t)(name_part_index * TEACHING_NAME_READ_PART_SIZE) <
                     sequence->name_length;
                 name_part_index++) {
              name_offset = (uint8_t)(name_part_index *
                                      TEACHING_NAME_READ_PART_SIZE);
              name_part_length =
                  (uint8_t)(sequence->name_length - name_offset);
              if (name_part_length > TEACHING_NAME_READ_PART_SIZE) {
                name_part_length = TEACHING_NAME_READ_PART_SIZE;
              }
              memset(name_response, 0, sizeof(name_response));
              memcpy(name_response,
                     &sequence->name[name_offset],
                     name_part_length);
              (void)Bluetooth_SendTeachingNamePart(
                  command.data[1],
                  name_part_index,
                  sequence->name_length,
                  command.data[2],
                   name_response);
            }
          }
        } else if (command.data[0] ==
                   BLUETOOTH_TEACHING_GET_SEQUENCE) {
          sequence = TeachingStorage_Get(command.data[1]);
          if (sequence != NULL) {
            (void)Bluetooth_SendTeachingSequenceMeta(command.data[1],
                                                     sequence->count,
                                                     command.data[2]);
            for (waypoint_index = 0U;
                 waypoint_index < sequence->count;
                 waypoint_index++) {
              (void)Bluetooth_SendTeachingWaypoint(
                  command.data[1],
                  waypoint_index,
                  sequence->waypoints[waypoint_index].angles,
                  command.data[2]);
            }
          }
        }
      } else if (command.mode == BLUETOOTH_MODE_SETTINGS) {
        HandleSettingsCommand(&command);
      }
    }

    if (arm_driver_ready != 0U) {
      RobotArm_Update();
      if (RobotArm_TakePreviewResult(&preview_joint,
                                     &preview_pulse_us,
                                     &preview_request_id,
                                     &preview_result) != 0U) {
        if (preview_result != ROBOT_ARM_OK) {
          Drive4WD_SetArmMotionInhibit(0U);
        }
        (void)Bluetooth_SendPreviewResult(
            preview_joint,
            (uint8_t)(preview_result == ROBOT_ARM_OK),
            (uint8_t)preview_result,
            preview_pulse_us,
            preview_request_id);
      }
      if (RobotArm_TakePoseReached() != 0U) {
        if (active_arm_transaction != 0U) {
          uint8_t arm_ack_reason =
              (RobotArm_TakeSpeedWarning() != 0U)
                  ? ROBOT_ARM_ACK_SPEED_WARNING
                  : ROBOT_ARM_OK;
          (void)Bluetooth_SendArmAck(active_arm_command,
                                     1U,
                                     arm_ack_reason,
                                     active_arm_transaction);
          active_arm_transaction = 0U;
          active_arm_command = BLUETOOTH_ARM_MOVE;
        }
      }
      if ((teaching_play_sequence != 0U) &&
          (RobotArm_IsMotionActive() == 0U)) {
        (void)Bluetooth_SendTeachingAck(
            2U,
            teaching_play_sequence,
            1U,
            RobotArm_TakeSpeedWarning());
        teaching_play_sequence = 0U;
      }
      if (RobotArm_IsMotionActive() == 0U) {
        Drive4WD_SetArmMotionInhibit(0U);
      }
    }
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
  IMU_Data imu_data;
  uint32_t processed_imu_tick = 0U;

  (void)argument;
  if (Drive4WD_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  for (;;) {
    if (osMessageQueueGet(driveQueueHandle,
                          &command,
                          NULL,
                          20U) == osOK) {
      Drive4WD_Apply(&command);
      if (command.control != BLUETOOTH_DRIVE_NORMAL) {
        (void)Bluetooth_SendDriveAck(command.control, 1U, 0U);
      }
    }
    if ((IMU_GetLatest(&imu_data) != 0U) &&
        (imu_data.valid != 0U) &&
        (imu_data.calibrated != 0U) &&
        (imu_data.sample_tick_ms != processed_imu_tick)) {
      processed_imu_tick = imu_data.sample_tick_ms;
      Drive4WD_ProcessYaw(imu_data.yaw_deg, imu_data.sample_tick_ms);
    }
    Drive4WD_CheckTimeout();
  }
  /* USER CODE END StartDriveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* MPU6050 연결을 재시도하고 성공하면 영점 보정 후 20 ms 주기로 읽는다. */
static void StartImuTask(void *argument) {
  uint8_t consecutive_errors;
  uint32_t next_tick;

  (void)argument;

  /*
   * 센서가 연결되지 않아도 기존 로봇 제어는 계속 실행한다.
   * 연결이 확인되면 차량을 정지하고 자이로 영점을 한 번 보정한다.
   */
  for (;;) {
    Drive4WD_SetImuAvailable(0U);
    if (IMU_Init(&hi2c1, i2cMutexHandle) != HAL_OK) {
      osDelay(IMU_RETRY_INTERVAL_MS);
      continue;
    }

    Drive4WD_SetImuCalibrationInhibit(1U);
    if (IMU_CalibrateGyro() != HAL_OK) {
      Drive4WD_SetImuCalibrationInhibit(0U);
      osDelay(IMU_RETRY_INTERVAL_MS);
      continue;
    }
    Drive4WD_SetImuCalibrationInhibit(0U);
    Drive4WD_SetImuAvailable(1U);

    consecutive_errors = 0U;
    next_tick = osKernelGetTickCount();
    while (consecutive_errors < 3U) {
      if (IMU_Read() == HAL_OK) {
        consecutive_errors = 0U;
      } else {
        consecutive_errors++;
      }
      next_tick += IMU_SAMPLE_INTERVAL_MS;
      (void)osDelayUntil(next_tick);
    }
    Drive4WD_SetImuAvailable(0U);
  }
}

/* 상태 패킷의 float 값을 signed 16비트 고정소수점 범위로 제한한다. */
static int16_t ScaleStatusValue(float value, float scale) {
  float scaled = value * scale;

  if (scaled > 32767.0f) {
    return 32767;
  }
  if (scaled < -32768.0f) {
    return -32768;
  }
  return (int16_t)scaled;
}

/*
 * 250 ms마다 PID와 IMU 상태를 번갈아 보내 UART 부하를 제한한다.
 * 9600 baud의 블로킹 전송은 Low 우선순위 태스크에서만 수행한다.
 */
static void StartStatusTask(void *argument) {
  DrivePidStatus pid;
  IMU_Data imu;
  uint8_t send_imu = 0U;
  uint8_t flags;

  (void)argument;
  for (;;) {
    if (send_imu == 0U) {
      Drive4WD_GetPidStatus(&pid);
      flags = (uint8_t)((pid.command_active != 0U ? 1U : 0U) |
                        (pid.pid_enabled != 0U ? 2U : 0U) |
                        (pid.pid_running != 0U ? 4U : 0U) |
                        (pid.reverse != 0U ? 8U : 0U) |
                        (pid.target_updated != 0U ? 16U : 0U) |
                        (pid.imu_available != 0U ? 32U : 0U));
      (void)Bluetooth_SendPidStatus(
          flags,
          ScaleStatusValue(pid.target_yaw_deg, 100.0f),
          ScaleStatusValue(pid.current_yaw_deg, 100.0f),
          ScaleStatusValue(pid.error_deg, 100.0f),
          ScaleStatusValue(pid.output, 100.0f),
          pid.left_pwm,
          pid.right_pwm);
    } else if (IMU_GetLatest(&imu) != 0U) {
      flags = (uint8_t)((imu.initialized != 0U ? 1U : 0U) |
                        (imu.calibrated != 0U ? 2U : 0U) |
                        (imu.valid != 0U ? 4U : 0U));
      (void)Bluetooth_SendImuStatus(
          flags,
          ScaleStatusValue(imu.temperature_c, 100.0f),
          ScaleStatusValue(imu.gyro_z_dps, 100.0f),
          ScaleStatusValue(imu.yaw_deg, 100.0f),
          imu.error_count);
    }
    send_imu ^= 1U;
    osDelay(STATUS_INTERVAL_MS);
  }
}

/* CPU 정렬에 의존하지 않고 signed 32비트를 Little-endian으로 기록한다. */
static void WriteInt32LittleEndian(uint8_t *data, int32_t value) {
  uint32_t bits = (uint32_t)value;

  data[0] = (uint8_t)bits;
  data[1] = (uint8_t)(bits >> 8U);
  data[2] = (uint8_t)(bits >> 16U);
  data[3] = (uint8_t)(bits >> 24U);
}

/* Little-endian 네 바이트를 signed 32비트 값으로 읽는다. */
static int32_t ReadInt32LittleEndian(const uint8_t *data) {
  uint32_t bits = (uint32_t)data[0] |
                  ((uint32_t)data[1] << 8U) |
                  ((uint32_t)data[2] << 16U) |
                  ((uint32_t)data[3] << 24U);

  return (int32_t)bits;
}

/* unsigned 16비트를 Little-endian 두 바이트로 기록한다. */
static void WriteUInt16LittleEndian(uint8_t *data, uint16_t value) {
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

/* Little-endian 두 바이트를 unsigned 16비트 값으로 읽는다. */
static uint16_t ReadUInt16LittleEndian(const uint8_t *data) {
  return (uint16_t)data[0] |
         ((uint16_t)data[1] << 8U);
}

/* PID, 3점 보정값과 5관절 이동 자세를 고정 53바이트 배열로 만든다. */
static void SerializeSettings(const RobotSettings *settings, uint8_t *data) {
  uint8_t joint;
  uint8_t point;

  WriteInt32LittleEndian(&data[0], settings->pid_kp_milli);
  WriteInt32LittleEndian(&data[4], settings->pid_ki_milli);
  WriteInt32LittleEndian(&data[8], settings->pid_kd_milli);
  for (joint = 0U; joint < ROBOT_SETTINGS_SERVO_COUNT; joint++) {
    for (point = 0U; point < ROBOT_SETTINGS_PULSE_COUNT; point++) {
      WriteUInt16LittleEndian(
          &data[12U + (joint * ROBOT_SETTINGS_PULSE_COUNT * 2U) +
                (point * 2U)],
          settings->servo_pulse_us[joint][point]);
    }
  }
  memcpy(&data[SETTINGS_TRAVEL_POSE_OFFSET],
         settings->travel_pose_angles,
         ROBOT_SETTINGS_TRAVEL_JOINT_COUNT);
}

/* 앱에서 받은 53바이트 배열을 펌웨어 설정 구조체로 복원한다. */
static void DeserializeSettings(const uint8_t *data, RobotSettings *settings) {
  uint8_t joint;
  uint8_t point;

  memset(settings, 0, sizeof(*settings));
  settings->pid_kp_milli = ReadInt32LittleEndian(&data[0]);
  settings->pid_ki_milli = ReadInt32LittleEndian(&data[4]);
  settings->pid_kd_milli = ReadInt32LittleEndian(&data[8]);
  for (joint = 0U; joint < ROBOT_SETTINGS_SERVO_COUNT; joint++) {
    for (point = 0U; point < ROBOT_SETTINGS_PULSE_COUNT; point++) {
      settings->servo_pulse_us[joint][point] =
          ReadUInt16LittleEndian(
              &data[12U + (joint * ROBOT_SETTINGS_PULSE_COUNT * 2U) +
                    (point * 2U)]);
    }
  }

  /* Gripper 0도는 앱에서 설정하지 않고 열림과 닫힘의 평균을 사용한다. */
  settings->servo_pulse_us[ROBOT_ARM_GRIPPER_INDEX][1] =
      (uint16_t)((settings->servo_pulse_us[ROBOT_ARM_GRIPPER_INDEX][0] +
                  settings->servo_pulse_us[ROBOT_ARM_GRIPPER_INDEX][2]) /
                 2U);
  memcpy(settings->travel_pose_angles,
         &data[SETTINGS_TRAVEL_POSE_OFFSET],
         ROBOT_SETTINGS_TRAVEL_JOINT_COUNT);
}

/* 설정 조각 COMMIT에서 사용하는 CRC-16/CCITT를 계산한다. */
static uint16_t CalculateSettingsCrc(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFFU;
  uint8_t index;
  uint8_t bit;

  for (index = 0U; index < length; index++) {
    crc ^= (uint16_t)data[index] << 8U;
    for (bit = 0U; bit < 8U; bit++) {
      crc = ((crc & 0x8000U) != 0U)
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
    }
  }

  return crc;
}

/* Mode 3 조회·조각 저장·미리보기 명령을 태스크 문맥에서 처리한다. */
static void HandleSettingsCommand(const BluetoothArmCommand *command) {
  RobotSettings settings;
  RobotSettings previous;
  RobotArmResult arm_result;
  HAL_StatusTypeDef status;
  uint16_t expected_crc;
  uint16_t received_crc;
  uint8_t read_data[SETTINGS_READ_PART_SIZE];
  uint8_t part;
  uint8_t offset;
  uint8_t length;
  uint8_t calibrations_changed;
  uint8_t pid_changed;
  uint8_t travel_joint;
  uint8_t travel_pose_valid;
  uint8_t result = SETTINGS_RESULT_INVALID;

  if (command->data[0] == BLUETOOTH_SETTINGS_PID_ENABLE) {
    DrivePidResult pid_result =
        Drive4WD_SetPidEnabled(command->data[1]);
    uint8_t reason = PID_TOGGLE_RESULT_OK;
    if (pid_result == DRIVE_PID_UNAVAILABLE) {
      reason = PID_TOGGLE_RESULT_UNAVAILABLE;
    } else if (pid_result == DRIVE_PID_INVALID_GAINS) {
      reason = PID_TOGGLE_RESULT_INVALID_GAINS;
    }
    (void)Bluetooth_SendPidAck(
        (uint8_t)(pid_result == DRIVE_PID_OK),
        Drive4WD_IsPidEnabled(),
        reason,
        command->data[2]);
    return;
  }

  if (command->data[0] == BLUETOOTH_SETTINGS_PREVIEW) {
    uint16_t pulse_us = (uint16_t)command->data[2] |
                        ((uint16_t)command->data[3] << 8U);
    Drive4WD_SetArmMotionInhibit(1U);
    arm_result = RobotArm_PreviewPulse(command->data[1],
                                       pulse_us,
                                       command->data[4],
                                       command->data[5]);
    if (arm_result != ROBOT_ARM_OK) {
      Drive4WD_SetArmMotionInhibit(0U);
      (void)Bluetooth_SendPreviewResult(command->data[1],
                                        0U,
                                        (uint8_t)arm_result,
                                        pulse_us,
                                        command->data[5]);
    }
    return;
  }

  if (command->data[0] == BLUETOOTH_SETTINGS_PREVIEW_STOP) {
    RobotArm_StopPreview();
    Drive4WD_SetArmMotionInhibit(0U);
    return;
  }

  if (command->data[0] == BLUETOOTH_SETTINGS_GET) {
    settings_upload_parts = 0U;
    TeachingStorage_GetSettings(&settings);
    SerializeSettings(&settings, settings_upload);
    for (part = 0U; part < SETTINGS_READ_PART_COUNT; part++) {
      offset = (uint8_t)(part * SETTINGS_READ_PART_SIZE);
      length = (uint8_t)(SETTINGS_SERIALIZED_SIZE - offset);
      if (length > SETTINGS_READ_PART_SIZE) {
        length = SETTINGS_READ_PART_SIZE;
      }
      memset(read_data, 0, sizeof(read_data));
      memcpy(read_data, &settings_upload[offset], length);
      (void)Bluetooth_SendSettingsPart(part,
                                       Drive4WD_IsPidEnabled(),
                                       read_data,
                                       command->data[1]);
    }
    return;
  }

  if (command->data[0] == BLUETOOTH_SETTINGS_WRITE_PART) {
    part = command->data[1];
    if (part == 0U) {
      settings_upload_parts = 0U;
      memset(settings_upload, 0, sizeof(settings_upload));
    }
    offset = (uint8_t)(part * SETTINGS_WRITE_PART_SIZE);
    length = (uint8_t)(SETTINGS_SERIALIZED_SIZE - offset);
    if (length > SETTINGS_WRITE_PART_SIZE) {
      length = SETTINGS_WRITE_PART_SIZE;
    }
    memcpy(&settings_upload[offset], &command->data[2], length);
    settings_upload_parts |= (uint8_t)(1U << part);
    return;
  }

  if (command->data[0] != BLUETOOTH_SETTINGS_COMMIT) {
    return;
  }

  if (settings_upload_parts !=
      ((1U << SETTINGS_WRITE_PART_COUNT) - 1U)) {
    result = SETTINGS_RESULT_INCOMPLETE;
  } else {
    received_crc = (uint16_t)command->data[2] |
                   ((uint16_t)command->data[3] << 8U);
    expected_crc =
        CalculateSettingsCrc(settings_upload, SETTINGS_SERIALIZED_SIZE);
    if (received_crc != expected_crc) {
      result = SETTINGS_RESULT_CRC;
    } else {
      DeserializeSettings(settings_upload, &settings);
      TeachingStorage_GetSettings(&previous);
      calibrations_changed =
          (uint8_t)(memcmp(settings.servo_pulse_us,
                           previous.servo_pulse_us,
                           sizeof(settings.servo_pulse_us)) != 0);
      pid_changed =
          (uint8_t)((settings.pid_kp_milli != previous.pid_kp_milli) ||
                    (settings.pid_ki_milli != previous.pid_ki_milli) ||
                     (settings.pid_kd_milli != previous.pid_kd_milli));
      travel_pose_valid = 1U;
      for (travel_joint = 0U;
           travel_joint < ROBOT_SETTINGS_TRAVEL_JOINT_COUNT;
           travel_joint++) {
        if (settings.travel_pose_angles[travel_joint] > 180U) {
          travel_pose_valid = 0U;
          break;
        }
      }

      if ((settings.pid_kp_milli < 0) ||
          (settings.pid_kp_milli > PID_KP_MAX_MILLI) ||
          ((settings.pid_kp_milli % PID_KP_STEP_MILLI) != 0L) ||
          (settings.pid_ki_milli < 0) ||
          (settings.pid_ki_milli > PID_GAIN_MAX_MILLI) ||
          (settings.pid_kd_milli < 0) ||
          (settings.pid_kd_milli > PID_GAIN_MAX_MILLI)) {
        result = SETTINGS_RESULT_PID_INVALID;
      } else if (travel_pose_valid == 0U) {
        result = SETTINGS_RESULT_INVALID;
      } else if ((pid_changed != 0U) &&
                 (Drive4WD_IsPidEnabled() != 0U)) {
        result = SETTINGS_RESULT_PID_ACTIVE;
      } else {
        arm_result = ROBOT_ARM_OK;
        if (calibrations_changed != 0U) {
          arm_result =
              RobotArm_SetCalibrations(settings.servo_pulse_us);
        }

        if (arm_result != ROBOT_ARM_OK) {
          result = SETTINGS_RESULT_INVALID;
        } else {
          Drive4WD_SetStorageInhibit(1U);
          status = HAL_ERROR;
          if (osMutexAcquire(flashMutexHandle, osWaitForever) == osOK) {
            status = TeachingStorage_SaveSettings(&settings);
            (void)osMutexRelease(flashMutexHandle);
          }
          Drive4WD_SetStorageInhibit(0U);

          if (status == HAL_OK) {
            (void)Drive4WD_SetPidGains(settings.pid_kp_milli,
                                       settings.pid_ki_milli,
                                       settings.pid_kd_milli);
            result = SETTINGS_RESULT_OK;
          } else {
            result = SETTINGS_RESULT_FLASH;
            if (calibrations_changed != 0U) {
              (void)RobotArm_SetCalibrations(previous.servo_pulse_us);
            }
          }
        }
      }
    }
  }

  settings_upload_parts = 0U;
  (void)Bluetooth_SendSettingsAck(
      (uint8_t)(result == SETTINGS_RESULT_OK),
      result,
      Drive4WD_IsPidEnabled());
}

/* USER CODE END Application */
