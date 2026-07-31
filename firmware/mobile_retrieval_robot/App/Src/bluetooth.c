#include "bluetooth.h"

#include <string.h>

#include "servo_driver.h"

/*
 * HC-05의 연속 UART 바이트를 16바이트 패킷으로 복구한다.
 * ISR은 DMA 위치만 갱신하고, 검증·큐 전달·ACK 송신은 태스크 문맥에서 수행한다.
 */

#define PACKET_HEADER       (0xAAU)
#define PACKET_TAIL         (0x55U)
#define DMA_BUFFER_SIZE     (128U)
#define MAX_SEQUENCE_COUNT  (12U)
#define MAX_WAYPOINT_COUNT  (30U)
#define MAX_ARM_ANGLE       (180U)
#define TEACHING_UPLOAD     (4U)
#define TEACHING_NAME_MAX_BYTES (21U)
#define TEACHING_NAME_UPLOAD_PART_COUNT (3U)
#define TEACHING_NAME_READ_PART_SIZE (7U)
#define SETTINGS_SIZE            (53U)
#define SETTINGS_WRITE_PART_COUNT (6U)
#define SETTINGS_READ_PART_COUNT  (7U)
#define SETTINGS_WRITE_PART_SIZE (10U)
#define SETTINGS_READ_PART_SIZE  (8U)
#define SETTINGS_SERVO_COUNT     (6U)
#define SETTINGS_SERVO_MIN_US    (350U)
#define SETTINGS_SERVO_MAX_US    (2650U)
#define SETTINGS_GRIPPER_INDEX   (5U)
#define SETTINGS_GRIPPER_MIN_US  (1000U)
#define SETTINGS_GRIPPER_MAX_US  (2000U)
#define ARM_SPEED_MIN_PERCENT    (50U)
#define ARM_SPEED_MAX_PERCENT    (100U)
#define CHECKSUM_INDEX      (BLUETOOTH_PACKET_SIZE - 2U)
#define TAIL_INDEX          (BLUETOOTH_PACKET_SIZE - 1U)

_Static_assert((DMA_BUFFER_SIZE % BLUETOOTH_PACKET_SIZE) == 0U,
               "DMA buffer must contain whole Bluetooth packets");

static UART_HandleTypeDef *bluetooth_uart;
static osMutexId_t bluetooth_tx_mutex;
static uint8_t dma_buffer[DMA_BUFFER_SIZE];
static uint16_t dma_read_index;
static uint8_t packet_buffer[BLUETOOTH_PACKET_SIZE];
static uint8_t packet_index;
static uint8_t previous_mode = BLUETOOTH_MODE_DRIVE;
static volatile uint8_t uart_error_pending;

/* Mode부터 Data11까지 더해 프로토콜 체크섬을 만든다. */
static uint8_t CalculateChecksum(const uint8_t *packet)
{
  uint16_t sum = 0U;
  uint8_t index;

  for (index = 1U; index < CHECKSUM_INDEX; index++)
  {
    sum = (uint16_t)(sum + packet[index]);
  }

  return (uint8_t)sum;
}

/* 상태 패킷의 signed 16비트 값을 Little-endian으로 기록한다. */
static void WriteInt16LittleEndian(uint8_t *data, int16_t value)
{
  uint16_t bits = (uint16_t)value;
  data[0] = (uint8_t)bits;
  data[1] = (uint8_t)(bits >> 8U);
}

/* 상태 패킷의 32비트 값을 Little-endian으로 기록한다. */
static void WriteUint32LittleEndian(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

/* 여러 태스크의 ACK와 상태 패킷이 UART에서 섞이지 않게 직렬화한다. */
static HAL_StatusTypeDef TransmitPacket(const uint8_t *packet)
{
  HAL_StatusTypeDef status;

  if ((bluetooth_uart == NULL) ||
      (bluetooth_tx_mutex == NULL) ||
      (packet == NULL) ||
      (osMutexAcquire(bluetooth_tx_mutex, 100U) != osOK))
  {
    return HAL_ERROR;
  }

  status = HAL_UART_Transmit(bluetooth_uart,
                            packet,
                            BLUETOOTH_PACKET_SIZE,
                            100U);
  (void)osMutexRelease(bluetooth_tx_mutex);
  return status;
}

/* 지정한 위치 이후의 예약 Data가 모두 0인지 확인한다. */
static uint8_t IsUnusedDataZero(const uint8_t *data, uint8_t start)
{
  uint8_t index;

  for (index = start; index < BLUETOOTH_DATA_SIZE; index++)
  {
    if (data[index] != 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

/* 티칭 재생·조회·조각 업로드 명령의 필드 범위를 검사한다. */
static uint8_t IsTeachingDataValid(const uint8_t *data)
{
  uint8_t index;

  if (data[0] == 2U)
  {
    return ((data[1] >= 1U) &&
            (data[1] <= MAX_SEQUENCE_COUNT) &&
            (data[2] >= ARM_SPEED_MIN_PERCENT) &&
            (data[2] <= ARM_SPEED_MAX_PERCENT) &&
            (IsUnusedDataZero(data, 3U) != 0U));
  }

  if (data[0] == 3U)
  {
    return ((data[1] >= 1U) &&
            (data[1] <= MAX_SEQUENCE_COUNT) &&
            (IsUnusedDataZero(data, 2U) != 0U));
  }

  if (data[0] == BLUETOOTH_TEACHING_GET_NAME)
  {
    return ((data[1] >= 1U) &&
            (data[1] <= MAX_SEQUENCE_COUNT) &&
            (data[2] != 0U) &&
            (IsUnusedDataZero(data, 3U) != 0U));
  }

  if (data[0] == BLUETOOTH_TEACHING_GET_SEQUENCE)
  {
    return ((data[1] >= 1U) &&
            (data[1] <= MAX_SEQUENCE_COUNT) &&
            (data[2] != 0U) &&
            (IsUnusedDataZero(data, 3U) != 0U));
  }

  if (data[0] == BLUETOOTH_TEACHING_PREVIEW)
  {
    return ((data[1] >= 1U) &&
            (data[1] <= MAX_SEQUENCE_COUNT) &&
            (data[2] >= ARM_SPEED_MIN_PERCENT) &&
            (data[2] <= ARM_SPEED_MAX_PERCENT) &&
            (IsUnusedDataZero(data, 3U) != 0U));
  }

  if (data[0] != TEACHING_UPLOAD)
  {
    return 0U;
  }

  if ((data[1] < 1U) || (data[1] > 5U) ||
      (data[2] < 1U) || (data[2] > MAX_SEQUENCE_COUNT))
  {
    return 0U;
  }

  if (data[1] == 1U)
  {
    return ((data[3] >= 1U) &&
            (data[3] <= MAX_WAYPOINT_COUNT) &&
            (data[4] >= 1U) &&
            (data[4] <= TEACHING_NAME_MAX_BYTES) &&
            (IsUnusedDataZero(data, 5U) != 0U));
  }

  if ((data[1] == 2U) || (data[1] == 3U))
  {
    if (data[3] >= MAX_WAYPOINT_COUNT)
    {
      return 0U;
    }

    for (index = 4U; index < 7U; index++)
    {
      if (data[index] > MAX_ARM_ANGLE)
      {
        return 0U;
      }
    }
    return IsUnusedDataZero(data, 7U);
  }

  if (data[1] == 5U)
  {
    return (data[3] < TEACHING_NAME_UPLOAD_PART_COUNT);
  }

  return (IsUnusedDataZero(data, 3U) != 0U);
}

/* 설정 조회·저장·서보 미리보기 명령의 길이와 값 범위를 검사한다. */
static uint8_t IsSettingsDataValid(const uint8_t *data)
{
  if (data[0] == BLUETOOTH_SETTINGS_GET)
  {
    return ((data[1] != 0U) && (IsUnusedDataZero(data, 2U) != 0U));
  }

  if (data[0] == BLUETOOTH_SETTINGS_WRITE_PART)
  {
    if (data[1] >= SETTINGS_WRITE_PART_COUNT)
    {
      return 0U;
    }

    /* 마지막 조각은 이동 자세 3바이트만 사용하고 나머지는 0으로 채운다. */
    if (data[1] == (SETTINGS_WRITE_PART_COUNT - 1U))
    {
      return IsUnusedDataZero(data, 5U);
    }
    return 1U;
  }

  if (data[0] == BLUETOOTH_SETTINGS_COMMIT)
  {
    return ((data[1] == SETTINGS_SIZE) &&
            (IsUnusedDataZero(data, 4U) != 0U));
  }

  if (data[0] == BLUETOOTH_SETTINGS_PREVIEW)
  {
    uint16_t pulse_us = (uint16_t)data[2] |
                        ((uint16_t)data[3] << 8U);

    if ((data[1] >= SETTINGS_SERVO_COUNT) ||
        (data[4] < 50U) ||
        (data[4] > 100U) ||
        (data[5] == 0U) ||
        (IsUnusedDataZero(data, 6U) == 0U))
    {
      return 0U;
    }

    if (data[1] == SETTINGS_GRIPPER_INDEX)
    {
      return ((pulse_us >= SETTINGS_GRIPPER_MIN_US) &&
              (pulse_us <= SETTINGS_GRIPPER_MAX_US));
    }
    return ((pulse_us >= SETTINGS_SERVO_MIN_US) &&
            (pulse_us <= SETTINGS_SERVO_MAX_US));
  }

  if (data[0] == BLUETOOTH_SETTINGS_PREVIEW_STOP)
  {
    return IsUnusedDataZero(data, 1U);
  }

  if (data[0] == BLUETOOTH_SETTINGS_PID_ENABLE)
  {
    return ((data[1] <= 1U) &&
            (data[2] != 0U) &&
            (IsUnusedDataZero(data, 3U) != 0U));
  }

  return 0U;
}

/* 공통 프레임과 Mode별 규칙을 모두 만족하는 명령인지 확인한다. */
static uint8_t IsPacketValid(const uint8_t *packet)
{
  const uint8_t *data = &packet[2];
  uint8_t index;

  if ((packet[0] != PACKET_HEADER) ||
      (packet[TAIL_INDEX] != PACKET_TAIL) ||
      (packet[CHECKSUM_INDEX] != CalculateChecksum(packet)))
  {
    return 0U;
  }

  if (packet[1] == BLUETOOTH_MODE_DRIVE)
  {
    if ((data[0] > 1U) ||
        (data[2] > 1U) ||
        (data[4] > BLUETOOTH_DRIVE_ESTOP_CLEAR) ||
        (data[5] > 1U) ||
        (data[6] > 1U) ||
        (data[7] > 1U) ||
        (IsUnusedDataZero(data, 8U) == 0U))
    {
      return 0U;
    }

    /* 비상정지와 해제 패킷은 모든 구동 요청이 0이어야 한다. */
    if ((data[4] != BLUETOOTH_DRIVE_NORMAL) &&
        ((data[0] != 0U) ||
         (data[1] != 0U) ||
         (data[2] != 0U) ||
         (data[3] != 0U) ||
         (data[5] != 0U) ||
         (data[6] != 0U) ||
         (data[7] != 0U)))
    {
      return 0U;
    }

    /* 엔진 OFF 또는 PID 미요청 상태에는 모순된 부가 요청을 허용하지 않는다. */
    if (((data[5] == 0U) &&
         ((data[1] != 0U) || (data[3] != 0U) ||
          (data[6] != 0U) || (data[7] != 0U))) ||
        ((data[6] == 0U) && (data[7] != 0U)) ||
        ((data[6] != 0U) &&
         ((data[0] != data[2]) ||
          ((data[1] == 0U) && (data[3] == 0U)))))
    {
      return 0U;
    }

    return 1U;
  }

  if (packet[1] == BLUETOOTH_MODE_ARM)
  {
    if (data[6] == BLUETOOTH_ARM_TRAVEL_KEEP_GRIPPER)
    {
      return ((data[0] == 0U) && (data[1] == 0U) &&
              (data[2] == 0U) && (data[3] == 0U) &&
              (data[4] == 0U) && (data[5] == 0U) &&
              (data[7] != 0U) &&
              (data[8] >= ARM_SPEED_MIN_PERCENT) &&
              (data[8] <= ARM_SPEED_MAX_PERCENT) &&
              (IsUnusedDataZero(data, 9U) != 0U));
    }

    for (index = 0U; index < 6U; index++)
    {
      if (data[index] > MAX_ARM_ANGLE)
      {
        return 0U;
      }
    }
    return ((data[6] <= BLUETOOTH_ARM_DISABLE) &&
            /* Data7은 앱이 발급한 로봇팔 명령 식별 번호다. */
            (data[7] != 0U) &&
            (data[8] >= ARM_SPEED_MIN_PERCENT) &&
            (data[8] <= ARM_SPEED_MAX_PERCENT) &&
            (IsUnusedDataZero(data, 9U) != 0U));
  }

  if (packet[1] == BLUETOOTH_MODE_TEACHING)
  {
    return IsTeachingDataValid(data);
  }

  if (packet[1] == BLUETOOTH_MODE_SETTINGS)
  {
    return IsSettingsDataValid(data);
  }

  return 0U;
}

/* 큐가 가득 찬 경우 오래된 명령을 버리고 최신 명령을 보존한다. */
static void PutLatest(osMessageQueueId_t queue, const void *message)
{
  union
  {
    BluetoothDriveCommand drive;
    BluetoothArmCommand arm;
  } old_message;
  uint8_t priority;

  if (osMessageQueuePut(queue, message, 0U, 0U) == osOK)
  {
    return;
  }

  /* 큐가 가득 찼으면 가장 오래된 명령을 버리고 최신 명령을 보관한다. */
  (void)osMessageQueueGet(queue, &old_message, &priority, 0U);
  (void)osMessageQueuePut(queue, message, 0U, 0U);
}

/* 검증된 패킷을 주행 큐 또는 로봇팔·설정 큐로 전달한다. */
static void DispatchPacket(const uint8_t *packet,
                           osMessageQueueId_t arm_queue,
                           osMessageQueueId_t drive_queue)
{
  BluetoothDriveCommand drive_command;
  BluetoothArmCommand arm_command;

  if ((previous_mode == BLUETOOTH_MODE_DRIVE) &&
      (packet[1] != BLUETOOTH_MODE_DRIVE))
  {
    memset(&drive_command, 0, sizeof(drive_command));
    drive_command.received_tick = HAL_GetTick();
    (void)osMessageQueueReset(drive_queue);
    PutLatest(drive_queue, &drive_command);
  }
  previous_mode = packet[1];

  if (packet[1] == BLUETOOTH_MODE_DRIVE)
  {
    drive_command.left_direction = packet[2];
    drive_command.left_pwm = packet[3];
    drive_command.right_direction = packet[4];
    drive_command.right_pwm = packet[5];
    drive_command.control = packet[6];
    drive_command.engine_enabled = packet[7];
    drive_command.pid_straight = packet[8];
    drive_command.refresh_yaw_target = packet[9];
    drive_command.received_tick = HAL_GetTick();

    /* 정지와 비상 제어는 대기 중인 이전 주행 명령보다 우선한다. */
    if ((drive_command.control != BLUETOOTH_DRIVE_NORMAL) ||
        ((drive_command.left_pwm == 0U) &&
         (drive_command.right_pwm == 0U)))
    {
      (void)osMessageQueueReset(drive_queue);
    }

    /* 비상정지 한 패킷으로 차량과 로봇팔 출력을 함께 안전하게 끈다. */
    if (drive_command.control == BLUETOOTH_DRIVE_ESTOP)
    {
      /*
       * ArmTask가 Flash 작업 중이어도 PCA9685 출력을 먼저 차단한다.
       * 상태 정리는 아래 ArmQueue의 DISABLE 명령에서 다시 수행한다.
       */
      (void)ServoDriver_DisableAll();
      memset(&arm_command, 0, sizeof(arm_command));
      arm_command.mode = BLUETOOTH_MODE_ARM;
      arm_command.data[6] = BLUETOOTH_ARM_DISABLE;
      arm_command.data[7] = 0xFFU; /* 펌웨어 내부 안전 명령용 식별 번호 */
      (void)osMessageQueueReset(arm_queue);
      PutLatest(arm_queue, &arm_command);
    }

    PutLatest(drive_queue, &drive_command);
    return;
  }

  arm_command.mode = packet[1];
  memcpy(arm_command.data, &packet[2], BLUETOOTH_DATA_SIZE);
  PutLatest(arm_queue, &arm_command);
}

/* 손상된 프레임 안에서 다음 Header를 찾아 수신 상태를 복구한다. */
static void ResynchronizePacket(void)
{
  uint8_t header_index;

  for (header_index = 1U;
       header_index < BLUETOOTH_PACKET_SIZE;
       header_index++)
  {
    if (packet_buffer[header_index] == PACKET_HEADER)
    {
      packet_index = (uint8_t)(BLUETOOTH_PACKET_SIZE - header_index);
      memmove(packet_buffer,
              &packet_buffer[header_index],
              packet_index);
      return;
    }
  }

  packet_index = 0U;
}

/* DMA에서 읽은 바이트 하나를 현재 패킷 버퍼에 누적한다. */
static void ProcessByte(uint8_t byte,
                        osMessageQueueId_t arm_queue,
                        osMessageQueueId_t drive_queue)
{
  if ((packet_index == 0U) && (byte != PACKET_HEADER))
  {
    return;
  }

  packet_buffer[packet_index] = byte;
  packet_index++;

  if (packet_index < BLUETOOTH_PACKET_SIZE)
  {
    return;
  }

  if (IsPacketValid(packet_buffer) != 0U)
  {
    DispatchPacket(packet_buffer, arm_queue, drive_queue);
    packet_index = 0U;
  }
  else
  {
    ResynchronizePacket();
  }
}

HAL_StatusTypeDef Bluetooth_Init(UART_HandleTypeDef *uart)
{
  if ((uart == NULL) || (uart->hdmarx == NULL))
  {
    return HAL_ERROR;
  }

  bluetooth_uart = uart;
  if (bluetooth_tx_mutex == NULL)
  {
    bluetooth_tx_mutex = osMutexNew(NULL);
    if (bluetooth_tx_mutex == NULL)
    {
      bluetooth_uart = NULL;
      return HAL_ERROR;
    }
  }
  dma_read_index = 0U;
  packet_index = 0U;
  uart_error_pending = 0U;

  return HAL_UART_Receive_DMA(bluetooth_uart,
                              dma_buffer,
                              DMA_BUFFER_SIZE);
}

void Bluetooth_Process(osMessageQueueId_t arm_queue,
                       osMessageQueueId_t drive_queue)
{
  uint16_t write_index;

  if ((bluetooth_uart == NULL) ||
      (arm_queue == NULL) ||
      (drive_queue == NULL))
  {
    return;
  }

  if (uart_error_pending != 0U)
  {
    /* 오류 콜백에서는 플래그만 세우고 실제 DMA 재시작은 태스크에서 한다. */
    uart_error_pending = 0U;
    (void)HAL_UART_AbortReceive(bluetooth_uart);
    dma_read_index = 0U;
    packet_index = 0U;
    if (HAL_UART_Receive_DMA(bluetooth_uart,
                             dma_buffer,
                             DMA_BUFFER_SIZE) != HAL_OK)
    {
      uart_error_pending = 1U;
    }
    return;
  }

  write_index = (uint16_t)(DMA_BUFFER_SIZE -
      __HAL_DMA_GET_COUNTER(bluetooth_uart->hdmarx));

  while (dma_read_index != write_index)
  {
    ProcessByte(dma_buffer[dma_read_index], arm_queue, drive_queue);
    dma_read_index++;
    if (dma_read_index >= DMA_BUFFER_SIZE)
    {
      dma_read_index = 0U;
    }
  }
}

HAL_StatusTypeDef Bluetooth_SendTeachingAck(uint8_t command,
                                            uint8_t sequence_id,
                                            uint8_t status,
                                            uint8_t speed_warning)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};

  if ((bluetooth_uart == NULL) ||
      ((command != 2U) && (command != 3U) &&
       (command != TEACHING_UPLOAD)) ||
      (sequence_id < 1U) ||
      (sequence_id > MAX_SEQUENCE_COUNT) ||
      (status > 1U) ||
      (speed_warning > 1U))
  {
    return HAL_ERROR;
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_TEACHING;
  packet[2] = command;
  packet[3] = sequence_id;
  packet[4] = status;
  packet[5] = speed_warning;
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;

  return TransmitPacket(packet);
}

HAL_StatusTypeDef Bluetooth_SendTeachingNamePart(uint8_t sequence_id,
                                                 uint8_t part,
                                                 uint8_t name_length,
                                                 uint8_t request_id,
                                                 const uint8_t data[7])
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};

  if ((bluetooth_uart == NULL) ||
      (sequence_id < 1U) ||
      (sequence_id > MAX_SEQUENCE_COUNT) ||
      (part >= TEACHING_NAME_UPLOAD_PART_COUNT) ||
      (name_length < 1U) ||
      (name_length > TEACHING_NAME_MAX_BYTES) ||
      (request_id == 0U) ||
      (data == NULL))
  {
    return HAL_ERROR;
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_TEACHING;
  packet[2] = BLUETOOTH_TEACHING_GET_NAME;
  packet[3] = sequence_id;
  packet[4] = part;
  packet[5] = name_length;
  packet[6] = request_id;
  memcpy(&packet[7], data, TEACHING_NAME_READ_PART_SIZE);
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;
  return TransmitPacket(packet);
}

HAL_StatusTypeDef Bluetooth_SendTeachingSequenceMeta(uint8_t sequence_id,
                                                     uint8_t waypoint_count,
                                                     uint8_t request_id)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};

  if ((bluetooth_uart == NULL) ||
      (sequence_id < 1U) ||
      (sequence_id > MAX_SEQUENCE_COUNT) ||
      (waypoint_count > MAX_WAYPOINT_COUNT) ||
      (request_id == 0U))
  {
    return HAL_ERROR;
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_TEACHING;
  packet[2] = BLUETOOTH_TEACHING_GET_SEQUENCE;
  packet[3] = sequence_id;
  packet[4] = 0U;
  packet[5] = waypoint_count;
  packet[6] = request_id;
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;
  return TransmitPacket(packet);
}

HAL_StatusTypeDef Bluetooth_SendTeachingWaypoint(uint8_t sequence_id,
                                                 uint8_t waypoint_index,
                                                 const uint8_t angles[6],
                                                 uint8_t request_id)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};
  uint8_t joint;

  if ((bluetooth_uart == NULL) ||
      (sequence_id < 1U) ||
      (sequence_id > MAX_SEQUENCE_COUNT) ||
      (waypoint_index >= MAX_WAYPOINT_COUNT) ||
      (angles == NULL) ||
      (request_id == 0U))
  {
    return HAL_ERROR;
  }
  for (joint = 0U; joint < 6U; joint++)
  {
    if (angles[joint] > MAX_ARM_ANGLE)
    {
      return HAL_ERROR;
    }
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_TEACHING;
  packet[2] = BLUETOOTH_TEACHING_GET_SEQUENCE;
  packet[3] = sequence_id;
  packet[4] = 1U;
  packet[5] = waypoint_index;
  memcpy(&packet[6], angles, 6U);
  packet[12] = request_id;
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;
  return TransmitPacket(packet);
}

HAL_StatusTypeDef Bluetooth_SendDriveAck(uint8_t control,
                                        uint8_t status,
                                        uint8_t reason)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};

  if ((bluetooth_uart == NULL) ||
      (control == BLUETOOTH_DRIVE_NORMAL) ||
      (control > BLUETOOTH_DRIVE_ESTOP_CLEAR) ||
      (status > 1U))
  {
    return HAL_ERROR;
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_DRIVE;
  packet[2] = control;
  packet[3] = status;
  packet[4] = reason;
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;

  return TransmitPacket(packet);
}

HAL_StatusTypeDef Bluetooth_SendArmAck(uint8_t command,
                                       uint8_t status,
                                       uint8_t reason,
                                       uint8_t transaction_id)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};

  if ((bluetooth_uart == NULL) ||
      (command > BLUETOOTH_ARM_TRAVEL_KEEP_GRIPPER) ||
      (status > 1U) ||
      (transaction_id == 0U))
  {
    return HAL_ERROR;
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_ARM;
  packet[2] = command;
  packet[3] = status;
  packet[4] = reason;
  packet[5] = transaction_id;
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;

  return TransmitPacket(packet);
}

HAL_StatusTypeDef Bluetooth_SendSettingsPart(uint8_t part,
                                             uint8_t pid_applied,
                                             const uint8_t data[8],
                                             uint8_t request_id)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};

  if ((bluetooth_uart == NULL) ||
      (part >= SETTINGS_READ_PART_COUNT) ||
      (pid_applied > 1U) ||
      (request_id == 0U) ||
      (data == NULL))
  {
    return HAL_ERROR;
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_SETTINGS;
  packet[2] = BLUETOOTH_SETTINGS_READ_PART;
  packet[3] = part;
  packet[4] = pid_applied;
  memcpy(&packet[5], data, SETTINGS_READ_PART_SIZE);
  packet[13] = request_id;
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;

  return TransmitPacket(packet);
}

HAL_StatusTypeDef Bluetooth_SendSettingsAck(uint8_t status,
                                            uint8_t reason,
                                            uint8_t pid_applied)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};

  if ((bluetooth_uart == NULL) || (status > 1U) || (pid_applied > 1U))
  {
    return HAL_ERROR;
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_SETTINGS;
  packet[2] = BLUETOOTH_SETTINGS_COMMIT;
  packet[3] = status;
  packet[4] = reason;
  packet[5] = pid_applied;
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;

  return TransmitPacket(packet);
}

HAL_StatusTypeDef Bluetooth_SendPreviewResult(uint8_t joint,
                                              uint8_t status,
                                              uint8_t reason,
                                              uint16_t pulse_us,
                                              uint8_t request_id)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};

  if ((bluetooth_uart == NULL) ||
      (joint >= SETTINGS_SERVO_COUNT) ||
      (status > 1U) ||
      (request_id == 0U))
  {
    return HAL_ERROR;
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_SETTINGS;
  packet[2] = BLUETOOTH_SETTINGS_PREVIEW;
  packet[3] = joint;
  packet[4] = status;
  packet[5] = reason;
  packet[6] = (uint8_t)(pulse_us & 0xFFU);
  packet[7] = (uint8_t)(pulse_us >> 8U);
  packet[8] = request_id;
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;

  return TransmitPacket(packet);
}

HAL_StatusTypeDef Bluetooth_SendPidAck(uint8_t status,
                                      uint8_t pid_applied,
                                      uint8_t reason,
                                      uint8_t request_id)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};

  if ((bluetooth_uart == NULL) ||
      (status > 1U) ||
      (pid_applied > 1U) ||
      (request_id == 0U))
  {
    return HAL_ERROR;
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_SETTINGS;
  packet[2] = BLUETOOTH_SETTINGS_PID_ENABLE;
  packet[3] = status;
  packet[4] = pid_applied;
  packet[5] = reason;
  packet[6] = request_id;
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;

  return TransmitPacket(packet);
}

HAL_StatusTypeDef Bluetooth_SendPidStatus(uint8_t flags,
                                         int16_t target_yaw_cdeg,
                                         int16_t current_yaw_cdeg,
                                         int16_t error_cdeg,
                                         int16_t output_centi,
                                         uint8_t left_pwm,
                                         uint8_t right_pwm)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};

  if (bluetooth_uart == NULL)
  {
    return HAL_ERROR;
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_DRIVE;
  packet[2] = BLUETOOTH_DRIVE_STATUS_PID;
  packet[3] = flags;
  WriteInt16LittleEndian(&packet[4], target_yaw_cdeg);
  WriteInt16LittleEndian(&packet[6], current_yaw_cdeg);
  WriteInt16LittleEndian(&packet[8], error_cdeg);
  WriteInt16LittleEndian(&packet[10], output_centi);
  packet[12] = left_pwm;
  packet[13] = right_pwm;
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;
  return TransmitPacket(packet);
}

HAL_StatusTypeDef Bluetooth_SendImuStatus(uint8_t flags,
                                         int16_t temperature_centi_c,
                                         int16_t gyro_z_centi_dps,
                                         int16_t yaw_cdeg,
                                         uint32_t error_count)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {0U};

  if (bluetooth_uart == NULL)
  {
    return HAL_ERROR;
  }

  packet[0] = PACKET_HEADER;
  packet[1] = BLUETOOTH_MODE_DRIVE;
  packet[2] = BLUETOOTH_DRIVE_STATUS_IMU;
  packet[3] = flags;
  WriteInt16LittleEndian(&packet[4], temperature_centi_c);
  WriteInt16LittleEndian(&packet[6], gyro_z_centi_dps);
  WriteInt16LittleEndian(&packet[8], yaw_cdeg);
  WriteUint32LittleEndian(&packet[10], error_count);
  packet[CHECKSUM_INDEX] = CalculateChecksum(packet);
  packet[TAIL_INDEX] = PACKET_TAIL;
  return TransmitPacket(packet);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart == bluetooth_uart)
  {
    uart_error_pending = 1U;
  }
}
