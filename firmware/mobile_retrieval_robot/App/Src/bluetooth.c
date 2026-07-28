#include "bluetooth.h"

#include <string.h>

#define PACKET_HEADER       (0xAAU)
#define PACKET_TAIL         (0x55U)
#define DMA_BUFFER_SIZE     (128U)
#define MAX_SEQUENCE_COUNT  (12U)
#define MAX_WAYPOINT_COUNT  (30U)
#define MAX_ARM_ANGLE       (180U)
#define TEACHING_UPLOAD     (4U)

static UART_HandleTypeDef *bluetooth_uart;
static uint8_t dma_buffer[DMA_BUFFER_SIZE];
static uint16_t dma_read_index;
static uint8_t packet_buffer[BLUETOOTH_PACKET_SIZE];
static uint8_t packet_index;
static uint8_t previous_mode = BLUETOOTH_MODE_DRIVE;
static volatile uint8_t uart_error_pending;

static uint8_t CalculateChecksum(const uint8_t *packet)
{
  uint16_t sum = 0U;
  uint8_t index;

  for (index = 1U; index <= 8U; index++)
  {
    sum = (uint16_t)(sum + packet[index]);
  }

  return (uint8_t)sum;
}

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

static uint8_t IsTeachingDataValid(const uint8_t *data)
{
  uint8_t index;

  if ((data[0] == 2U) || (data[0] == 3U))
  {
    return ((data[1] >= 1U) &&
            (data[1] <= MAX_SEQUENCE_COUNT) &&
            (IsUnusedDataZero(data, 2U) != 0U));
  }

  if (data[0] != TEACHING_UPLOAD)
  {
    return 0U;
  }

  if ((data[1] < 1U) || (data[1] > 4U) ||
      (data[2] < 1U) || (data[2] > MAX_SEQUENCE_COUNT))
  {
    return 0U;
  }

  if (data[1] == 1U)
  {
    return ((data[3] >= 1U) &&
            (data[3] <= MAX_WAYPOINT_COUNT) &&
            (IsUnusedDataZero(data, 4U) != 0U));
  }

  if ((data[1] == 2U) || (data[1] == 3U))
  {
    if (data[3] >= MAX_WAYPOINT_COUNT)
    {
      return 0U;
    }

    for (index = 4U; index < BLUETOOTH_DATA_SIZE; index++)
    {
      if (data[index] > MAX_ARM_ANGLE)
      {
        return 0U;
      }
    }
    return 1U;
  }

  return (IsUnusedDataZero(data, 3U) != 0U);
}

static uint8_t IsPacketValid(const uint8_t *packet)
{
  const uint8_t *data = &packet[2];
  uint8_t index;

  if ((packet[0] != PACKET_HEADER) ||
      (packet[10] != PACKET_TAIL) ||
      (packet[9] != CalculateChecksum(packet)))
  {
    return 0U;
  }

  if (packet[1] == BLUETOOTH_MODE_DRIVE)
  {
    if ((data[0] > 1U) ||
        (data[2] > 1U) ||
        (data[4] > BLUETOOTH_DRIVE_ESTOP_CLEAR) ||
        (IsUnusedDataZero(data, 5U) == 0U))
    {
      return 0U;
    }

    /* 비상정지와 해제 패킷은 모터 명령이 모두 0이어야 한다. */
    if ((data[4] != BLUETOOTH_DRIVE_NORMAL) &&
        ((data[0] != 0U) ||
         (data[1] != 0U) ||
         (data[2] != 0U) ||
         (data[3] != 0U)))
    {
      return 0U;
    }

    return 1U;
  }

  if (packet[1] == BLUETOOTH_MODE_ARM)
  {
    for (index = 0U; index < 6U; index++)
    {
      if (data[index] > MAX_ARM_ANGLE)
      {
        return 0U;
      }
    }
    return (data[6] <= BLUETOOTH_ARM_DISABLE);
  }

  if (packet[1] == BLUETOOTH_MODE_TEACHING)
  {
    return IsTeachingDataValid(data);
  }

  return 0U;
}

static void PutLatest(osMessageQueueId_t queue, const void *message)
{
  uint8_t priority;
  uint8_t old_message[sizeof(BluetoothDriveCommand)];

  if (osMessageQueuePut(queue, message, 0U, 0U) == osOK)
  {
    return;
  }

  /* 큐가 가득 찼으면 가장 오래된 명령을 버리고 최신 명령을 보관한다. */
  (void)osMessageQueueGet(queue, old_message, &priority, 0U);
  (void)osMessageQueuePut(queue, message, 0U, 0U);
}

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
      memset(&arm_command, 0, sizeof(arm_command));
      arm_command.mode = BLUETOOTH_MODE_ARM;
      arm_command.data[6] = BLUETOOTH_ARM_DISABLE;
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

HAL_StatusTypeDef Bluetooth_SendTeachingAck(uint8_t sequence_id,
                                            uint8_t status)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {
      PACKET_HEADER,
      BLUETOOTH_MODE_TEACHING,
      TEACHING_UPLOAD,
      0U,
      0U,
      0U,
      0U,
      0U,
      0U,
      0U,
      PACKET_TAIL};

  if ((bluetooth_uart == NULL) ||
      (sequence_id < 1U) ||
      (sequence_id > MAX_SEQUENCE_COUNT))
  {
    return HAL_ERROR;
  }

  packet[3] = sequence_id;
  packet[4] = status;
  packet[9] = CalculateChecksum(packet);

  return HAL_UART_Transmit(bluetooth_uart,
                           packet,
                           BLUETOOTH_PACKET_SIZE,
                           100U);
}

HAL_StatusTypeDef Bluetooth_SendArmAck(uint8_t command,
                                      uint8_t status,
                                      uint8_t reason)
{
  uint8_t packet[BLUETOOTH_PACKET_SIZE] = {
      PACKET_HEADER,
      BLUETOOTH_MODE_ARM,
      0U,
      0U,
      0U,
      0U,
      0U,
      0U,
      0U,
      0U,
      PACKET_TAIL};

  if ((bluetooth_uart == NULL) ||
      (command > BLUETOOTH_ARM_DISABLE) ||
      (status > 1U))
  {
    return HAL_ERROR;
  }

  packet[2] = command;
  packet[3] = status;
  packet[4] = reason;
  packet[9] = CalculateChecksum(packet);

  return HAL_UART_Transmit(bluetooth_uart,
                           packet,
                           BLUETOOTH_PACKET_SIZE,
                           100U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart == bluetooth_uart)
  {
    uart_error_pending = 1U;
  }
}
