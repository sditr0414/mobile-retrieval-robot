#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#define BLUETOOTH_PACKET_SIZE (11U)
#define BLUETOOTH_DATA_SIZE   (7U)

#define BLUETOOTH_MODE_DRIVE    (0U)
#define BLUETOOTH_MODE_ARM      (1U)
#define BLUETOOTH_MODE_TEACHING (2U)

#define BLUETOOTH_DRIVE_NORMAL      (0U)
#define BLUETOOTH_DRIVE_ESTOP       (1U)
#define BLUETOOTH_DRIVE_ESTOP_CLEAR (2U)

#define BLUETOOTH_ARM_MOVE        (0U)
#define BLUETOOTH_ARM_ENABLE_HOME (1U)
#define BLUETOOTH_ARM_DISABLE     (2U)

/* 앱에서 받은 좌우 모터 명령과 수신 시각을 전달한다. */
typedef struct
{
  uint8_t left_direction;
  uint8_t left_pwm;
  uint8_t right_direction;
  uint8_t right_pwm;
  uint8_t control;
  uint32_t received_tick;
} BluetoothDriveCommand;

/* 로봇팔과 티칭 명령의 모드 및 데이터 7바이트를 전달한다. */
typedef struct
{
  uint8_t mode;
  uint8_t data[BLUETOOTH_DATA_SIZE];
} BluetoothArmCommand;

/* USART RX circular DMA를 시작한다. */
HAL_StatusTypeDef Bluetooth_Init(UART_HandleTypeDef *uart);

/* DMA 버퍼의 새 바이트를 읽어 유효한 명령을 큐로 보낸다. */
void Bluetooth_Process(osMessageQueueId_t arm_queue,
                       osMessageQueueId_t drive_queue);

/* 티칭 처리 결과를 앱의 ACK 형식으로 전송한다. */
HAL_StatusTypeDef Bluetooth_SendTeachingAck(uint8_t sequence_id,
                                            uint8_t status);

/* 로봇팔 활성화와 비활성화 처리 결과를 앱에 알린다. */
HAL_StatusTypeDef Bluetooth_SendArmAck(uint8_t command,
                                      uint8_t status,
                                      uint8_t reason);

#endif /* BLUETOOTH_H */
