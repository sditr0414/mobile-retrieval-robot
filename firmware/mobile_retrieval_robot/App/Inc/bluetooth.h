#ifndef BLUETOOTH_H
#define BLUETOOTH_H

/* HC-05 16바이트 프로토콜의 공용 상수, 큐 메시지와 ACK 인터페이스. */

#include <stdint.h>

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#define BLUETOOTH_PACKET_SIZE (16U)
#define BLUETOOTH_DATA_SIZE   (12U)

#define BLUETOOTH_MODE_DRIVE    (0U)
#define BLUETOOTH_MODE_ARM      (1U)
#define BLUETOOTH_MODE_TEACHING (2U)
#define BLUETOOTH_MODE_SETTINGS (3U)

#define BLUETOOTH_DRIVE_NORMAL      (0U)
#define BLUETOOTH_DRIVE_ESTOP       (1U)
#define BLUETOOTH_DRIVE_ESTOP_CLEAR (2U)
#define BLUETOOTH_DRIVE_STATUS_PID  (3U)
#define BLUETOOTH_DRIVE_STATUS_IMU  (4U)

#define BLUETOOTH_ARM_MOVE        (0U)
#define BLUETOOTH_ARM_ENABLE_ORIGIN (1U)
#define BLUETOOTH_ARM_DISABLE     (2U)
#define BLUETOOTH_ARM_TRAVEL_KEEP_GRIPPER (3U)

#define BLUETOOTH_TEACHING_GET_NAME (5U)
#define BLUETOOTH_TEACHING_GET_SEQUENCE (6U)
#define BLUETOOTH_TEACHING_PREVIEW      (7U)

#define BLUETOOTH_SETTINGS_GET        (1U)
#define BLUETOOTH_SETTINGS_WRITE_PART (2U)
#define BLUETOOTH_SETTINGS_COMMIT     (3U)
#define BLUETOOTH_SETTINGS_READ_PART  (4U)
#define BLUETOOTH_SETTINGS_PREVIEW    (5U)
#define BLUETOOTH_SETTINGS_PREVIEW_STOP (6U)
#define BLUETOOTH_SETTINGS_PID_ENABLE (7U)
#define BLUETOOTH_SETTINGS_IMU_CALIBRATE (8U)

/* 앱에서 받은 좌우 모터 명령과 수신 시각을 전달한다. */
typedef struct
{
  uint8_t left_direction;
  uint8_t left_pwm;
  uint8_t right_direction;
  uint8_t right_pwm;
  uint8_t control;
  uint8_t engine_enabled;
  uint8_t pid_straight;
  uint8_t refresh_yaw_target;
  uint32_t received_tick;
} BluetoothDriveCommand;

/* 로봇팔, 티칭과 설정 명령의 모드 및 데이터 12바이트를 전달한다. */
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

/* 티칭 저장 또는 초기화 결과를 앱의 ACK 형식으로 전송한다. */
HAL_StatusTypeDef Bluetooth_SendTeachingAck(uint8_t command,
                                            uint8_t sequence_id,
                                            uint8_t status,
                                            uint8_t speed_warning);

/* 저장된 UTF-8 시퀀스 이름을 7바이트 조각으로 전송한다. */
HAL_StatusTypeDef Bluetooth_SendTeachingNamePart(uint8_t sequence_id,
                                                 uint8_t part,
                                                 uint8_t name_length,
                                                 uint8_t request_id,
                                                 const uint8_t data[7]);

/* 저장된 시퀀스의 웨이포인트 수를 조회 응답으로 전송한다. */
HAL_StatusTypeDef Bluetooth_SendTeachingSequenceMeta(uint8_t sequence_id,
                                                     uint8_t waypoint_count,
                                                     uint8_t request_id);

/* 저장된 웨이포인트 한 개를 조회 응답으로 전송한다. */
HAL_StatusTypeDef Bluetooth_SendTeachingWaypoint(uint8_t sequence_id,
                                                 uint8_t waypoint_index,
                                                 const uint8_t angles[6],
                                                 uint8_t request_id);

/* E-STOP 설정 또는 해제 결과를 앱에 알린다. */
HAL_StatusTypeDef Bluetooth_SendDriveAck(uint8_t control,
                                        uint8_t status,
                                        uint8_t reason);

/*
 * 로봇팔 명령 결과를 앱에 알린다.
 * transaction_id는 지연된 이전 ACK를 새 이동의 결과로 오인하지 않게 한다.
 */
HAL_StatusTypeDef Bluetooth_SendArmAck(uint8_t command,
                                      uint8_t status,
                                      uint8_t reason,
                                      uint8_t transaction_id);

/* 설정 8바이트 조각과 현재 PID 적용 상태를 앱에 전송한다. */
HAL_StatusTypeDef Bluetooth_SendSettingsPart(uint8_t part,
                                             uint8_t pid_applied,
                                             const uint8_t data[8],
                                             uint8_t request_id);

/* 설정 저장 결과와 현재 PID 적용 상태를 앱에 전송한다. */
HAL_StatusTypeDef Bluetooth_SendSettingsAck(uint8_t status,
                                            uint8_t reason,
                                            uint8_t pid_applied);

/* 최신 서보 보정 미리보기가 목표 펄스에 도착했는지 앱에 알린다. */
HAL_StatusTypeDef Bluetooth_SendPreviewResult(uint8_t joint,
                                              uint8_t status,
                                              uint8_t reason,
                                              uint16_t pulse_us,
                                              uint8_t request_id);

/* 방향 PID 활성화/비활성화 요청 결과와 실제 적용 상태를 전송한다. */
HAL_StatusTypeDef Bluetooth_SendPidAck(uint8_t status,
                                       uint8_t pid_applied,
                                       uint8_t reason,
                                       uint8_t request_id);

/* MPU6050 영점 보정 완료 여부를 요청 ID와 함께 전송한다. */
HAL_StatusTypeDef Bluetooth_SendImuCalibrationAck(uint8_t status,
                                                  uint8_t reason,
                                                  uint8_t request_id);

/* 방향 PID의 실제 실행 상태를 고정 16바이트 상태 패킷으로 전송한다. */
HAL_StatusTypeDef Bluetooth_SendPidStatus(uint8_t flags,
                                         int16_t target_yaw_cdeg,
                                         int16_t current_yaw_cdeg,
                                         int16_t error_cdeg,
                                         int16_t output_centi,
                                         uint8_t left_pwm,
                                         uint8_t right_pwm);

/* MPU6050 상태, 온도와 필터된 Z축 값을 앱에 전송한다. */
HAL_StatusTypeDef Bluetooth_SendImuStatus(uint8_t flags,
                                         int16_t temperature_centi_c,
                                         int16_t gyro_z_centi_dps,
                                         int16_t yaw_cdeg,
                                         uint32_t error_count);

#endif /* BLUETOOTH_H */
