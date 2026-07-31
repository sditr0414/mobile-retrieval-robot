#ifndef TEACHING_STORAGE_H
#define TEACHING_STORAGE_H

/* 티칭 시퀀스와 중요 설정의 Sector 7 영구 저장 인터페이스. */

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define TEACHING_SEQUENCE_COUNT       (12U)
#define TEACHING_MAX_WAYPOINTS        (30U)
#define TEACHING_JOINT_COUNT          (6U)
#define TEACHING_NAME_MAX_BYTES       (21U)
#define TEACHING_NAME_STORAGE_SIZE    (TEACHING_NAME_MAX_BYTES + 1U)
#define ROBOT_SETTINGS_SERVO_COUNT    (6U)
#define ROBOT_SETTINGS_PULSE_COUNT    (3U)
#define ROBOT_SETTINGS_TRAVEL_JOINT_COUNT (5U)

typedef struct
{
  uint8_t angles[TEACHING_JOINT_COUNT];
} TeachingWaypoint;

typedef struct
{
  TeachingWaypoint waypoints[TEACHING_MAX_WAYPOINTS];
  uint8_t count;
  uint8_t name_length;
  uint8_t name[TEACHING_NAME_STORAGE_SIZE];
} TeachingSequence;

typedef struct
{
  /* PID 값은 소수점 셋째 자리까지 보존하기 위해 1000배 정수로 저장한다. */
  int32_t pid_kp_milli;
  int32_t pid_ki_milli;
  int32_t pid_kd_milli;
  /*
   * 관절별 -90도, 0도, +90도 펄스폭이다.
   * Gripper의 가운데 값은 열림과 닫힘 값의 평균으로 자동 계산한다.
   */
  uint16_t servo_pulse_us[ROBOT_SETTINGS_SERVO_COUNT]
                         [ROBOT_SETTINGS_PULSE_COUNT];
  /*
   * 차량 주행 중 사용할 Base~Wrist Rotate 자세다.
   * Bluetooth 패킷과 동일하게 -90~+90도를 0~180으로 저장한다.
   */
  uint8_t travel_pose_angles[ROBOT_SETTINGS_TRAVEL_JOINT_COUNT];
} RobotSettings;

/* Flash 형식과 CRC가 맞으면 시퀀스와 설정을 RAM으로 불러온다. */
void TeachingStorage_Init(void);

/* 앱이 보낼 시퀀스의 번호와 웨이포인트 수를 먼저 등록한다. */
HAL_StatusTypeDef TeachingStorage_BeginUpload(uint8_t sequence_id,
                                              uint8_t waypoint_count,
                                              uint8_t name_length);

/* 한 웨이포인트의 앞쪽 또는 뒤쪽 각도 3개를 임시 버퍼에 넣는다. */
HAL_StatusTypeDef TeachingStorage_WriteHalf(uint8_t sequence_id,
                                            uint8_t waypoint_index,
                                            uint8_t second_half,
                                            const uint8_t angles[3]);

/* 시퀀스 이름의 8바이트 조각을 임시 버퍼에 넣는다. */
HAL_StatusTypeDef TeachingStorage_WriteNamePart(uint8_t sequence_id,
                                                uint8_t part,
                                                const uint8_t name[8]);

/* 완전한 임시 시퀀스를 RAM과 Flash에 저장하고 다시 읽어 검증한다. */
HAL_StatusTypeDef TeachingStorage_Commit(uint8_t sequence_id);

/* Flash에 쓰지 않고 완전한 임시 시퀀스를 미리보기 재생에 제공한다. */
const TeachingSequence *TeachingStorage_GetUpload(uint8_t sequence_id);

/* 지정한 시퀀스를 비우고 Flash에 반영한다. */
HAL_StatusTypeDef TeachingStorage_Reset(uint8_t sequence_id);

/* 재생할 시퀀스를 읽기 전용 포인터로 반환한다. */
const TeachingSequence *TeachingStorage_Get(uint8_t sequence_id);

/* 현재 PID 및 서보 3점 펄스 보정값을 복사한다. */
void TeachingStorage_GetSettings(RobotSettings *settings);

/* 서보 보정값만 펌웨어 기본값으로 되돌리고 PID와 티칭값은 보존한다. */
void TeachingStorage_ResetServoCalibrations(void);

/* 설정을 시퀀스와 함께 Sector 7에 저장하고 다시 읽어 검증한다. */
HAL_StatusTypeDef TeachingStorage_SaveSettings(const RobotSettings *settings);

#endif /* TEACHING_STORAGE_H */
