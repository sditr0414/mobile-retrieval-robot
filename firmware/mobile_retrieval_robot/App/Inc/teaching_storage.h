#ifndef TEACHING_STORAGE_H
#define TEACHING_STORAGE_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define TEACHING_SEQUENCE_COUNT       (12U)
#define TEACHING_MAX_WAYPOINTS        (30U)
#define TEACHING_JOINT_COUNT          (6U)

typedef struct
{
  uint8_t angles[TEACHING_JOINT_COUNT];
} TeachingWaypoint;

typedef struct
{
  TeachingWaypoint waypoints[TEACHING_MAX_WAYPOINTS];
  uint8_t count;
} TeachingSequence;

/* Flash의 형식과 CRC가 맞으면 12개 시퀀스를 RAM으로 불러온다. */
void TeachingStorage_Init(void);

/* 앱이 보낼 시퀀스의 번호와 웨이포인트 수를 먼저 등록한다. */
HAL_StatusTypeDef TeachingStorage_BeginUpload(uint8_t sequence_id,
                                              uint8_t waypoint_count);

/* 한 웨이포인트의 앞쪽 또는 뒤쪽 각도 3개를 임시 버퍼에 넣는다. */
HAL_StatusTypeDef TeachingStorage_WriteHalf(uint8_t sequence_id,
                                            uint8_t waypoint_index,
                                            uint8_t second_half,
                                            const uint8_t angles[3]);

/* 완전한 임시 시퀀스를 RAM과 Flash에 저장하고 다시 읽어 검증한다. */
HAL_StatusTypeDef TeachingStorage_Commit(uint8_t sequence_id);

/* 지정한 시퀀스를 비우고 Flash에 반영한다. */
HAL_StatusTypeDef TeachingStorage_Reset(uint8_t sequence_id);

/* 재생할 시퀀스를 읽기 전용 포인터로 반환한다. */
const TeachingSequence *TeachingStorage_Get(uint8_t sequence_id);

#endif /* TEACHING_STORAGE_H */
