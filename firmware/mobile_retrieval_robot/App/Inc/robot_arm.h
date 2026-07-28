#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "teaching_storage.h"

#define ROBOT_ARM_JOINT_COUNT (6U)
#define ROBOT_ARM_ANGLE_OFFSET (90)
#define ROBOT_ARM_GRIPPER_INDEX (5U)

typedef enum
{
  ROBOT_ARM_OK = 0,
  ROBOT_ARM_INVALID,
  ROBOT_ARM_NOT_CALIBRATED,
  ROBOT_ARM_NOT_ENABLED,
  ROBOT_ARM_BUSY,
  ROBOT_ARM_DRIVER_ERROR
} RobotArmResult;

typedef enum
{
  ROBOT_ARM_TELEMETRY_DISABLED = 0,
  ROBOT_ARM_TELEMETRY_IDLE,
  ROBOT_ARM_TELEMETRY_MOVING,
  ROBOT_ARM_TELEMETRY_HOLDING
} RobotArmTelemetryState;

typedef struct
{
  int16_t current_values[ROBOT_ARM_JOINT_COUNT];
  int16_t target_values[ROBOT_ARM_JOINT_COUNT];
  uint16_t pulse_widths_us[ROBOT_ARM_JOINT_COUNT];
  RobotArmTelemetryState state;
  uint8_t enabled;
  uint8_t sequence_active;
  uint8_t waypoint;
} RobotArmTelemetry;

/* PCA9685를 준비한다. 부팅 직후에는 서보 위치를 임의로 바꾸지 않는다. */
HAL_StatusTypeDef RobotArm_Init(I2C_HandleTypeDef *i2c);

/* 마지막 명령 위치에서 지정한 홈 자세로 천천히 이동하며 출력을 활성화한다. */
RobotArmResult RobotArm_EnableHome(
    const uint8_t angles[ROBOT_ARM_JOINT_COUNT]);

/* 앱에서 받은 자세를 새 목표로 지정하고 제한된 속도로 이동한다. */
RobotArmResult RobotArm_SetPose(
    const uint8_t angles[ROBOT_ARM_JOINT_COUNT]);

/* 저장된 시퀀스의 첫 웨이포인트부터 비동기 재생을 시작한다. */
RobotArmResult RobotArm_Play(const TeachingSequence *sequence);

/* 진행 중인 시퀀스를 중지한다. */
void RobotArm_Stop(void);

/* 모든 서보 출력을 끄고 다시 명시적으로 활성화할 때까지 이동을 막는다. */
void RobotArm_Disable(void);

/* 관절 이동 또는 티칭 시퀀스가 진행 중이면 1을 반환한다. */
uint8_t RobotArm_IsMotionActive(void);

/*
 * 방향 보정까지 적용해 마지막으로 출력한 관절 각도와 펄스폭을 복사한다.
 * 반환값이 1이면 서보 출력이 활성화된 상태이다.
 */
uint8_t RobotArm_GetStatus(
    int16_t values[ROBOT_ARM_JOINT_COUNT],
    uint16_t pulse_widths_us[ROBOT_ARM_JOINT_COUNT]);

/* Teleplot 출력에 사용할 현재값, 목표값, 펄스폭과 동작 상태를 복사한다. */
uint8_t RobotArm_GetTelemetry(RobotArmTelemetry *telemetry);

/*
 * 일반 관절은 20 ms, Gripper는 15 ms마다 한 단계씩 이동하고
 * 티칭 시퀀스의 다음 자세를 처리한다.
 */
void RobotArm_Update(void);

#endif /* ROBOT_ARM_H */
