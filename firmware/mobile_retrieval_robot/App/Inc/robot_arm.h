#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

/* 6관절 자세 이동, 티칭 재생과 원점 보정 적용 인터페이스. */

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "teaching_storage.h"

#define ROBOT_ARM_JOINT_COUNT (6U)
#define ROBOT_ARM_ANGLE_OFFSET (90)
#define ROBOT_ARM_GRIPPER_INDEX (5U)
#define ROBOT_ARM_CALIBRATION_POINT_COUNT (3U)
#define ROBOT_ARM_ACK_SPEED_WARNING (6U)

typedef enum
{
  ROBOT_ARM_OK = 0,
  ROBOT_ARM_INVALID,
  ROBOT_ARM_NOT_CALIBRATED,
  ROBOT_ARM_NOT_ENABLED,
  ROBOT_ARM_BUSY,
  ROBOT_ARM_DRIVER_ERROR
} RobotArmResult;

/* PCA9685를 준비한다. 부팅 직후에는 서보 위치를 임의로 바꾸지 않는다. */
HAL_StatusTypeDef RobotArm_Init(I2C_HandleTypeDef *i2c);

/* 마지막 명령 위치에서 지정한 원점 자세로 천천히 이동하며 출력을 활성화한다. */
RobotArmResult RobotArm_EnableOrigin(
    const uint8_t angles[ROBOT_ARM_JOINT_COUNT]);

/* 앱에서 받은 자세를 새 목표로 지정하고 제한된 속도로 이동한다. */
RobotArmResult RobotArm_SetPose(
    const uint8_t angles[ROBOT_ARM_JOINT_COUNT]);

/*
 * 다음 이동의 속도를 50~100%로 설정한다.
 * 50~100% 범위에서 S-커브의 속도·가속도·저크 한계를 함께 조절한다.
 */
RobotArmResult RobotArm_SetSpeedPercent(uint8_t percent);

/* 저장된 시퀀스를 재생하며 요청한 경우 현재 Gripper 명령값을 유지한다. */
RobotArmResult RobotArm_Play(const TeachingSequence *sequence,
                             uint8_t preserve_gripper);

/* 진행 중인 시퀀스를 중지한다. */
void RobotArm_Stop(void);

/* 모든 서보 출력을 끄고, 실제 출력 차단 결과를 반환한다. */
RobotArmResult RobotArm_Disable(void);

/* 출력이 꺼진 동안 관절별 -90도, 0도, +90도 펄스폭을 등록한다. */
RobotArmResult RobotArm_SetCalibrations(
    const uint16_t pulse_us[ROBOT_ARM_JOINT_COUNT]
                           [ROBOT_ARM_CALIBRATION_POINT_COUNT]);

/*
 * 보정 화면에서 선택한 서보 한 개를 지정 펄스폭까지 S-커브로 이동한다.
 * request_id는 연속된 슬라이더 명령 중 최신 목표의 완료 여부를 구분한다.
 */
RobotArmResult RobotArm_PreviewPulse(uint8_t joint,
                                    uint16_t pulse_us,
                                    uint8_t speed_percent,
                                    uint8_t request_id);

/* 보정 시험 출력을 모두 끈다. */
void RobotArm_StopPreview(void);

/* 보정 미리보기 완료 또는 출력 오류 결과를 한 번만 꺼낸다. */
uint8_t RobotArm_TakePreviewResult(uint8_t *joint,
                                  uint16_t *pulse_us,
                                  uint8_t *request_id,
                                  RobotArmResult *result);

/* 서보 출력이 활성화되어 있으면 1을 반환한다. */
uint8_t RobotArm_IsEnabled(void);

/* 관절 이동, 티칭 또는 원점 보정 미리보기가 진행 중이면 1을 반환한다. */
uint8_t RobotArm_IsMotionActive(void);

/* 앱이 요청한 일반 자세에 도착했으면 한 번만 1을 반환한다. */
uint8_t RobotArm_TakePoseReached(void);

/* 해당 이동에서 설정 속도를 보장하지 못한 경우 한 번만 1을 반환한다. */
uint8_t RobotArm_TakeSpeedWarning(void);

/*
 * 모든 관절은 20 ms 고정 주기의 저크 제한 S-커브로 이동한다.
 * 시작과 목표에서 속도와 가속도를 낮춰 급격한 펄스 변화를 줄인다.
 * 티칭 시퀀스의 다음 자세도 같은 비동기 루프에서 처리한다.
 */
void RobotArm_Update(void);

#endif /* ROBOT_ARM_H */
