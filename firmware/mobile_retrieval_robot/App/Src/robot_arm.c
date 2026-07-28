#include "robot_arm.h"

#include <string.h>

#include "servo_driver.h"

#define ARM_MOVE_INTERVAL_MS (20U)
#define ARM_GRIPPER_MOVE_INTERVAL_MS (15U)
#define ARM_HOLD_TIME_MS (400U)

typedef enum { ARM_IDLE = 0, ARM_MOVING, ARM_HOLDING } ArmState;

typedef struct {
  ServoCalibration calibration;
  uint8_t min_angle;
  uint8_t max_angle;
  uint8_t reversed;
  uint8_t calibrated;
} JointConfig;

/*
 * Wrist Tilt는 수동 보정을 시작할 수 있도록 500~2500 us를 임시 사용한다.
 * 실제 끝단을 확인한 뒤 실측값으로 반드시 교체한다.
 */
static const JointConfig joint_config[ROBOT_ARM_JOINT_COUNT] = {
    {{500U, 1500U, 2500U}, 0U, 180U, 1U, 1U}, /* Base: 방향 반전 */
    {{633U, 1466U, 2322U}, 0U, 180U, 0U, 1U}, /* Shoulder */
    {{633U, 1500U, 2388U}, 0U, 180U, 1U, 1U}, /* Elbow: 방향 반전 */
    {{500U, 1500U, 2500U}, 0U, 180U, 1U, 1U}, /* Wrist Tilt: 임시, 방향 반전 */
    {{500U, 1500U, 2533U}, 0U, 180U, 0U, 1U}, /* Wrist Rotate */
    {{1140U, 1490U, 1840U}, 0U, 180U, 1U, 1U} /* Gripper: 방향 반전 */
};

/* Base 0, Shoulder -50, Elbow +90, Wrist Tilt 0, Wrist Rotate +60, Gripper 0%.
 */
static const uint8_t home_angles[ROBOT_ARM_JOINT_COUNT] = {90U, 40U,  180U,
                                                           90U, 150U, 0U};

static uint8_t current_angles[ROBOT_ARM_JOINT_COUNT];
static uint8_t target_angles[ROBOT_ARM_JOINT_COUNT];
static const TeachingSequence *active_sequence;
static uint8_t waypoint_index;
static uint8_t arm_enabled;
static uint8_t arm_initialized;
static uint32_t state_tick;
static uint32_t joint_move_ticks[ROBOT_ARM_JOINT_COUNT];
static ArmState arm_state;

static int16_t GetDisplayValue(uint8_t joint,
                               uint8_t angle,
                               uint16_t *pulse_width_us) {
  uint16_t output_angle = angle;

  if (joint_config[joint].reversed != 0U) {
    output_angle =
        (uint16_t)(joint_config[joint].max_angle -
                   (angle - joint_config[joint].min_angle));
  }

  if (pulse_width_us != NULL) {
    *pulse_width_us = ServoDriver_AngleToPulseUs(
        output_angle, &joint_config[joint].calibration);
  }

  if (joint == ROBOT_ARM_GRIPPER_INDEX) {
    return (int16_t)(((180U - output_angle) * 100U + 90U) / 180U);
  }

  return (int16_t)output_angle - ROBOT_ARM_ANGLE_OFFSET;
}

static void ResetMoveTicks(uint32_t tick) {
  uint8_t joint;

  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    joint_move_ticks[joint] = tick;
  }
}

static uint32_t GetMoveInterval(uint8_t joint) {
  return (joint == ROBOT_ARM_GRIPPER_INDEX) ? ARM_GRIPPER_MOVE_INTERVAL_MS
                                            : ARM_MOVE_INTERVAL_MS;
}

static uint8_t AreAllJointsCalibrated(void) {
  uint8_t joint;

  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    if (joint_config[joint].calibrated == 0U) {
      return 0U;
    }
  }

  return 1U;
}

static RobotArmResult
ValidatePose(const uint8_t angles[ROBOT_ARM_JOINT_COUNT]) {
  uint8_t joint;

  if (angles == NULL) {
    return ROBOT_ARM_INVALID;
  }

  if (AreAllJointsCalibrated() == 0U) {
    return ROBOT_ARM_NOT_CALIBRATED;
  }

  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    if ((angles[joint] < joint_config[joint].min_angle) ||
        (angles[joint] > joint_config[joint].max_angle)) {
      return ROBOT_ARM_INVALID;
    }
  }

  return ROBOT_ARM_OK;
}

static HAL_StatusTypeDef SetJointAngle(uint8_t joint, uint8_t angle) {
  uint16_t output_angle = angle;

  if ((joint >= ROBOT_ARM_JOINT_COUNT) ||
      (joint_config[joint].calibrated == 0U) ||
      (angle < joint_config[joint].min_angle) ||
      (angle > joint_config[joint].max_angle)) {
    return HAL_ERROR;
  }

  if (joint_config[joint].reversed != 0U) {
    output_angle = (uint16_t)(joint_config[joint].max_angle -
                              (angle - joint_config[joint].min_angle));
  }

  return ServoDriver_SetAngle(joint, output_angle,
                              &joint_config[joint].calibration);
}

static void LoadTarget(const TeachingWaypoint *waypoint) {
  memcpy(target_angles, waypoint->angles, ROBOT_ARM_JOINT_COUNT);
}

HAL_StatusTypeDef RobotArm_Init(I2C_HandleTypeDef *i2c) {
  HAL_StatusTypeDef status;
  uint8_t joint;

  arm_state = ARM_IDLE;
  arm_enabled = 0U;
  arm_initialized = 0U;
  active_sequence = NULL;
  ResetMoveTicks(HAL_GetTick());
  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    current_angles[joint] = home_angles[joint];
    target_angles[joint] = home_angles[joint];
  }

  /* 실제 위치를 읽을 수 없어 부팅 직후의 내부 위치는 홈으로 가정한다. */
  status = ServoDriver_Init(i2c);
  if (status == HAL_OK) {
    arm_initialized = 1U;
  }
  return status;
}

RobotArmResult
RobotArm_EnableHome(const uint8_t angles[ROBOT_ARM_JOINT_COUNT]) {
  RobotArmResult result;
  uint8_t joint;

  result = ValidatePose(angles);
  if (result != ROBOT_ARM_OK) {
    return result;
  }

  RobotArm_Stop();

  /*
   * 출력이 꺼지기 전의 마지막 명령 위치부터 다시 시작한다.
   * 부팅 직후에는 위치 피드백이 없어 홈 위치를 시작점으로 가정한다.
   */
  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    if (SetJointAngle(joint, current_angles[joint]) != HAL_OK) {
      RobotArm_Disable();
      return ROBOT_ARM_DRIVER_ERROR;
    }
  }

  arm_enabled = 1U;
  memcpy(target_angles, angles, sizeof(target_angles));
  state_tick = HAL_GetTick();
  ResetMoveTicks(state_tick);
  arm_state = ARM_MOVING;
  return ROBOT_ARM_OK;
}

RobotArmResult RobotArm_SetPose(const uint8_t angles[ROBOT_ARM_JOINT_COUNT]) {
  RobotArmResult result;
  if (arm_enabled == 0U) {
    return ROBOT_ARM_NOT_ENABLED;
  }

  result = ValidatePose(angles);
  if (result != ROBOT_ARM_OK) {
    return result;
  }

  active_sequence = NULL;
  memcpy(target_angles, angles, sizeof(target_angles));

  /*
   * 슬라이더 패킷이 연속으로 와도 기존 이동 시간을 유지해야
   * RobotArm_Update가 일정한 속도로 계속 움직일 수 있다.
   */
  if (arm_state != ARM_MOVING) {
    state_tick = HAL_GetTick();
    ResetMoveTicks(state_tick);
  }
  arm_state = ARM_MOVING;

  return ROBOT_ARM_OK;
}

RobotArmResult RobotArm_Play(const TeachingSequence *sequence) {
  RobotArmResult result;
  uint8_t waypoint;

  if (arm_enabled == 0U) {
    return ROBOT_ARM_NOT_ENABLED;
  }

  if ((sequence == NULL) || (sequence->count == 0U)) {
    return ROBOT_ARM_INVALID;
  }

  if (arm_state != ARM_IDLE) {
    return ROBOT_ARM_BUSY;
  }

  for (waypoint = 0U; waypoint < sequence->count; waypoint++) {
    result = ValidatePose(sequence->waypoints[waypoint].angles);
    if (result != ROBOT_ARM_OK) {
      return result;
    }
  }

  active_sequence = sequence;
  waypoint_index = 0U;
  LoadTarget(&active_sequence->waypoints[0]);
  state_tick = HAL_GetTick();
  ResetMoveTicks(state_tick);
  arm_state = ARM_MOVING;
  return ROBOT_ARM_OK;
}

void RobotArm_Stop(void) {
  arm_state = ARM_IDLE;
  active_sequence = NULL;
}

void RobotArm_Disable(void) {
  RobotArm_Stop();
  arm_enabled = 0U;
  (void)ServoDriver_DisableAll();
}

uint8_t RobotArm_IsMotionActive(void) {
  return (uint8_t)((arm_enabled != 0U) && (arm_state != ARM_IDLE));
}

uint8_t RobotArm_GetStatus(int16_t values[ROBOT_ARM_JOINT_COUNT],
                           uint16_t pulse_widths_us[ROBOT_ARM_JOINT_COUNT]) {
  uint8_t joint;

  if ((values == NULL) || (pulse_widths_us == NULL)) {
    return 0U;
  }

  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    values[joint] = GetDisplayValue(
        joint, current_angles[joint], &pulse_widths_us[joint]);
  }

  return arm_enabled;
}

uint8_t RobotArm_GetTelemetry(RobotArmTelemetry *telemetry) {
  uint8_t joint;

  if ((telemetry == NULL) || (arm_initialized == 0U)) {
    return 0U;
  }

  telemetry->enabled = arm_enabled;
  telemetry->sequence_active = (active_sequence != NULL) ? 1U : 0U;
  telemetry->waypoint =
      (active_sequence != NULL) ? (uint8_t)(waypoint_index + 1U) : 0U;

  if (arm_enabled == 0U) {
    telemetry->state = ROBOT_ARM_TELEMETRY_DISABLED;
  } else if (arm_state == ARM_MOVING) {
    telemetry->state = ROBOT_ARM_TELEMETRY_MOVING;
  } else if (arm_state == ARM_HOLDING) {
    telemetry->state = ROBOT_ARM_TELEMETRY_HOLDING;
  } else {
    telemetry->state = ROBOT_ARM_TELEMETRY_IDLE;
  }

  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    telemetry->current_values[joint] = GetDisplayValue(
        joint, current_angles[joint], &telemetry->pulse_widths_us[joint]);
    telemetry->target_values[joint] = GetDisplayValue(
        joint, target_angles[joint], NULL);

    if (arm_enabled == 0U) {
      telemetry->pulse_widths_us[joint] = 0U;
    }
  }

  return 1U;
}

void RobotArm_Update(void) {
  uint32_t now = HAL_GetTick();
  uint32_t move_interval;
  uint8_t joint;
  uint8_t target_pending = 0U;

  if ((arm_enabled == 0U) || (arm_state == ARM_IDLE)) {
    return;
  }

  if (arm_state == ARM_MOVING) {
    for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
      if (current_angles[joint] == target_angles[joint]) {
        continue;
      }

      target_pending = 1U;
      move_interval = GetMoveInterval(joint);
      if ((now - joint_move_ticks[joint]) < move_interval) {
        continue;
      }
      joint_move_ticks[joint] = now;

      if (current_angles[joint] < target_angles[joint]) {
        current_angles[joint]++;
      } else {
        current_angles[joint]--;
      }

      if (SetJointAngle(joint, current_angles[joint]) != HAL_OK) {
        RobotArm_Disable();
        return;
      }
    }

    if (target_pending == 0U) {
      if (active_sequence == NULL) {
        RobotArm_Stop();
      } else {
        arm_state = ARM_HOLDING;
        state_tick = now;
      }
    }
  } else if ((arm_state == ARM_HOLDING) &&
             ((now - state_tick) >= ARM_HOLD_TIME_MS)) {
    waypoint_index++;
    if ((active_sequence != NULL) &&
        (waypoint_index < active_sequence->count)) {
      LoadTarget(&active_sequence->waypoints[waypoint_index]);
      arm_state = ARM_MOVING;
      state_tick = now;
      ResetMoveTicks(now);
    } else {
      RobotArm_Stop();
    }
  }
}
