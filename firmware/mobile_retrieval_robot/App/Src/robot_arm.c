#include "robot_arm.h"

#include <string.h>

#include "servo_driver.h"

/*
 * 6개 관절의 명령 각도, 목표 각도와 티칭 재생 상태를 관리한다.
 * 위치 센서가 없으므로 current_angles는 실측값이 아니라 마지막 PWM 명령값이다.
 */

#define ARM_CONTROL_INTERVAL_MS (20U)
#define ARM_REFERENCE_STEP_INTERVAL_MS (16U)
#define ARM_RAMP_STEPS (12U)
#define ARM_TRAJECTORY_SAMPLE_COUNT (32U)
#define ARM_TRAJECTORY_SEARCH_COUNT (32U)
#define ARM_HOLD_TIME_MS (400U)
#define ARM_CALIBRATION_MIN_US (350U)
#define ARM_CALIBRATION_MAX_US (2650U)
#define ARM_GRIPPER_CALIBRATION_MIN_US (1000U)
#define ARM_GRIPPER_CALIBRATION_MAX_US (2000U)
#define ARM_PREVIEW_TIMEOUT_MS (2000U)
#define ARM_SPEED_MIN_PERCENT (50U)
#define ARM_SPEED_MAX_PERCENT (100U)

typedef enum { ARM_IDLE = 0, ARM_MOVING, ARM_HOLDING } ArmState;

typedef struct {
  ServoCalibration calibration;
  uint8_t min_angle;
  uint8_t max_angle;
  uint8_t reversed;
  uint8_t calibrated;
} JointConfig;

/*
 * Flash 보정값이 없을 때는 좁힌 안전 초기 범위를 사용한다.
 * 조립 후 앱의 관절별 보정 화면에서 실제 세 지점을 다시 저장한다.
 */
static JointConfig joint_config[ROBOT_ARM_JOINT_COUNT] = {
    {{700U, 1500U, 2300U}, 0U, 180U, 1U, 1U}, /* Base: 방향 반전 */
    {{700U, 1500U, 2300U}, 0U, 180U, 1U, 1U}, /* Shoulder: 방향 반전 */
    {{700U, 1500U, 2300U}, 0U, 180U, 0U, 1U}, /* Elbow */
    {{700U, 1500U, 2300U}, 0U, 180U, 1U, 1U}, /* Wrist Tilt: 방향 반전 */
    {{700U, 1500U, 2300U}, 0U, 180U, 1U, 1U}, /* Wrist Rotate: 방향 반전 */
    {{1200U, 1500U, 1800U}, 0U, 180U, 1U, 1U} /* Gripper: 방향 반전 */
};

/* 모든 일반 관절은 표시 0도, Gripper는 최대 열림 0%를 홈으로 사용한다. */
static const uint8_t origin_angles[ROBOT_ARM_JOINT_COUNT] = {
    90U, 90U, 90U, 90U, 90U, 0U};

static uint8_t current_angles[ROBOT_ARM_JOINT_COUNT];
static uint8_t target_angles[ROBOT_ARM_JOINT_COUNT];
static const TeachingSequence *active_sequence;
static uint8_t waypoint_index;
static uint8_t arm_enabled;
static uint8_t arm_initialized;
static uint8_t pose_reached_pending;
static uint8_t speed_warning_pending;
static uint8_t preserve_sequence_gripper;
static uint8_t speed_percent = ARM_SPEED_MAX_PERCENT;
static uint8_t preview_active;
static uint8_t preview_moving;
static uint8_t preview_joint;
static uint8_t preview_position_valid;
static uint8_t preview_position_joint;
static uint8_t preview_request_id;
static uint8_t preview_result_pending;
static RobotArmResult preview_result;
static uint16_t preview_target_pulse_us;
static float preview_position_us;
static float preview_velocity_us_s;
static float preview_acceleration_us_s2;
static float preview_coefficients[6];
static float preview_duration_s;
static float preview_elapsed_s;
static uint32_t preview_command_tick;
static uint32_t preview_motion_tick;
static uint32_t state_tick;
static uint32_t motion_tick;
static float motion_positions[ROBOT_ARM_JOINT_COUNT];
static float motion_velocities[ROBOT_ARM_JOINT_COUNT];
static float motion_accelerations[ROBOT_ARM_JOINT_COUNT];
static float trajectory_coefficients[ROBOT_ARM_JOINT_COUNT][6];
static float trajectory_duration_s;
static float trajectory_elapsed_s;
static uint16_t last_pulse_widths_us[ROBOT_ARM_JOINT_COUNT];
static ArmState arm_state;

static float AbsFloat(float value) {
  return (value < 0.0F) ? -value : value;
}

/* 조립 방향이 반전된 관절의 논리 위치를 실제 출력 방향으로 바꾼다. */
static float GetOutputPosition(uint8_t joint, float position) {
  if (joint_config[joint].reversed != 0U) {
    return (float)joint_config[joint].max_angle -
           (position - (float)joint_config[joint].min_angle);
  }

  return position;
}

/*
 * 3점 보정값 사이를 소수점 위치로 보간한다.
 * 1도 단위로 끊지 않아 낮은 속도에서도 PCA9685 펄스가 고르게 변한다.
 */
static uint16_t GetPulseWidthForPosition(uint8_t joint, float position) {
  const ServoCalibration *calibration = &joint_config[joint].calibration;
  float output_position = GetOutputPosition(joint, position);
  float pulse_width;

  if (output_position <= 90.0F) {
    pulse_width =
        (float)calibration->min_pulse_us +
        (((float)calibration->center_pulse_us -
          (float)calibration->min_pulse_us) *
         (output_position / 90.0F));
  } else {
    pulse_width =
        (float)calibration->center_pulse_us +
        (((float)calibration->max_pulse_us -
          (float)calibration->center_pulse_us) *
         ((output_position - 90.0F) / 90.0F));
  }

  return (uint16_t)(pulse_width + 0.5F);
}

/* 이동을 멈출 때 속도와 가속도만 0으로 만든다. */
static void StopMotion(void) {
  uint8_t joint;

  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    motion_velocities[joint] = 0.0F;
    motion_accelerations[joint] = 0.0F;
  }
}

/* 6개 관절에 유효한 3점 보정값이 모두 등록됐는지 확인한다. */
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

/* 한 관절의 소수점 위치를 펄스폭으로 바꿔 PCA9685 채널에 출력한다. */
static HAL_StatusTypeDef SetJointPosition(uint8_t joint, float position) {
  HAL_StatusTypeDef status;
  uint16_t pulse_width_us;
  uint8_t rounded_angle;

  if ((joint >= ROBOT_ARM_JOINT_COUNT) ||
      (joint_config[joint].calibrated == 0U) ||
      (position < (float)joint_config[joint].min_angle) ||
      (position > (float)joint_config[joint].max_angle)) {
    return HAL_ERROR;
  }

  pulse_width_us = GetPulseWidthForPosition(joint, position);
  if (pulse_width_us != last_pulse_widths_us[joint]) {
    status = ServoDriver_SetPulseUs(joint, pulse_width_us);
    if (status != HAL_OK) {
      return status;
    }
    last_pulse_widths_us[joint] = pulse_width_us;
  }

  motion_positions[joint] = position;
  rounded_angle = (uint8_t)(position + 0.5F);
  current_angles[joint] = rounded_angle;
  return HAL_OK;
}

/* 현재 상태와 목표점의 속도·가속도가 이어지는 5차 다항식 계수를 만든다. */
static void BuildTrajectoryCoefficients(uint8_t joint,
                                        float duration,
                                        float coefficients[6]) {
  float duration_2 = duration * duration;
  float duration_3 = duration_2 * duration;
  float duration_4 = duration_3 * duration;
  float duration_5 = duration_4 * duration;
  float distance = (float)target_angles[joint] - motion_positions[joint];
  float velocity = motion_velocities[joint];
  float acceleration = motion_accelerations[joint];

  coefficients[0] = motion_positions[joint];
  coefficients[1] = velocity;
  coefficients[2] = acceleration * 0.5F;
  coefficients[3] =
      ((20.0F * distance) - (12.0F * velocity * duration) -
       (3.0F * acceleration * duration_2)) /
      (2.0F * duration_3);
  coefficients[4] =
      ((-30.0F * distance) + (16.0F * velocity * duration) +
       (3.0F * acceleration * duration_2)) /
      (2.0F * duration_4);
  coefficients[5] =
      ((12.0F * distance) - (6.0F * velocity * duration) -
       (acceleration * duration_2)) /
      (2.0F * duration_5);
}

/* 5차 궤적의 위치, 속도, 가속도와 저크를 한 시점에서 계산한다. */
static void EvaluateTrajectory(const float coefficients[6],
                               float time,
                               float *position,
                               float *velocity,
                               float *acceleration,
                               float *jerk) {
  float time_2 = time * time;
  float time_3 = time_2 * time;
  float time_4 = time_3 * time;
  float time_5 = time_4 * time;

  *position = coefficients[0] + (coefficients[1] * time) +
              (coefficients[2] * time_2) +
              (coefficients[3] * time_3) +
              (coefficients[4] * time_4) +
              (coefficients[5] * time_5);
  *velocity = coefficients[1] + (2.0F * coefficients[2] * time) +
              (3.0F * coefficients[3] * time_2) +
              (4.0F * coefficients[4] * time_3) +
              (5.0F * coefficients[5] * time_4);
  *acceleration = (2.0F * coefficients[2]) +
                  (6.0F * coefficients[3] * time) +
                  (12.0F * coefficients[4] * time_2) +
                  (20.0F * coefficients[5] * time_3);
  *jerk = (6.0F * coefficients[3]) +
          (24.0F * coefficients[4] * time) +
          (60.0F * coefficients[5] * time_2);
}

/* 현재 펄스 상태에서 새 보정 목표로 이어지는 5차 다항식 계수를 만든다. */
static void BuildPreviewCoefficients(float duration, float coefficients[6]) {
  float duration_2 = duration * duration;
  float duration_3 = duration_2 * duration;
  float duration_4 = duration_3 * duration;
  float duration_5 = duration_4 * duration;
  float distance = (float)preview_target_pulse_us - preview_position_us;

  coefficients[0] = preview_position_us;
  coefficients[1] = preview_velocity_us_s;
  coefficients[2] = preview_acceleration_us_s2 * 0.5F;
  coefficients[3] =
      ((20.0F * distance) - (12.0F * preview_velocity_us_s * duration) -
       (3.0F * preview_acceleration_us_s2 * duration_2)) /
      (2.0F * duration_3);
  coefficients[4] =
      ((-30.0F * distance) + (16.0F * preview_velocity_us_s * duration) +
       (3.0F * preview_acceleration_us_s2 * duration_2)) /
      (2.0F * duration_4);
  coefficients[5] =
      ((12.0F * distance) - (6.0F * preview_velocity_us_s * duration) -
       (preview_acceleration_us_s2 * duration_2)) /
      (2.0F * duration_5);
}

/* 후보 보정 궤적이 펄스 범위와 속도·가속도·저크 한계를 지키는지 검사한다. */
static uint8_t IsPreviewTrajectorySafe(float duration,
                                       float maximum_velocity,
                                       float maximum_acceleration,
                                       float maximum_jerk,
                                       uint16_t minimum,
                                       uint16_t maximum) {
  float coefficients[6];
  float position;
  float velocity;
  float acceleration;
  float jerk;
  float velocity_limit = maximum_velocity;
  float acceleration_limit = maximum_acceleration;
  float time;
  uint8_t sample;

  if (AbsFloat(preview_velocity_us_s) > velocity_limit) {
    velocity_limit = AbsFloat(preview_velocity_us_s);
  }
  if (AbsFloat(preview_acceleration_us_s2) > acceleration_limit) {
    acceleration_limit = AbsFloat(preview_acceleration_us_s2);
  }

  BuildPreviewCoefficients(duration, coefficients);
  for (sample = 0U; sample <= ARM_TRAJECTORY_SAMPLE_COUNT; sample++) {
    time = duration * (float)sample / (float)ARM_TRAJECTORY_SAMPLE_COUNT;
    EvaluateTrajectory(coefficients,
                       time,
                       &position,
                       &velocity,
                       &acceleration,
                       &jerk);
    if ((((sample != 0U) && (sample != ARM_TRAJECTORY_SAMPLE_COUNT)) &&
         ((position < (float)minimum) || (position > (float)maximum))) ||
        (AbsFloat(velocity) > velocity_limit) ||
        (AbsFloat(acceleration) > acceleration_limit) ||
        (AbsFloat(jerk) > maximum_jerk)) {
      return 0U;
    }
  }

  return 1U;
}

/*
 * 일반 각도 제어와 같은 한계를 펄스 단위로 환산해 보정 이동을 계획한다.
 * 보정 중인 서보의 전체 펄스 범위를 180도 회전에 대응시킨다.
 */
static uint8_t PlanPreviewTrajectory(uint8_t requested_speed_percent,
                                     uint16_t minimum,
                                     uint16_t maximum) {
  const float time_step = (float)ARM_CONTROL_INTERVAL_MS / 1000.0F;
  const float ramp_time = (float)(ARM_RAMP_STEPS * ARM_CONTROL_INTERVAL_MS) /
                          1000.0F;
  const float speed_scale = (float)requested_speed_percent / 100.0F;
  const float pulse_per_degree =
      ((float)joint_config[preview_joint].calibration.max_pulse_us -
       (float)joint_config[preview_joint].calibration.min_pulse_us) /
      180.0F;
  const float maximum_velocity =
      (1000.0F / (float)ARM_REFERENCE_STEP_INTERVAL_MS) *
      speed_scale * pulse_per_degree;
  const float maximum_acceleration = maximum_velocity / ramp_time;
  const float maximum_jerk = maximum_acceleration / ramp_time;
  float duration = time_step;
  uint8_t attempt;

  for (attempt = 0U; attempt < ARM_TRAJECTORY_SEARCH_COUNT; attempt++) {
    if (IsPreviewTrajectorySafe(duration,
                                maximum_velocity,
                                maximum_acceleration,
                                maximum_jerk,
                                minimum,
                                maximum) != 0U) {
      BuildPreviewCoefficients(duration, preview_coefficients);
      preview_duration_s = duration;
      preview_elapsed_s = 0.0F;
      return 1U;
    }
    duration *= 1.25F;
  }

  /* 급격한 방향 전환에서는 현재 펄스에서 정지한 뒤 안전 궤적을 다시 찾는다. */
  preview_velocity_us_s = 0.0F;
  preview_acceleration_us_s2 = 0.0F;
  duration = time_step;
  for (attempt = 0U; attempt < ARM_TRAJECTORY_SEARCH_COUNT; attempt++) {
    if (IsPreviewTrajectorySafe(duration,
                                maximum_velocity,
                                maximum_acceleration,
                                maximum_jerk,
                                minimum,
                                maximum) != 0U) {
      BuildPreviewCoefficients(duration, preview_coefficients);
      preview_duration_s = duration;
      preview_elapsed_s = 0.0F;
      return 1U;
    }
    duration *= 1.25F;
  }

  return 0U;
}

/*
 * 모든 관절이 같은 시간에 끝나도록 공통 지속 시간을 찾는다.
 * 후보 궤적을 표본 검사해 위치 범위와 속도·가속도·저크 한계를 모두 지킨다.
 */
static uint8_t IsTrajectorySafe(float duration,
                                float maximum_velocity,
                                float maximum_acceleration,
                                float maximum_jerk) {
  float coefficients[6];
  float position;
  float velocity;
  float acceleration;
  float jerk;
  float time;
  float velocity_limit;
  float acceleration_limit;
  uint8_t joint;
  uint8_t sample;

  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    BuildTrajectoryCoefficients(joint, duration, coefficients);
    velocity_limit = maximum_velocity;
    acceleration_limit = maximum_acceleration;

    /* 이동 중 속도를 낮춘 경우 현재 값부터 연속적으로 감속할 수 있게 한다. */
    if (AbsFloat(motion_velocities[joint]) > velocity_limit) {
      velocity_limit = AbsFloat(motion_velocities[joint]);
    }
    if (AbsFloat(motion_accelerations[joint]) > acceleration_limit) {
      acceleration_limit = AbsFloat(motion_accelerations[joint]);
    }

    for (sample = 0U; sample <= ARM_TRAJECTORY_SAMPLE_COUNT; sample++) {
      time = duration * (float)sample /
             (float)ARM_TRAJECTORY_SAMPLE_COUNT;
      EvaluateTrajectory(coefficients,
                         time,
                         &position,
                         &velocity,
                         &acceleration,
                         &jerk);
      if ((((sample != 0U) &&
            (sample != ARM_TRAJECTORY_SAMPLE_COUNT)) &&
           ((position < (float)joint_config[joint].min_angle) ||
            (position > (float)joint_config[joint].max_angle))) ||
          (AbsFloat(velocity) > velocity_limit) ||
          (AbsFloat(acceleration) > acceleration_limit) ||
          (AbsFloat(jerk) > maximum_jerk)) {
        return 0U;
      }
    }
  }

  return 1U;
}

/*
 * 현재 위치·속도·가속도에서 새 목표로 이어지는 S-커브를 계획한다.
 * 한계를 만족하지 않으면 시간을 25%씩 늘리며 가장 짧은 안전 궤적을 찾는다.
 */
static uint8_t PlanTrajectory(void) {
  const float time_step = (float)ARM_CONTROL_INTERVAL_MS / 1000.0F;
  const float ramp_time = (float)(ARM_RAMP_STEPS * ARM_CONTROL_INTERVAL_MS) /
                          1000.0F;
  const float speed_scale = (float)speed_percent / 100.0F;
  const float maximum_velocity =
      (1000.0F / (float)ARM_REFERENCE_STEP_INTERVAL_MS) * speed_scale;
  const float maximum_acceleration = maximum_velocity / ramp_time;
  const float maximum_jerk = maximum_acceleration / ramp_time;
  float duration = time_step;
  uint8_t attempt;
  uint8_t joint;

  for (attempt = 0U; attempt < ARM_TRAJECTORY_SEARCH_COUNT; attempt++) {
    if (IsTrajectorySafe(duration,
                         maximum_velocity,
                         maximum_acceleration,
                         maximum_jerk) != 0U) {
      for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
        BuildTrajectoryCoefficients(
            joint, duration, trajectory_coefficients[joint]);
      }
      trajectory_duration_s = duration;
      trajectory_elapsed_s = 0.0F;
      return 1U;
    }
    duration *= 1.25F;
  }

  /*
   * 관절 끝단에서 진행 방향을 급히 뒤집는 등 연속 궤적이 범위를 벗어나면
   * 현재 위치에서 정지한 뒤 다시 계획한다. 위치 점프보다 안전을 우선한다.
   */
  speed_warning_pending = 1U;
  StopMotion();
  duration = time_step;
  for (attempt = 0U; attempt < ARM_TRAJECTORY_SEARCH_COUNT; attempt++) {
    if (IsTrajectorySafe(duration,
                         maximum_velocity,
                         maximum_acceleration,
                         maximum_jerk) != 0U) {
      for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
        BuildTrajectoryCoefficients(
            joint, duration, trajectory_coefficients[joint]);
      }
      trajectory_duration_s = duration;
      trajectory_elapsed_s = 0.0F;
      return 1U;
    }
    duration *= 1.25F;
  }

  return 0U;
}

/* 계획된 공통 궤적에서 한 관절의 다음 20 ms 위치를 출력한다. */
static uint8_t StepJointTrajectory(uint8_t joint, float elapsed) {
  float position;
  float velocity;
  float acceleration;
  float jerk;

  if (elapsed >= trajectory_duration_s) {
    position = (float)target_angles[joint];
    velocity = 0.0F;
    acceleration = 0.0F;
  } else {
    EvaluateTrajectory(trajectory_coefficients[joint],
                       elapsed,
                       &position,
                       &velocity,
                       &acceleration,
                       &jerk);
  }

  motion_velocities[joint] = velocity;
  motion_accelerations[joint] = acceleration;
  return (SetJointPosition(joint, position) == HAL_OK) ? 1U : 0U;
}

/* 티칭 웨이포인트의 6개 각도를 새 목표 자세로 복사한다. */
static void LoadTarget(const TeachingWaypoint *waypoint) {
  memcpy(target_angles, waypoint->angles, ROBOT_ARM_JOINT_COUNT);
  if (preserve_sequence_gripper != 0U) {
    target_angles[ROBOT_ARM_JOINT_COUNT - 1U] =
        current_angles[ROBOT_ARM_JOINT_COUNT - 1U];
  }
}

HAL_StatusTypeDef RobotArm_Init(I2C_HandleTypeDef *i2c) {
  HAL_StatusTypeDef status;
  uint8_t joint;

  arm_state = ARM_IDLE;
  arm_enabled = 0U;
  arm_initialized = 0U;
  pose_reached_pending = 0U;
  speed_warning_pending = 0U;
  preserve_sequence_gripper = 0U;
  preview_active = 0U;
  preview_moving = 0U;
  preview_joint = 0U;
  preview_position_valid = 0U;
  preview_position_joint = 0U;
  preview_request_id = 0U;
  preview_result_pending = 0U;
  active_sequence = NULL;
  motion_tick = HAL_GetTick();
  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    current_angles[joint] = origin_angles[joint];
    target_angles[joint] = origin_angles[joint];
    motion_positions[joint] = (float)origin_angles[joint];
    motion_velocities[joint] = 0.0F;
    motion_accelerations[joint] = 0.0F;
    last_pulse_widths_us[joint] = 0U;
  }

  /* 실제 위치를 읽을 수 없어 부팅 직후의 내부 위치는 홈으로 가정한다. */
  status = ServoDriver_Init(i2c);
  if (status == HAL_OK) {
    arm_initialized = 1U;
  }
  return status;
}

RobotArmResult
RobotArm_EnableOrigin(const uint8_t angles[ROBOT_ARM_JOINT_COUNT]) {
  RobotArmResult result;
  uint8_t joint;

  result = ValidatePose(angles);
  if (result != ROBOT_ARM_OK) {
    return result;
  }

  RobotArm_Stop();
  RobotArm_StopPreview();
  preview_position_valid = 0U;

  /*
   * 출력이 꺼지기 전의 마지막 명령 위치부터 다시 시작한다.
   * 부팅 직후에는 위치 피드백이 없어 홈 위치를 시작점으로 가정한다.
  */
  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    if (SetJointPosition(joint, motion_positions[joint]) != HAL_OK) {
      (void)RobotArm_Disable();
      return ROBOT_ARM_DRIVER_ERROR;
    }
  }

  arm_enabled = 1U;
  /*
   * 위치 피드백이 없어 첫 PWM 출력에서 실제 이동 속도를 보장할 수 없다.
   * 완료 ACK에서 앱이 사용자에게 별도 경고를 표시하게 한다.
   */
  speed_warning_pending = 1U;
  memcpy(target_angles, angles, sizeof(target_angles));
  state_tick = HAL_GetTick();
  motion_tick = state_tick;
  StopMotion();
  if (PlanTrajectory() == 0U) {
    speed_warning_pending = 0U;
    (void)RobotArm_Disable();
    return ROBOT_ARM_INVALID;
  }
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
  pose_reached_pending = 0U;
  memcpy(target_angles, angles, sizeof(target_angles));

  /*
   * 슬라이더 패킷이 연속으로 와도 기존 이동 시간을 유지해야
   * RobotArm_Update가 일정한 속도로 계속 움직일 수 있다.
   */
  if (arm_state != ARM_MOVING) {
    state_tick = HAL_GetTick();
    motion_tick = state_tick;
  }
  if (PlanTrajectory() == 0U) {
    speed_warning_pending = 0U;
    RobotArm_Stop();
    return ROBOT_ARM_INVALID;
  }
  arm_state = ARM_MOVING;

  return ROBOT_ARM_OK;
}

RobotArmResult RobotArm_SetSpeedPercent(uint8_t percent) {
  float maximum_velocity;
  uint8_t joint;

  if ((percent < ARM_SPEED_MIN_PERCENT) ||
      (percent > ARM_SPEED_MAX_PERCENT)) {
    return ROBOT_ARM_INVALID;
  }

  maximum_velocity =
      (1000.0F / (float)ARM_REFERENCE_STEP_INTERVAL_MS) *
      ((float)percent / 100.0F);
  if (arm_state == ARM_MOVING) {
    for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
      if (AbsFloat(motion_velocities[joint]) > maximum_velocity) {
        /*
         * 즉시 속도를 잘라내면 펄스가 튀므로 부드럽게 감속한다.
         * 그동안 새 설정값보다 빠를 수 있음을 완료 ACK로 알린다.
         */
        speed_warning_pending = 1U;
        break;
      }
    }
  }

  speed_percent = percent;
  return ROBOT_ARM_OK;
}

RobotArmResult RobotArm_Play(const TeachingSequence *sequence,
                             uint8_t preserve_gripper) {
  RobotArmResult result;
  uint8_t waypoint;

  if (arm_enabled == 0U) {
    return ROBOT_ARM_NOT_ENABLED;
  }

  if ((sequence == NULL) || (sequence->count == 0U) ||
      (preserve_gripper > 1U)) {
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
  preserve_sequence_gripper = preserve_gripper;
  waypoint_index = 0U;
  LoadTarget(&active_sequence->waypoints[0]);
  state_tick = HAL_GetTick();
  motion_tick = state_tick;
  StopMotion();
  if (PlanTrajectory() == 0U) {
    speed_warning_pending = 0U;
    RobotArm_Stop();
    return ROBOT_ARM_INVALID;
  }
  arm_state = ARM_MOVING;
  return ROBOT_ARM_OK;
}

void RobotArm_Stop(void) {
  arm_state = ARM_IDLE;
  active_sequence = NULL;
  preserve_sequence_gripper = 0U;
  pose_reached_pending = 0U;
  /* 중단 뒤 다시 활성화해도 이전 목표로 갑자기 이어서 움직이지 않게 한다. */
  memcpy(target_angles, current_angles, sizeof(target_angles));
  StopMotion();
}

RobotArmResult RobotArm_Disable(void) {
  HAL_StatusTypeDef status;
  uint8_t joint;

  RobotArm_Stop();
  arm_enabled = 0U;
  preview_active = 0U;
  preview_moving = 0U;
  preview_position_valid = 0U;
  preview_result_pending = 0U;
  speed_warning_pending = 0U;
  status = ServoDriver_DisableAll();
  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    /* 출력 차단 뒤 같은 위치를 다시 켜도 반드시 PWM을 재전송한다. */
    last_pulse_widths_us[joint] = 0U;
  }
  return (status == HAL_OK) ? ROBOT_ARM_OK : ROBOT_ARM_DRIVER_ERROR;
}

RobotArmResult RobotArm_SetCalibrations(
    const uint16_t pulse_us[ROBOT_ARM_JOINT_COUNT]
                           [ROBOT_ARM_CALIBRATION_POINT_COUNT]) {
  uint8_t joint;
  uint16_t minimum;
  uint16_t maximum;
  uint16_t minus_90;
  uint16_t center;
  uint16_t plus_90;

  if (pulse_us == NULL) {
    return ROBOT_ARM_INVALID;
  }

  /*
   * 보정 화면에서는 다른 관절을 원점으로 유지한 채 새 보정값을 적용한다.
   * 일반 이동이나 미리보기 궤적이 진행 중일 때만 변경을 거부한다.
   */
  if (((arm_enabled != 0U) && (arm_state != ARM_IDLE)) ||
      (preview_active != 0U)) {
    return ROBOT_ARM_BUSY;
  }

  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    minus_90 = pulse_us[joint][0];
    center = pulse_us[joint][1];
    plus_90 = pulse_us[joint][2];
    minimum = (joint == ROBOT_ARM_GRIPPER_INDEX)
                  ? ARM_GRIPPER_CALIBRATION_MIN_US
                  : ARM_CALIBRATION_MIN_US;
    maximum = (joint == ROBOT_ARM_GRIPPER_INDEX)
                  ? ARM_GRIPPER_CALIBRATION_MAX_US
                  : ARM_CALIBRATION_MAX_US;

    if ((minus_90 < minimum) || (minus_90 > maximum) ||
        (center < minimum) || (center > maximum) ||
        (plus_90 < minimum) || (plus_90 > maximum)) {
      return ROBOT_ARM_INVALID;
    }

    if (joint == ROBOT_ARM_GRIPPER_INDEX) {
      if (center != (uint16_t)((minus_90 + plus_90) / 2U)) {
        return ROBOT_ARM_INVALID;
      }
    }

    if (joint_config[joint].reversed != 0U) {
      if (!((minus_90 > center) && (center > plus_90))) {
        return ROBOT_ARM_INVALID;
      }
    } else if (!((minus_90 < center) && (center < plus_90))) {
      return ROBOT_ARM_INVALID;
    }
  }

  for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
    if (joint_config[joint].reversed != 0U) {
      joint_config[joint].calibration.min_pulse_us = pulse_us[joint][2];
      joint_config[joint].calibration.center_pulse_us = pulse_us[joint][1];
      joint_config[joint].calibration.max_pulse_us = pulse_us[joint][0];
    } else {
      joint_config[joint].calibration.min_pulse_us = pulse_us[joint][0];
      joint_config[joint].calibration.center_pulse_us = pulse_us[joint][1];
      joint_config[joint].calibration.max_pulse_us = pulse_us[joint][2];
    }
  }

  return ROBOT_ARM_OK;
}

RobotArmResult RobotArm_PreviewPulse(uint8_t joint,
                                    uint16_t pulse_us,
                                    uint8_t requested_speed_percent,
                                    uint8_t request_id) {
  uint16_t initial_pulse_us;
  uint16_t minimum;
  uint16_t maximum;

  if ((joint >= ROBOT_ARM_JOINT_COUNT) ||
      (arm_initialized == 0U) ||
      (request_id == 0U) ||
      (requested_speed_percent < ARM_SPEED_MIN_PERCENT) ||
      (requested_speed_percent > ARM_SPEED_MAX_PERCENT)) {
    return ROBOT_ARM_INVALID;
  }
  if (arm_enabled == 0U) {
    return ROBOT_ARM_NOT_ENABLED;
  }
  if (arm_state != ARM_IDLE) {
    return ROBOT_ARM_BUSY;
  }

  minimum = (joint == ROBOT_ARM_GRIPPER_INDEX)
                ? ARM_GRIPPER_CALIBRATION_MIN_US
                : ARM_CALIBRATION_MIN_US;
  maximum = (joint == ROBOT_ARM_GRIPPER_INDEX)
                ? ARM_GRIPPER_CALIBRATION_MAX_US
                : ARM_CALIBRATION_MAX_US;
  if ((pulse_us < minimum) || (pulse_us > maximum)) {
    return ROBOT_ARM_INVALID;
  }

  /*
   * 다른 관절의 PCA9685 출력을 그대로 유지하고 선택한 관절만 조정한다.
   * 원점 진입이 먼저 완료되므로 마지막 출력 펄스를 보정 시작점으로 사용한다.
   */
  if ((preview_active == 0U) || (preview_joint != joint)) {
    if (last_pulse_widths_us[joint] != 0U) {
      initial_pulse_us = last_pulse_widths_us[joint];
    } else if ((preview_position_valid != 0U) &&
        (preview_position_joint == joint)) {
      initial_pulse_us = (uint16_t)(preview_position_us + 0.5F);
    } else {
      initial_pulse_us = GetPulseWidthForPosition(joint, motion_positions[joint]);
    }
    if (initial_pulse_us < minimum) {
      initial_pulse_us = minimum;
    } else if (initial_pulse_us > maximum) {
      initial_pulse_us = maximum;
    }
    if (ServoDriver_SetPulseUs(joint, initial_pulse_us) != HAL_OK) {
      return ROBOT_ARM_DRIVER_ERROR;
    }
    preview_position_us = (float)initial_pulse_us;
    preview_position_valid = 1U;
    preview_position_joint = joint;
    preview_velocity_us_s = 0.0F;
    preview_acceleration_us_s2 = 0.0F;
    preview_joint = joint;
    preview_active = 1U;
    last_pulse_widths_us[joint] = initial_pulse_us;
  }

  preview_target_pulse_us = pulse_us;
  preview_request_id = request_id;
  preview_result_pending = 0U;
  preview_command_tick = HAL_GetTick();
  preview_motion_tick = preview_command_tick;

  if ((uint16_t)(preview_position_us + 0.5F) == pulse_us) {
    preview_position_us = (float)pulse_us;
    preview_velocity_us_s = 0.0F;
    preview_acceleration_us_s2 = 0.0F;
    preview_moving = 0U;
    preview_result = ROBOT_ARM_OK;
    preview_result_pending = 1U;
    return ROBOT_ARM_OK;
  }

  if (PlanPreviewTrajectory(requested_speed_percent, minimum, maximum) == 0U) {
    preview_moving = 0U;
    return ROBOT_ARM_INVALID;
  }

  preview_moving = 1U;
  return ROBOT_ARM_OK;
}

void RobotArm_StopPreview(void) {
  uint8_t joint;

  preview_active = 0U;
  preview_moving = 0U;
  preview_request_id = 0U;
  preview_result_pending = 0U;

  /* 활성 보정에서는 현재 6채널 출력을 유지하고 미리보기 상태만 끝낸다. */
  if (arm_enabled == 0U) {
    (void)ServoDriver_DisableAll();
    for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
      last_pulse_widths_us[joint] = 0U;
    }
  }
}

uint8_t RobotArm_TakePreviewResult(uint8_t *joint,
                                  uint16_t *pulse_us,
                                  uint8_t *request_id,
                                  RobotArmResult *result) {
  if ((preview_result_pending == 0U) ||
      (joint == NULL) ||
      (pulse_us == NULL) ||
      (request_id == NULL) ||
      (result == NULL)) {
    return 0U;
  }

  *joint = preview_joint;
  *pulse_us = preview_target_pulse_us;
  *request_id = preview_request_id;
  *result = preview_result;
  preview_result_pending = 0U;
  return 1U;
}

uint8_t RobotArm_IsEnabled(void) {
  return arm_enabled;
}

uint8_t RobotArm_IsMotionActive(void) {
  /*
   * 보정 미리보기 중에도 차량이 움직이면 작업자와 기구물이 위험하다.
   * 이동 완료 뒤 펄스를 확인하는 2초 유지 구간도 PREVIEW_STOP까지 잠근다.
   */
  return (uint8_t)(((arm_enabled != 0U) && (arm_state != ARM_IDLE)) ||
                   (preview_active != 0U));
}

uint8_t RobotArm_TakePoseReached(void) {
  uint8_t reached = pose_reached_pending;

  pose_reached_pending = 0U;
  return reached;
}

uint8_t RobotArm_TakeSpeedWarning(void) {
  uint8_t warning = speed_warning_pending;

  speed_warning_pending = 0U;
  return warning;
}

void RobotArm_Update(void) {
  uint32_t now = HAL_GetTick();
  float preview_position;
  float preview_velocity;
  float preview_acceleration;
  float preview_jerk;
  uint16_t preview_output_us;
  uint8_t joint;
  uint8_t target_pending = 0U;

  if (preview_active != 0U) {
    if ((preview_moving != 0U) &&
        ((now - preview_motion_tick) >= ARM_CONTROL_INTERVAL_MS)) {
      preview_motion_tick = now;
      preview_elapsed_s += (float)ARM_CONTROL_INTERVAL_MS / 1000.0F;
      if (preview_elapsed_s > preview_duration_s) {
        preview_elapsed_s = preview_duration_s;
      }

      if (preview_elapsed_s >= preview_duration_s) {
        preview_position = (float)preview_target_pulse_us;
        preview_velocity = 0.0F;
        preview_acceleration = 0.0F;
      } else {
        EvaluateTrajectory(preview_coefficients,
                           preview_elapsed_s,
                           &preview_position,
                           &preview_velocity,
                           &preview_acceleration,
                           &preview_jerk);
      }

      preview_output_us = (uint16_t)(preview_position + 0.5F);
      if (preview_output_us != last_pulse_widths_us[preview_joint]) {
        if (ServoDriver_SetPulseUs(preview_joint, preview_output_us) != HAL_OK) {
          preview_moving = 0U;
          preview_active = 0U;
          arm_enabled = 0U;
          arm_state = ARM_IDLE;
          active_sequence = NULL;
          preview_result = ROBOT_ARM_DRIVER_ERROR;
          preview_result_pending = 1U;
          (void)ServoDriver_DisableAll();
          for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
            last_pulse_widths_us[joint] = 0U;
          }
          return;
        }
        last_pulse_widths_us[preview_joint] = preview_output_us;
      }
      preview_position_us = preview_position;
      preview_position_valid = 1U;
      preview_position_joint = preview_joint;
      preview_velocity_us_s = preview_velocity;
      preview_acceleration_us_s2 = preview_acceleration;

      if (preview_elapsed_s >= preview_duration_s) {
        preview_moving = 0U;
        preview_command_tick = now;
        preview_result = ROBOT_ARM_OK;
        preview_result_pending = 1U;
      }
    }

    /* 이동 완료 후 2초가 지나면 펄스는 유지하고 미리보기 상태만 끝낸다. */
    if ((preview_moving == 0U) &&
        ((now - preview_command_tick) >= ARM_PREVIEW_TIMEOUT_MS)) {
      RobotArm_StopPreview();
    }
    return;
  }

  if ((arm_enabled == 0U) || (arm_state == ARM_IDLE)) {
    return;
  }

  if (arm_state == ARM_MOVING) {
    if ((now - motion_tick) < ARM_CONTROL_INTERVAL_MS) {
      return;
    }
    /* 지연된 호출을 몰아서 실행하지 않아 I2C와 서보에 급격한 출력을 주지 않는다. */
    motion_tick = now;
    trajectory_elapsed_s +=
        (float)ARM_CONTROL_INTERVAL_MS / 1000.0F;
    if (trajectory_elapsed_s > trajectory_duration_s) {
      trajectory_elapsed_s = trajectory_duration_s;
    }

    for (joint = 0U; joint < ROBOT_ARM_JOINT_COUNT; joint++) {
      if (StepJointTrajectory(joint, trajectory_elapsed_s) == 0U) {
        (void)RobotArm_Disable();
        return;
      }
    }
    target_pending =
        (trajectory_elapsed_s < trajectory_duration_s) ? 1U : 0U;

    if (target_pending == 0U) {
      if (active_sequence == NULL) {
        RobotArm_Stop();
        pose_reached_pending = 1U;
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
      motion_tick = now;
      if (PlanTrajectory() == 0U) {
        RobotArm_Stop();
      }
    } else {
      const uint8_t travel_sequence_completed = preserve_sequence_gripper;
      RobotArm_Stop();
      pose_reached_pending = travel_sequence_completed;
    }
  }
}
