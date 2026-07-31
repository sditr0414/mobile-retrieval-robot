#include "drive_4wd.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "main.h"

/*
 * L298N 좌우 모터 그룹과 MPU6050 상대 Yaw PID를 함께 관리한다.
 * 모든 실제 PWM 기록은 High 우선순위 DriveTask에서 수행하며, E-STOP과
 * 통신·Flash·로봇팔·IMU 인터록이 PID보다 항상 먼저 적용된다.
 */

#define DRIVE_PWM_MAX              (999U)
#define APP_PWM_MAX                (255U)
#define MIN_PWM_THRESHOLD          (15U)
#define DRIVE_TIMEOUT_MS           (500U)
#define PID_KP_MAX_MILLI           (2550L)
#define PID_GAIN_MAX_MILLI         (100000L)
#define PID_KP_STEP_MILLI          (10L)
#define PID_INTEGRAL_LIMIT         (50.0f)
#define PID_OUTPUT_LIMIT           (80.0f)
#define PID_SAMPLE_MAX_MS          (200U)
#define PID_CORRECTION_SIGN        (-1.0f)

static TIM_HandleTypeDef *drive_timer;
static uint32_t last_command_tick;
static volatile uint8_t command_received;
static volatile uint8_t estop_latched;
static volatile uint8_t storage_inhibit;
static volatile uint8_t arm_motion_inhibit;
static volatile uint8_t imu_calibration_inhibit;
static volatile uint8_t pid_transition_inhibit;
static volatile uint8_t fresh_drive_command_required;
static volatile uint8_t pid_enabled;
static volatile uint8_t imu_available;
static volatile uint8_t pid_gains_valid;
static volatile uint32_t drive_inhibit_release_tick;

static BluetoothDriveCommand active_command;
static uint8_t pid_straight_active;
static uint8_t pid_target_pending;
static uint8_t applied_left_direction;
static uint8_t applied_right_direction;
static uint8_t applied_left_pwm;
static uint8_t applied_right_pwm;
static float pid_kp;
static float pid_ki;
static float pid_kd;
static float pid_target_yaw;
static float pid_integral;
static float pid_previous_error;
static uint32_t pid_previous_sample_tick;
static DrivePidStatus pid_status;

/* 한쪽 모터의 두 방향 핀을 전진·후진·중립 상태로 설정한다. */
static void SetMotorDirection(GPIO_TypeDef *in1_port,
                              uint16_t in1_pin,
                              GPIO_TypeDef *in2_port,
                              uint16_t in2_pin,
                              uint8_t direction,
                              uint8_t pwm)
{
  GPIO_PinState in1 = GPIO_PIN_RESET;
  GPIO_PinState in2 = GPIO_PIN_RESET;

  if (pwm != 0U)
  {
    if (direction == 0U)
    {
      in1 = GPIO_PIN_SET;
    }
    else
    {
      in2 = GPIO_PIN_SET;
    }
  }

  HAL_GPIO_WritePin(in1_port, in1_pin, in1);
  HAL_GPIO_WritePin(in2_port, in2_pin, in2);
}

/* 앱의 0~255 속도를 TIM2의 0~999 Compare 값으로 바꾼다. */
static uint32_t ScalePwm(uint8_t pwm)
{
  if (pwm < MIN_PWM_THRESHOLD)
  {
    return 0U;
  }
  return ((uint32_t)pwm * DRIVE_PWM_MAX) / APP_PWM_MAX;
}

static uint8_t ClampPwm(int32_t pwm)
{
  if (pwm <= 0)
  {
    return 0U;
  }
  if (pwm >= (int32_t)APP_PWM_MAX)
  {
    return APP_PWM_MAX;
  }
  return (uint8_t)pwm;
}

/* -180~+180도 범위에서 가장 짧은 방향 오차를 만든다. */
static float WrapYawError(float error)
{
  while (error > 180.0f)
  {
    error -= 360.0f;
  }
  while (error < -180.0f)
  {
    error += 360.0f;
  }
  return error;
}

static uint8_t IsInhibited(void)
{
  return (uint8_t)((estop_latched != 0U) ||
                   (storage_inhibit != 0U) ||
                   (arm_motion_inhibit != 0U) ||
                   (imu_calibration_inhibit != 0U) ||
                   (pid_transition_inhibit != 0U));
}

/* 목표와 누적항을 버려 다음 직진 구간에서 새 Yaw를 기준으로 잡는다. */
static void ResetPidMotion(void)
{
  pid_straight_active = 0U;
  pid_target_pending = 0U;
  pid_target_yaw = 0.0f;
  pid_integral = 0.0f;
  pid_previous_error = 0.0f;
  pid_previous_sample_tick = 0U;
  pid_status.pid_running = 0U;
  pid_status.target_updated = 0U;
  pid_status.target_yaw_deg = 0.0f;
  pid_status.error_deg = 0.0f;
  pid_status.output = 0.0f;
}

/* 방향 GPIO보다 PWM을 먼저 내린 뒤 좌우 출력을 함께 갱신한다. */
static void ApplyMotorOutput(uint8_t left_direction,
                             uint8_t left_pwm,
                             uint8_t right_direction,
                             uint8_t right_pwm)
{
  uint32_t left_compare;
  uint32_t right_compare;

  if (drive_timer == NULL)
  {
    return;
  }

  left_compare = ScalePwm(left_pwm);
  right_compare = ScalePwm(right_pwm);
  __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_2, 0U);
  SetMotorDirection(MOTOR_L_IN1_GPIO_Port,
                    MOTOR_L_IN1_Pin,
                    MOTOR_L_IN2_GPIO_Port,
                    MOTOR_L_IN2_Pin,
                    left_direction,
                    (uint8_t)(left_compare != 0U));
  SetMotorDirection(MOTOR_R_IN3_GPIO_Port,
                    MOTOR_R_IN3_Pin,
                    MOTOR_R_IN4_GPIO_Port,
                    MOTOR_R_IN4_Pin,
                    right_direction,
                    (uint8_t)(right_compare != 0U));
  __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_1, left_compare);
  __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_2, right_compare);

  applied_left_direction = left_direction;
  applied_right_direction = right_direction;
  applied_left_pwm = (left_compare == 0U) ? 0U : left_pwm;
  applied_right_pwm = (right_compare == 0U) ? 0U : right_pwm;
  pid_status.left_pwm = applied_left_pwm;
  pid_status.right_pwm = applied_right_pwm;
}

HAL_StatusTypeDef Drive4WD_Init(TIM_HandleTypeDef *timer)
{
  if (timer == NULL)
  {
    return HAL_ERROR;
  }

  drive_timer = timer;
  if ((HAL_TIM_PWM_Start(drive_timer, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(drive_timer, TIM_CHANNEL_2) != HAL_OK))
  {
    return HAL_ERROR;
  }

  memset(&active_command, 0, sizeof(active_command));
  memset(&pid_status, 0, sizeof(pid_status));
  estop_latched = 1U;
  storage_inhibit = 0U;
  arm_motion_inhibit = 0U;
  imu_calibration_inhibit = 0U;
  pid_transition_inhibit = 0U;
  pid_enabled = 0U;
  imu_available = 0U;
  fresh_drive_command_required = 0U;
  drive_inhibit_release_tick = 0U;
  command_received = 0U;
  ResetPidMotion();
  Drive4WD_Stop();
  return HAL_OK;
}

void Drive4WD_Apply(const BluetoothDriveCommand *command)
{
  uint8_t previous_pid_straight;
  uint8_t direction_changed;

  if ((drive_timer == NULL) || (command == NULL))
  {
    return;
  }

  if (command->control == BLUETOOTH_DRIVE_ESTOP)
  {
    estop_latched = 1U;
    pid_enabled = 0U;
    command_received = 0U;
    memset(&active_command, 0, sizeof(active_command));
    ResetPidMotion();
    Drive4WD_Stop();
    return;
  }

  if (command->control == BLUETOOTH_DRIVE_ESTOP_CLEAR)
  {
    estop_latched = 0U;
    command_received = 0U;
    memset(&active_command, 0, sizeof(active_command));
    ResetPidMotion();
    Drive4WD_Stop();
    return;
  }

  if (IsInhibited() != 0U)
  {
    Drive4WD_Stop();
    return;
  }

  if (fresh_drive_command_required != 0U)
  {
    if ((int32_t)(command->received_tick -
                  drive_inhibit_release_tick) <= 0)
    {
      Drive4WD_Stop();
      return;
    }
    fresh_drive_command_required = 0U;
  }

  if (command->engine_enabled == 0U)
  {
    command_received = 0U;
    memset(&active_command, 0, sizeof(active_command));
    ResetPidMotion();
    Drive4WD_Stop();
    return;
  }

  /*
   * 회전 중 방향이 바뀌면 현재 패킷은 중립 정지로 사용한다.
   * 다음 새 패킷에서만 반대 방향을 허용해 H-bridge 역전 전류를 줄인다.
   */
  if (((applied_left_pwm != 0U) && (command->left_pwm != 0U) &&
       (command->left_direction != applied_left_direction)) ||
      ((applied_right_pwm != 0U) && (command->right_pwm != 0U) &&
       (command->right_direction != applied_right_direction)))
  {
    ResetPidMotion();
    Drive4WD_Stop();
    last_command_tick = command->received_tick;
    command_received = 1U;
    return;
  }

  previous_pid_straight = pid_straight_active;
  direction_changed =
      (uint8_t)((active_command.left_direction != command->left_direction) ||
                (active_command.right_direction != command->right_direction));
  active_command = *command;
  pid_straight_active =
      (uint8_t)((pid_enabled != 0U) &&
                (imu_available != 0U) &&
                (command->pid_straight != 0U) &&
                (command->left_direction == command->right_direction) &&
                ((command->left_pwm != 0U) || (command->right_pwm != 0U)));

  if (pid_straight_active != 0U)
  {
    if ((previous_pid_straight == 0U) ||
        (direction_changed != 0U) ||
        (command->refresh_yaw_target != 0U))
    {
      pid_target_pending = 1U;
      pid_integral = 0.0f;
      pid_previous_error = 0.0f;
      pid_previous_sample_tick = 0U;
    }
  }
  else
  {
    ResetPidMotion();
  }

  ApplyMotorOutput(command->left_direction,
                   command->left_pwm,
                   command->right_direction,
                   command->right_pwm);
  last_command_tick = command->received_tick;
  command_received = 1U;
  pid_status.command_active = 1U;
  pid_status.reverse =
      (uint8_t)((command->left_direction != 0U) &&
                (command->right_direction != 0U));
}

void Drive4WD_ProcessYaw(float yaw_deg, uint32_t sample_tick_ms)
{
  float dt_sec;
  float error;
  float derivative;
  float output;
  float correction;
  int32_t final_left;
  int32_t final_right;

  pid_status.current_yaw_deg = yaw_deg;
  pid_status.imu_available = imu_available;
  pid_status.pid_enabled = pid_enabled;
  pid_status.pid_running = 0U;

  if ((imu_available == 0U) ||
      (pid_enabled == 0U) ||
      (pid_straight_active == 0U) ||
      (command_received == 0U) ||
      (IsInhibited() != 0U) ||
      ((HAL_GetTick() - last_command_tick) > DRIVE_TIMEOUT_MS))
  {
    return;
  }

  if (pid_target_pending != 0U)
  {
    pid_target_yaw = yaw_deg;
    pid_target_pending = 0U;
    pid_integral = 0.0f;
    pid_previous_error = 0.0f;
    pid_previous_sample_tick = sample_tick_ms;
    pid_status.target_updated = 1U;
  }

  error = WrapYawError(pid_target_yaw - yaw_deg);
  if ((pid_previous_sample_tick == 0U) ||
      ((sample_tick_ms - pid_previous_sample_tick) == 0U) ||
      ((sample_tick_ms - pid_previous_sample_tick) > PID_SAMPLE_MAX_MS))
  {
    dt_sec = (float)20U / 1000.0f;
    derivative = 0.0f;
  }
  else
  {
    dt_sec =
        (float)(sample_tick_ms - pid_previous_sample_tick) / 1000.0f;
    derivative = (error - pid_previous_error) / dt_sec;
  }
  pid_previous_sample_tick = sample_tick_ms;

  pid_integral += pid_ki * error * dt_sec;
  if (pid_integral > PID_INTEGRAL_LIMIT)
  {
    pid_integral = PID_INTEGRAL_LIMIT;
  }
  else if (pid_integral < -PID_INTEGRAL_LIMIT)
  {
    pid_integral = -PID_INTEGRAL_LIMIT;
  }

  output = (pid_kp * error) + pid_integral + (pid_kd * derivative);
  if (output > PID_OUTPUT_LIMIT)
  {
    output = PID_OUTPUT_LIMIT;
  }
  else if (output < -PID_OUTPUT_LIMIT)
  {
    output = -PID_OUTPUT_LIMIT;
  }

  correction = PID_CORRECTION_SIGN * output;
  if ((active_command.left_direction != 0U) &&
      (active_command.right_direction != 0U))
  {
    correction = -correction;
  }

  final_left = (int32_t)((float)active_command.left_pwm + correction);
  final_right = (int32_t)((float)active_command.right_pwm - correction);
  ApplyMotorOutput(active_command.left_direction,
                   ClampPwm(final_left),
                   active_command.right_direction,
                   ClampPwm(final_right));

  pid_previous_error = error;
  pid_status.pid_running = 1U;
  pid_status.target_yaw_deg = pid_target_yaw;
  pid_status.current_yaw_deg = yaw_deg;
  pid_status.error_deg = error;
  pid_status.output = output;
}

void Drive4WD_Stop(void)
{
  HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, GPIO_PIN_RESET);

  if (drive_timer != NULL)
  {
    __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_2, 0U);
  }
  applied_left_pwm = 0U;
  applied_right_pwm = 0U;
  pid_status.left_pwm = 0U;
  pid_status.right_pwm = 0U;
  pid_status.command_active = 0U;
  pid_status.pid_running = 0U;
}

void Drive4WD_CheckTimeout(void)
{
  if ((command_received != 0U) &&
      ((HAL_GetTick() - last_command_tick) > DRIVE_TIMEOUT_MS))
  {
    command_received = 0U;
    memset(&active_command, 0, sizeof(active_command));
    ResetPidMotion();
    pid_status.command_active = 0U;
    Drive4WD_Stop();
  }
}

static void SetInhibit(volatile uint8_t *state, uint8_t inhibit)
{
  if (inhibit != 0U)
  {
    *state = 1U;
    fresh_drive_command_required = 1U;
    command_received = 0U;
    ResetPidMotion();
    Drive4WD_Stop();
  }
  else if (*state != 0U)
  {
    drive_inhibit_release_tick = HAL_GetTick();
    *state = 0U;
  }
}

void Drive4WD_SetStorageInhibit(uint8_t inhibit)
{
  SetInhibit(&storage_inhibit, inhibit);
}

void Drive4WD_SetArmMotionInhibit(uint8_t inhibit)
{
  SetInhibit(&arm_motion_inhibit, inhibit);
}

void Drive4WD_SetImuCalibrationInhibit(uint8_t inhibit)
{
  SetInhibit(&imu_calibration_inhibit, inhibit);
}

DrivePidResult Drive4WD_SetPidGains(int32_t kp_milli,
                                    int32_t ki_milli,
                                    int32_t kd_milli)
{
  uint8_t valid =
      (uint8_t)((kp_milli >= 0) && (kp_milli <= PID_KP_MAX_MILLI) &&
                ((kp_milli % PID_KP_STEP_MILLI) == 0L) &&
                (ki_milli >= 0) && (ki_milli <= PID_GAIN_MAX_MILLI) &&
                (kd_milli >= 0) && (kd_milli <= PID_GAIN_MAX_MILLI) &&
                ((kp_milli != 0) || (ki_milli != 0) || (kd_milli != 0)));

  pid_kp = (float)kp_milli / 1000.0f;
  pid_ki = (float)ki_milli / 1000.0f;
  pid_kd = (float)kd_milli / 1000.0f;
  pid_gains_valid = valid;
  if (valid == 0U)
  {
    (void)Drive4WD_SetPidEnabled(0U);
    return DRIVE_PID_INVALID_GAINS;
  }
  return DRIVE_PID_OK;
}

void Drive4WD_SetImuAvailable(uint8_t available)
{
  imu_available = (available != 0U) ? 1U : 0U;
  pid_status.imu_available = imu_available;
  if (imu_available == 0U)
  {
    pid_enabled = 0U;
    command_received = 0U;
    fresh_drive_command_required = 1U;
    drive_inhibit_release_tick = HAL_GetTick();
    ResetPidMotion();
    Drive4WD_Stop();
  }
}

DrivePidResult Drive4WD_SetPidEnabled(uint8_t enabled)
{
  DrivePidResult result = DRIVE_PID_OK;

  /*
   * High 우선순위 DriveTask가 상태 변경 중간에 이전 PID PWM을 다시 쓰지
   * 못하도록 먼저 인터록을 올리고 마지막에 해제한다.
   */
  pid_transition_inhibit = 1U;
  pid_enabled = 0U;
  pid_status.pid_enabled = 0U;
  command_received = 0U;
  ResetPidMotion();
  Drive4WD_Stop();
  fresh_drive_command_required = 1U;
  drive_inhibit_release_tick = HAL_GetTick();

  if (enabled == 0U)
  {
    result = DRIVE_PID_OK;
  }
  else if ((imu_available == 0U) ||
           (storage_inhibit != 0U) ||
           (arm_motion_inhibit != 0U) ||
           (imu_calibration_inhibit != 0U) ||
           (estop_latched != 0U))
  {
    result = DRIVE_PID_UNAVAILABLE;
  }
  else if (pid_gains_valid == 0U)
  {
    result = DRIVE_PID_INVALID_GAINS;
  }
  else
  {
    pid_enabled = 1U;
    pid_status.pid_enabled = 1U;
  }

  pid_transition_inhibit = 0U;
  return result;
}

uint8_t Drive4WD_IsPidEnabled(void)
{
  return pid_enabled;
}

void Drive4WD_GetPidStatus(DrivePidStatus *status)
{
  uint32_t primask;

  if (status == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  pid_status.pid_enabled = pid_enabled;
  pid_status.imu_available = imu_available;
  *status = pid_status;
  pid_status.target_updated = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}
