#ifndef DRIVE_4WD_H
#define DRIVE_4WD_H

/* L298N 좌우 모터 그룹과 주행 안전 인터록 인터페이스. */

#include "bluetooth.h"
#include "stm32f4xx_hal.h"

typedef enum
{
  DRIVE_PID_OK = 0,
  DRIVE_PID_UNAVAILABLE,
  DRIVE_PID_INVALID_GAINS
} DrivePidResult;

typedef struct
{
  uint8_t command_active;
  uint8_t pid_enabled;
  uint8_t pid_running;
  uint8_t reverse;
  uint8_t target_updated;
  uint8_t imu_available;
  float target_yaw_deg;
  float current_yaw_deg;
  float error_deg;
  float output;
  uint8_t left_pwm;
  uint8_t right_pwm;
} DrivePidStatus;

/* TIM3 PWM을 시작하고 모든 모터를 정지 상태로 만든다. */
HAL_StatusTypeDef Drive4WD_Init(TIM_HandleTypeDef *timer);

/* 앱의 0=전진, 1=후진 명령을 좌우 모터에 적용한다. */
void Drive4WD_Apply(const BluetoothDriveCommand *command);

/* 방향 핀과 PWM을 모두 0으로 만든다. */
void Drive4WD_Stop(void);

/* 마지막 주행 명령 후 500 ms가 지나면 차량을 정지한다. */
void Drive4WD_CheckTimeout(void);

/* Flash 작업 중에는 차량을 정지하고 새 주행 명령을 적용하지 않는다. */
void Drive4WD_SetStorageInhibit(uint8_t inhibit);

/* 로봇팔이 움직이는 동안 차량을 정지하고 새 주행 명령을 적용하지 않는다. */
void Drive4WD_SetArmMotionInhibit(uint8_t inhibit);

/* IMU 자이로 영점 보정 중에는 차량을 정지 상태로 유지한다. */
void Drive4WD_SetImuCalibrationInhibit(uint8_t inhibit);

/* Flash에서 읽은 1000배 정수 PID 계수를 실행 제어기에 적용한다. */
DrivePidResult Drive4WD_SetPidGains(int32_t kp_milli,
                                    int32_t ki_milli,
                                    int32_t kd_milli);

/* IMU 사용 가능 상태가 사라지면 PID를 해제하고 차량을 정지한다. */
void Drive4WD_SetImuAvailable(uint8_t available);

/* 새 IMU Yaw 샘플마다 직진 PID를 한 번 계산한다. */
void Drive4WD_ProcessYaw(float yaw_deg, uint32_t sample_tick_ms);

/* 방향 PID 적용 상태를 바꾸기 전에 차량을 정지한다. */
DrivePidResult Drive4WD_SetPidEnabled(uint8_t enabled);

/* 실제 차량 제어에 PID가 적용 중이면 1을 반환한다. */
uint8_t Drive4WD_IsPidEnabled(void);

/* 앱 상태 표시용 PID 스냅샷을 복사한다. */
void Drive4WD_GetPidStatus(DrivePidStatus *status);

#endif /* DRIVE_4WD_H */
