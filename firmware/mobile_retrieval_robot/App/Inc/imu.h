#ifndef IMU_H
#define IMU_H

/* MPU6050 초기화, 영점 보정, 필터 측정값 조회 인터페이스. */

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#define IMU_SAMPLE_INTERVAL_MS       (20U)
#define IMU_CALIBRATION_DURATION_MS  (3000U)
#define IMU_CALIBRATION_INTERVAL_MS  (10U)

/* 상태 전송과 현재 방향 PID에서 함께 사용할 최신 MPU6050 측정값이다. */
typedef struct
{
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  float temperature_c;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  float yaw_deg;
  uint32_t sample_tick_ms;
  uint32_t error_count;
  uint8_t initialized;
  uint8_t calibrated;
  uint8_t valid;
} IMU_Data;

/* MPU6050을 ±2g, ±250 dps 범위로 초기화한다. */
HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *i2c,
                           osMutexId_t i2c_mutex);

/* 정지 상태에서 3초 동안 자이로 영점 오차를 측정한다. */
HAL_StatusTypeDef IMU_CalibrateGyro(void);

/* 가속도와 자이로를 한 번 읽고 최신 측정값을 갱신한다. */
HAL_StatusTypeDef IMU_Read(void);

/* 다른 태스크에서 사용할 수 있도록 최신 측정값을 복사한다. */
uint8_t IMU_GetLatest(IMU_Data *data);

#endif /* IMU_H */
