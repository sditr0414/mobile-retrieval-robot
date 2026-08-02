#include "imu.h"

#include <stddef.h>
#include <string.h>

/*
 * MPU6050의 가속도·자이로 값을 I2C로 읽고 영점과 저역통과 필터를 적용한다.
 * MPU6050 전용 I2C3 접근을 Mutex로 보호해 태스크 간 충돌을 막는다.
 */

#define MPU6050_ADDRESS            (0x68U << 1)
#define MPU6050_REG_ACCEL_CONFIG   (0x1CU)
#define MPU6050_REG_GYRO_CONFIG    (0x1BU)
#define MPU6050_REG_ACCEL_XOUT_H   (0x3BU)
#define MPU6050_REG_PWR_MGMT_1     (0x6BU)
#define MPU6050_REG_PWR_MGMT_2     (0x6CU)
#define MPU6050_REG_WHO_AM_I       (0x75U)
#define MPU6050_WHO_AM_I_VALUE     (0x68U)

#define MPU6050_ACCEL_SCALE        (16384.0f)
#define MPU6050_GYRO_SCALE         (131.0f)
#define MPU6050_TIMEOUT_MS         (1000U)
#define MPU6050_PROBE_TIMEOUT_MS   (1000U)
#define IMU_CALIBRATION_MAX_ERRORS (5U)

/* F411 참조 프로젝트의 필터값이다. 실제 차체에서 다시 확인한다. */
#define IMU_GYRO_LPF_CUTOFF_HZ     (2.0f)
#define IMU_GYRO_XY_DEADBAND_DPS   (0.30f)
#define IMU_GYRO_Z_DEADBAND_DPS    (0.15f)
#define IMU_TWO_PI                 (6.28318530718f)

typedef struct
{
  float alpha;
  float x;
  float y;
  float z;
  uint8_t initialized;
} IMU_Filter;

static I2C_HandleTypeDef *imu_i2c;
static osMutexId_t imu_i2c_mutex;
static IMU_Data imu_latest;
static IMU_Filter imu_filter;
static float gyro_bias_x_raw;
static float gyro_bias_y_raw;
static float gyro_bias_z_raw;
static float integrated_yaw_deg;
static uint32_t previous_sample_tick;

/* 최신 측정 구조체는 짧은 임계구역으로 복사해 I2C Mutex 경합을 피한다. */
static uint32_t EnterStateCritical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void ExitStateCritical(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/* 공용 I2C Mutex를 제한 시간 안에 획득한다. */
static HAL_StatusTypeDef LockI2C(void)
{
  if ((imu_i2c == NULL) || (imu_i2c_mutex == NULL))
  {
    return HAL_ERROR;
  }

  return (osMutexAcquire(imu_i2c_mutex, MPU6050_TIMEOUT_MS) == osOK)
             ? HAL_OK
             : HAL_TIMEOUT;
}

/* 현재 태스크가 사용한 공용 I2C Mutex를 반환한다. */
static void UnlockI2C(void)
{
  (void)osMutexRelease(imu_i2c_mutex);
}

/* MPU6050의 Big-endian 두 바이트를 signed 16비트 값으로 합친다. */
static int16_t CombineBytes(uint8_t high_byte, uint8_t low_byte)
{
  return (int16_t)(((uint16_t)high_byte << 8) | low_byte);
}

/* 필터 출력과 최초 샘플 상태를 초기화한다. */
static void ResetFilter(void)
{
  float sample_period_sec =
      (float)IMU_SAMPLE_INTERVAL_MS / 1000.0f;
  float rc = 1.0f / (IMU_TWO_PI * IMU_GYRO_LPF_CUTOFF_HZ);

  imu_filter.alpha = sample_period_sec / (rc + sample_period_sec);
  imu_filter.x = 0.0f;
  imu_filter.y = 0.0f;
  imu_filter.z = 0.0f;
  imu_filter.initialized = 0U;
}

/* 영점 부근의 작은 자이로 노이즈를 0으로 만든다. */
static float ApplyDeadband(float value, float threshold)
{
  if ((value >= -threshold) && (value <= threshold))
  {
    return 0.0f;
  }

  return value;
}

/* 3축 자이로에 1차 저역통과 필터를 적용한다. */
static void ApplyGyroFilter(float input_x,
                            float input_y,
                            float input_z,
                            float *output_x,
                            float *output_y,
                            float *output_z)
{
  if (imu_filter.initialized == 0U)
  {
    imu_filter.x = input_x;
    imu_filter.y = input_y;
    imu_filter.z = input_z;
    imu_filter.initialized = 1U;
  }
  else
  {
    imu_filter.x += imu_filter.alpha * (input_x - imu_filter.x);
    imu_filter.y += imu_filter.alpha * (input_y - imu_filter.y);
    imu_filter.z += imu_filter.alpha * (input_z - imu_filter.z);
  }

  *output_x = ApplyDeadband(imu_filter.x, IMU_GYRO_XY_DEADBAND_DPS);
  *output_y = ApplyDeadband(imu_filter.y, IMU_GYRO_XY_DEADBAND_DPS);
  *output_z = ApplyDeadband(imu_filter.z, IMU_GYRO_Z_DEADBAND_DPS);
}

/* MPU6050 레지스터 하나를 Mutex로 보호해 기록한다. */
static HAL_StatusTypeDef WriteRegister(uint8_t register_address,
                                       uint8_t value)
{
  HAL_StatusTypeDef status;

  if (LockI2C() != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  status = HAL_I2C_Mem_Write(imu_i2c,
                             MPU6050_ADDRESS,
                             register_address,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             MPU6050_TIMEOUT_MS);
  UnlockI2C();
  return status;
}

/* MPU6050의 연속 레지스터를 Mutex로 보호해 읽는다. */
static HAL_StatusTypeDef ReadRegisters(uint8_t register_address,
                                       uint8_t *data,
                                       uint16_t length)
{
  HAL_StatusTypeDef status;

  if ((data == NULL) || (length == 0U) || (LockI2C() != HAL_OK))
  {
    return HAL_ERROR;
  }

  status = HAL_I2C_Mem_Read(imu_i2c,
                            MPU6050_ADDRESS,
                            register_address,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            length,
                            MPU6050_TIMEOUT_MS);
  UnlockI2C();
  return status;
}

/* 가속도 6바이트, 온도 2바이트와 자이로 6바이트를 한 번에 읽는다. */
static HAL_StatusTypeDef ReadRaw(uint8_t raw_data[14])
{
  return ReadRegisters(MPU6050_REG_ACCEL_XOUT_H, raw_data, 14U);
}

/* 최근 통신 오류 시각과 누적 오류 횟수를 기록한다. */
static void RecordError(void)
{
  uint32_t primask = EnterStateCritical();
  imu_latest.error_count++;
  imu_latest.valid = 0U;
  ExitStateCritical(primask);
}

HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *i2c,
                           osMutexId_t i2c_mutex)
{
  HAL_StatusTypeDef status;
  uint32_t primask;
  uint8_t who_am_i = 0U;

  if ((i2c == NULL) || (i2c_mutex == NULL))
  {
    return HAL_ERROR;
  }

  imu_i2c = i2c;
  imu_i2c_mutex = i2c_mutex;

  primask = EnterStateCritical();
  imu_latest.initialized = 0U;
  imu_latest.calibrated = 0U;
  imu_latest.valid = 0U;
  ExitStateCritical(primask);

  if (LockI2C() != HAL_OK)
  {
    return HAL_TIMEOUT;
  }
  status = HAL_I2C_IsDeviceReady(imu_i2c,
                                 MPU6050_ADDRESS,
                                 3U,
                                 MPU6050_PROBE_TIMEOUT_MS);
  UnlockI2C();
  if (status != HAL_OK)
  {
    RecordError();
    return status;
  }

  status = ReadRegisters(MPU6050_REG_WHO_AM_I, &who_am_i, 1U);
  if ((status != HAL_OK) ||
      ((who_am_i != MPU6050_WHO_AM_I_VALUE) &&
       (who_am_i != 0x70U) &&
       (who_am_i != 0x71U) &&
       (who_am_i != 0x73U)))
  {
    RecordError();
    return HAL_ERROR;
  }

  if (WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x80U) != HAL_OK)
  {
    RecordError();
    return HAL_ERROR;
  }
  osDelay(150U);

  if (WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x01U) != HAL_OK)
  {
    RecordError();
    return HAL_ERROR;
  }
  osDelay(10U);

  if ((WriteRegister(MPU6050_REG_PWR_MGMT_2, 0x00U) != HAL_OK) ||
      (WriteRegister(MPU6050_REG_ACCEL_CONFIG, 0x00U) != HAL_OK) ||
      (WriteRegister(MPU6050_REG_GYRO_CONFIG, 0x00U) != HAL_OK))
  {
    RecordError();
    return HAL_ERROR;
  }

  gyro_bias_x_raw = 0.0f;
  gyro_bias_y_raw = 0.0f;
  gyro_bias_z_raw = 0.0f;
  integrated_yaw_deg = 0.0f;
  previous_sample_tick = 0U;
  ResetFilter();

  primask = EnterStateCritical();
  imu_latest.initialized = 1U;
  imu_latest.calibrated = 0U;
  imu_latest.valid = 0U;
  ExitStateCritical(primask);
  return HAL_OK;
}

HAL_StatusTypeDef IMU_CalibrateGyro(void)
{
  int64_t gyro_x_sum = 0;
  int64_t gyro_y_sum = 0;
  int64_t gyro_z_sum = 0;
  uint32_t sample_count = 0U;
  uint32_t consecutive_errors = 0U;
  uint32_t start_tick;
  uint32_t primask;
  uint8_t raw_data[14];

  if (imu_latest.initialized == 0U)
  {
    return HAL_ERROR;
  }

  primask = EnterStateCritical();
  imu_latest.calibrated = 0U;
  imu_latest.valid = 0U;
  ExitStateCritical(primask);

  start_tick = osKernelGetTickCount();
  while ((osKernelGetTickCount() - start_tick) <
         IMU_CALIBRATION_DURATION_MS)
  {
    if (ReadRaw(raw_data) == HAL_OK)
    {
      gyro_x_sum += CombineBytes(raw_data[8], raw_data[9]);
      gyro_y_sum += CombineBytes(raw_data[10], raw_data[11]);
      gyro_z_sum += CombineBytes(raw_data[12], raw_data[13]);
      sample_count++;
      consecutive_errors = 0U;
    }
    else
    {
      RecordError();
      consecutive_errors++;
      if (consecutive_errors >= IMU_CALIBRATION_MAX_ERRORS)
      {
        return HAL_ERROR;
      }
    }

    osDelay(IMU_CALIBRATION_INTERVAL_MS);
  }

  if (sample_count == 0U)
  {
    return HAL_ERROR;
  }

  gyro_bias_x_raw = (float)gyro_x_sum / (float)sample_count;
  gyro_bias_y_raw = (float)gyro_y_sum / (float)sample_count;
  gyro_bias_z_raw = (float)gyro_z_sum / (float)sample_count;
  integrated_yaw_deg = 0.0f;
  previous_sample_tick = 0U;
  ResetFilter();

  primask = EnterStateCritical();
  imu_latest.calibrated = 1U;
  imu_latest.valid = 0U;
  ExitStateCritical(primask);
  return HAL_OK;
}

HAL_StatusTypeDef IMU_Read(void)
{
  IMU_Data next_data;
  HAL_StatusTypeDef status;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float sample_period_sec;
  uint32_t sample_tick;
  uint32_t primask;
  uint8_t raw_data[14];

  if (imu_latest.initialized == 0U)
  {
    return HAL_ERROR;
  }

  status = ReadRaw(raw_data);
  if (status != HAL_OK)
  {
    RecordError();
    return status;
  }

  memset(&next_data, 0, sizeof(next_data));
  next_data.accel_x_g =
      (float)CombineBytes(raw_data[0], raw_data[1]) /
      MPU6050_ACCEL_SCALE;
  next_data.accel_y_g =
      (float)CombineBytes(raw_data[2], raw_data[3]) /
      MPU6050_ACCEL_SCALE;
  next_data.accel_z_g =
      (float)CombineBytes(raw_data[4], raw_data[5]) /
      MPU6050_ACCEL_SCALE;
  next_data.temperature_c =
      ((float)CombineBytes(raw_data[6], raw_data[7]) / 340.0f) + 36.53f;

  gyro_x = ((float)CombineBytes(raw_data[8], raw_data[9]) -
            gyro_bias_x_raw) /
           MPU6050_GYRO_SCALE;
  gyro_y = ((float)CombineBytes(raw_data[10], raw_data[11]) -
            gyro_bias_y_raw) /
           MPU6050_GYRO_SCALE;
  gyro_z = ((float)CombineBytes(raw_data[12], raw_data[13]) -
            gyro_bias_z_raw) /
           MPU6050_GYRO_SCALE;

  if (imu_latest.calibrated != 0U)
  {
    ApplyGyroFilter(gyro_x,
                    gyro_y,
                    gyro_z,
                    &next_data.gyro_x_dps,
                    &next_data.gyro_y_dps,
                    &next_data.gyro_z_dps);
  }
  else
  {
    next_data.gyro_x_dps = gyro_x;
    next_data.gyro_y_dps = gyro_y;
    next_data.gyro_z_dps = gyro_z;
  }

  sample_tick = HAL_GetTick();
  if ((previous_sample_tick == 0U) ||
      ((sample_tick - previous_sample_tick) == 0U) ||
      ((sample_tick - previous_sample_tick) > 200U))
  {
    sample_period_sec = (float)IMU_SAMPLE_INTERVAL_MS / 1000.0f;
  }
  else
  {
    sample_period_sec =
        (float)(sample_tick - previous_sample_tick) / 1000.0f;
  }
  previous_sample_tick = sample_tick;

  if (imu_latest.calibrated != 0U)
  {
    integrated_yaw_deg += next_data.gyro_z_dps * sample_period_sec;
    while (integrated_yaw_deg > 180.0f)
    {
      integrated_yaw_deg -= 360.0f;
    }
    while (integrated_yaw_deg < -180.0f)
    {
      integrated_yaw_deg += 360.0f;
    }
  }
  next_data.yaw_deg = integrated_yaw_deg;

  next_data.sample_tick_ms = sample_tick;
  next_data.error_count = imu_latest.error_count;
  next_data.initialized = 1U;
  next_data.calibrated = imu_latest.calibrated;
  next_data.valid = 1U;
  primask = EnterStateCritical();
  imu_latest = next_data;
  ExitStateCritical(primask);
  return HAL_OK;
}

uint8_t IMU_GetLatest(IMU_Data *data)
{
  uint32_t primask;

  if (data == NULL)
  {
    return 0U;
  }

  primask = EnterStateCritical();
  *data = imu_latest;
  ExitStateCritical(primask);
  return 1U;
}
