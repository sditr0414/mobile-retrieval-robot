#include "servo_driver.h"

/*
 * PCA9685를 50 Hz로 초기화하고 각 채널의 서보 펄스폭을 설정한다.
 * IMU와 같은 I2C1 버스를 사용하므로 모든 통신은 공용 Mutex로 직렬화한다.
 */

#define PCA9685_ADDRESS       (0x40U << 1)
#define PCA9685_MODE1         (0x00U)
#define PCA9685_MODE2         (0x01U)
#define PCA9685_LED0_ON_L     (0x06U)
#define PCA9685_ALL_LED_ON_L  (0xFAU)
#define PCA9685_PRE_SCALE     (0xFEU)

#define PCA9685_MODE1_RESTART (0x80U)
#define PCA9685_MODE1_AI      (0x20U)
#define PCA9685_MODE1_SLEEP   (0x10U)
#define PCA9685_MODE2_OUTDRV  (0x04U)

#define PCA9685_PRESCALE_50HZ (121U)
#define PCA9685_PERIOD_US     (20000U)
#define PCA9685_COUNT         (4096U)
#define PCA9685_FULL_OFF      (0x10U)
#define PCA9685_TIMEOUT_MS    (100U)

static I2C_HandleTypeDef *servo_i2c;
static osMutexId_t servo_i2c_mutex;

/* 공용 I2C Mutex를 획득해 다른 센서의 전송과 겹치지 않게 한다. */
static HAL_StatusTypeDef LockI2C(void)
{
  if (servo_i2c_mutex == NULL)
  {
    return HAL_ERROR;
  }

  return (osMutexAcquire(servo_i2c_mutex,
                         PCA9685_TIMEOUT_MS) == osOK)
             ? HAL_OK
             : HAL_TIMEOUT;
}

/* PCA9685 전송이 끝난 뒤 공용 I2C Mutex를 반환한다. */
static void UnlockI2C(void)
{
  (void)osMutexRelease(servo_i2c_mutex);
}

/* PCA9685 설정 레지스터 하나에 값을 기록한다. */
static HAL_StatusTypeDef WriteRegister(uint8_t register_address,
                                       uint8_t value)
{
  HAL_StatusTypeDef status;

  if (LockI2C() != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  status = HAL_I2C_Mem_Write(servo_i2c,
                             PCA9685_ADDRESS,
                             register_address,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             PCA9685_TIMEOUT_MS);
  UnlockI2C();
  return status;
}

/* 한 채널의 ON/OFF Count 네 바이트를 연속으로 기록한다. */
static HAL_StatusTypeDef WriteChannel(uint8_t channel,
                                      const uint8_t data[4])
{
  uint8_t register_address =
      (uint8_t)(PCA9685_LED0_ON_L + (4U * channel));
  HAL_StatusTypeDef status;

  if (LockI2C() != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  status = HAL_I2C_Mem_Write(servo_i2c,
                             PCA9685_ADDRESS,
                             register_address,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)data,
                             4U,
                             PCA9685_TIMEOUT_MS);
  UnlockI2C();
  return status;
}

void ServoDriver_SetI2CMutex(osMutexId_t i2c_mutex)
{
  servo_i2c_mutex = i2c_mutex;
}

HAL_StatusTypeDef ServoDriver_Init(I2C_HandleTypeDef *i2c)
{
  HAL_StatusTypeDef status;
  uint8_t mode1;

  if ((i2c == NULL) || (servo_i2c_mutex == NULL))
  {
    return HAL_ERROR;
  }

  servo_i2c = i2c;
  if (LockI2C() != HAL_OK)
  {
    return HAL_TIMEOUT;
  }
  status = HAL_I2C_IsDeviceReady(servo_i2c,
                                 PCA9685_ADDRESS,
                                 3U,
                                 PCA9685_TIMEOUT_MS);
  UnlockI2C();
  if (status != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (LockI2C() != HAL_OK)
  {
    return HAL_TIMEOUT;
  }
  status = HAL_I2C_Mem_Read(servo_i2c,
                            PCA9685_ADDRESS,
                            PCA9685_MODE1,
                            I2C_MEMADD_SIZE_8BIT,
                            &mode1,
                            1U,
                            PCA9685_TIMEOUT_MS);
  UnlockI2C();
  if (status != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 주파수 설정 중에는 PCA9685의 내부 발진기를 잠시 정지한다. */
  mode1 = (uint8_t)((mode1 | PCA9685_MODE1_AI | PCA9685_MODE1_SLEEP) &
                    ~PCA9685_MODE1_RESTART);
  if ((WriteRegister(PCA9685_MODE1, mode1) != HAL_OK) ||
      (WriteRegister(PCA9685_PRE_SCALE, PCA9685_PRESCALE_50HZ) != HAL_OK))
  {
    return HAL_ERROR;
  }

  mode1 = (uint8_t)(mode1 & ~PCA9685_MODE1_SLEEP);
  if (WriteRegister(PCA9685_MODE1, mode1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_Delay(1U);
  if ((WriteRegister(PCA9685_MODE1,
                     (uint8_t)(mode1 | PCA9685_MODE1_RESTART)) != HAL_OK) ||
      (WriteRegister(PCA9685_MODE2, PCA9685_MODE2_OUTDRV) != HAL_OK))
  {
    return HAL_ERROR;
  }

  /* 전원을 켰을 때 서보가 갑자기 움직이지 않도록 전 채널을 끈다. */
  return ServoDriver_DisableAll();
}

HAL_StatusTypeDef ServoDriver_DisableAll(void)
{
  uint8_t all_channels_off[4] = {0U, 0U, 0U, PCA9685_FULL_OFF};
  HAL_StatusTypeDef status;

  if ((servo_i2c == NULL) || (LockI2C() != HAL_OK))
  {
    return HAL_ERROR;
  }

  status = HAL_I2C_Mem_Write(servo_i2c,
                             PCA9685_ADDRESS,
                             PCA9685_ALL_LED_ON_L,
                             I2C_MEMADD_SIZE_8BIT,
                             all_channels_off,
                             4U,
                             PCA9685_TIMEOUT_MS);
  UnlockI2C();
  return status;
}

uint16_t ServoDriver_AngleToPulseUs(
    uint16_t angle_deg,
    const ServoCalibration *calibration)
{
  if ((calibration == NULL) ||
      (angle_deg > SERVO_MAX_ANGLE_DEG) ||
      (calibration->min_pulse_us >= calibration->center_pulse_us) ||
      (calibration->center_pulse_us >= calibration->max_pulse_us))
  {
    return 0U;
  }

  /* 0~90도와 90~180도를 각각 실측한 중앙값에 맞춰 계산한다. */
  if (angle_deg <= 90U)
  {
    return (uint16_t)(calibration->min_pulse_us +
                      (((uint32_t)(calibration->center_pulse_us -
                                   calibration->min_pulse_us) *
                        angle_deg) /
                       90U));
  }

  return (uint16_t)(calibration->center_pulse_us +
                    (((uint32_t)(calibration->max_pulse_us -
                                 calibration->center_pulse_us) *
                      (angle_deg - 90U)) /
                     90U));
}

HAL_StatusTypeDef ServoDriver_SetPulseUs(uint8_t channel,
                                         uint16_t pulse_width_us)
{
  uint16_t off_count;
  uint8_t data[4];

  if ((servo_i2c == NULL) ||
      (channel >= SERVO_DRIVER_CHANNEL_COUNT) ||
      (pulse_width_us >= PCA9685_PERIOD_US))
  {
    return HAL_ERROR;
  }

  off_count = (uint16_t)((((uint32_t)pulse_width_us * PCA9685_COUNT) +
                          (PCA9685_PERIOD_US / 2U)) /
                         PCA9685_PERIOD_US);
  if (off_count >= PCA9685_COUNT)
  {
    off_count = PCA9685_COUNT - 1U;
  }

  data[0] = 0U;
  data[1] = 0U;
  data[2] = (uint8_t)(off_count & 0xFFU);
  data[3] = (uint8_t)(off_count >> 8);

  return WriteChannel(channel, data);
}

HAL_StatusTypeDef ServoDriver_SetAngle(
    uint8_t channel,
    uint16_t angle_deg,
    const ServoCalibration *calibration)
{
  uint16_t pulse_width_us =
      ServoDriver_AngleToPulseUs(angle_deg, calibration);

  if (pulse_width_us == 0U)
  {
    return HAL_ERROR;
  }

  return ServoDriver_SetPulseUs(channel, pulse_width_us);
}
