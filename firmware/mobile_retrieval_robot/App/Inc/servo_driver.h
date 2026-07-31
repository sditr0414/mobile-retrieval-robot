#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

/* PCA9685 50 Hz PWM과 서보 펄스 변환 인터페이스. */

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#define SERVO_DRIVER_CHANNEL_COUNT (16U)
#define SERVO_MAX_ANGLE_DEG        (180U)

/* 각 서보에서 실측한 0도, 90도, 180도 펄스폭을 저장한다. */
typedef struct
{
  uint16_t min_pulse_us;
  uint16_t center_pulse_us;
  uint16_t max_pulse_us;
} ServoCalibration;

/* PCA9685와 IMU가 공유하는 I2C 버스 mutex를 등록한다. */
void ServoDriver_SetI2CMutex(osMutexId_t i2c_mutex);

/* PCA9685를 50 Hz로 초기화하고 모든 채널을 정지한다. */
HAL_StatusTypeDef ServoDriver_Init(I2C_HandleTypeDef *i2c);

/* PCA9685의 모든 채널 출력을 한 번에 끈다. */
HAL_StatusTypeDef ServoDriver_DisableAll(void);

/* 보정값을 이용해 각도에 해당하는 펄스폭을 계산한다. */
uint16_t ServoDriver_AngleToPulseUs(
    uint16_t angle_deg,
    const ServoCalibration *calibration);

/* 지정한 PCA9685 채널에 펄스폭을 직접 출력한다. */
HAL_StatusTypeDef ServoDriver_SetPulseUs(uint8_t channel,
                                         uint16_t pulse_width_us);

/* 지정한 채널의 서보를 0~180도 범위로 이동한다. */
HAL_StatusTypeDef ServoDriver_SetAngle(
    uint8_t channel,
    uint16_t angle_deg,
    const ServoCalibration *calibration);

#endif /* SERVO_DRIVER_H */
