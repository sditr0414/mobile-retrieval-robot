#ifndef DRIVE_4WD_H
#define DRIVE_4WD_H

#include "bluetooth.h"
#include "stm32f4xx_hal.h"

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

#endif /* DRIVE_4WD_H */
