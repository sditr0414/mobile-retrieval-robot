# STM32F411 권장 핀맵

> 기준 보드: NUCLEO-F411RE. 다른 F411 보드 사용 시 Alternate Function을 다시 확인합니다.

## I2C1

| 기능 | STM32 핀 | 연결 대상 |
|---|---|---|
| SCL | PB8 | PCA9685 SCL, IMU SCL |
| SDA | PB9 | PCA9685 SDA, IMU SDA |

## USART1

| 기능 | STM32 핀 | 연결 대상 |
|---|---|---|
| TX | PA9 | HC-05 RXD |
| RX | PA10 | HC-05 TXD |

## L298N

| 기능 | STM32 핀 | 비고 |
|---|---|---|
| ENA | PA6 / TIM3_CH1 | 왼쪽 PWM |
| IN1 | PB6 | 왼쪽 방향 |
| IN2 | PB7 | 왼쪽 방향 |
| ENB | PA7 / TIM3_CH2 | 오른쪽 PWM |
| IN3 | PA8 | 오른쪽 방향 |
| IN4 | PB10 | 오른쪽 방향 |

## PCA9685 채널

| 채널 | 관절 |
|---|---|
| 0 | Base |
| 1 | Shoulder |
| 2 | Arm/Elbow |
| 3 | Wrist Up/Down |
| 4 | Wrist Rotation |
| 5 | Gripper |

## 배선 원칙

- PCA9685 `VCC`는 STM32 논리 전원에 연결합니다.
- PCA9685 `V+`는 별도의 6V 고전류 서보 전원에 연결합니다.
- 모터, 서보, 제어부 GND는 공통으로 연결합니다.
- 고전류 전원은 STM32 보드를 경유하지 않습니다.
- ENA/ENB PWM 제어 시 L298N 점퍼를 제거합니다.
