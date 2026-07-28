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

## USART2 디버그·Teleplot

| 기능 | STM32 핀 | 연결 대상 |
|---|---|---|
| TX | PA2 | NUCLEO ST-LINK Virtual COM Port |
| RX | PA3 | NUCLEO ST-LINK Virtual COM Port |

- 설정: `115200 baud`, 8 data bits, no parity, 1 stop bit
- Teleplot은 PC에 표시된 ST-LINK Virtual COM Port를 선택합니다.

## L298N

| 기능 | STM32 핀 | 비고 |
|---|---|---|
| ENA | PA6 / TIM3_CH1 | 왼쪽 PWM |
| IN1 | PB6 | 왼쪽 방향 |
| IN2 | PB7 | 왼쪽 방향 |
| ENB | PA7 / TIM3_CH2 | 오른쪽 PWM |
| IN3 | PA8 | 오른쪽 방향 |
| IN4 | PB10 | 오른쪽 방향 |

## 전방 초음파 센서

HC-SR04 계열을 기준으로 한 권장 예시입니다.

| 기능 | STM32 핀 | 비고 |
|---|---|---|
| TRIG | PC8 | GPIO Output, 약 10 µs 펄스 출력 |
| ECHO | PA0 / TIM2_CH1 | Input Capture 권장 |
| VCC | 외부 5V | STM32 5V 핀에서 고전류 부하와 함께 공급하지 않음 |
| GND | 공통 GND | STM32·센서·모터 드라이버 공통 접지 |

### ECHO 전압 주의

일반적인 HC-SR04의 ECHO는 5V 레벨일 수 있으므로 STM32 입력에 직접 연결하지 않습니다.

```text
HC-SR04 ECHO ── 1 kΩ ──┬── STM32 PA0
                       │
                      2 kΩ
                       │
                      GND
```

위 저항값은 약 3.3V 수준으로 낮추는 단순 분압 예시입니다. 실제 사용 저항값의 오차와 센서 모듈 사양을 확인합니다.

### 설치 위치

- 차체 전면 중앙에 설치합니다.
- 로봇팔이 접힌 상태와 전개되는 경로를 함께 감시할 수 있는 높이로 맞춥니다.
- 바닥을 향하거나 차체 프레임이 측정 범위에 들어오지 않도록 수평을 조정합니다.
- 센서 주변에 진동 흡수용 고정부를 적용합니다.

## PCA9685 채널

| 채널 | 관절 |
|---|---|
| 0 | Base |
| 1 | Shoulder |
| 2 | Arm/Elbow |
| 3 | Wrist Up/Down |
| 4 | Wrist Rotation |
| 5 | Gripper |

## 로봇팔·차량 동작 인터록

현재 펌웨어는 로봇팔 수동 이동과 티칭 재생을 시작하기 전에 차량 PWM을 0으로
만듭니다. 관절 이동 또는 웨이포인트 유지가 진행되는 동안에는 새 주행 명령을
적용하지 않으며, 로봇팔 동작이 끝난 후 들어온 주행 명령부터 허용합니다.

### 후속 초음파 전개 인터록

아래 초음파 기반 조건은 후속 구현 범위이며 현재 펌웨어에는 아직 연결되지
않았습니다.

```text
vehicle_speed_command == 0
&& ultrasonic_valid == true
&& front_distance_cm >= ARM_DEPLOY_DISTANCE_CM
&& safe_sample_count >= 3
&& estop == false
```

프로젝트 기준값은 `ARM_DEPLOY_DISTANCE_CM = 20`입니다. 전방 거리가 20 cm 미만이면 로봇팔 전개를 금지하고, 20 cm 이상인 측정값이 연속 3회 확인될 때만 전개를 허용합니다.

센서 응답 시간 초과, 범위 밖 값, 급격한 값 변화가 발생하면 안전하지 않은 상태로 처리합니다. 이미 로봇팔이 전개된 뒤 장애물이 접근하면 자동으로 팔을 급히 접기보다 차량을 정지하고 앱에 경고를 보내는 것을 우선합니다.

## 배선 원칙

- PCA9685 `VCC`는 STM32 논리 전원에 연결합니다.
- PCA9685 `V+`는 별도의 6V 고전류 서보 전원에 연결합니다.
- 모터, 서보, 제어부 GND는 공통으로 연결합니다.
- 고전류 전원은 STM32 보드를 경유하지 않습니다.
- ENA/ENB PWM 제어 시 L298N 점퍼를 제거합니다.
- 초음파 센서 ECHO의 입력 전압을 STM32 허용 범위로 낮춥니다.
