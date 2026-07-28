# STM32F411 제어 펌웨어

NUCLEO-F411RE에서 로봇팔과 4WD 이동 플랫폼을 제어하는 STM32CubeMX CMake 프로젝트입니다.

## 현재 상태

- STM32CubeMX 초기 프로젝트 생성 완료
- HAL, CMSIS-RTOS2, FreeRTOS 포함
- I2C1, USART1/2, USART1 RX DMA, TIM3 PWM, 모터 GPIO 생성 완료
- 애플리케이션 라이브러리 5개 구현 및 CMake target 등록
- circular DMA 파서와 FreeRTOS 3개 태스크 연결
- 12개 티칭 시퀀스의 Sector 7 저장 및 CRC/재읽기 검증 구현

## 1차 구현 범위

- HC-05 Bluetooth Classic SPP 11바이트 바이너리 패킷 수신
- PCA9685 기반 6채널 서보 제어
- 로봇팔 수동 관절 제어
- L298N 기반 좌우 모터 PWM 및 방향 제어
- Bluetooth 차량 명령 500 ms 타임아웃
- 잠금식 E-STOP과 로봇팔 출력 활성화 확인
- 티칭 자세 RAM 저장, 재생, 내부 Flash 영구 저장

후속 범위:

- 역기구학과 XYZ 제어
- IMU 방향 안정화
- 초음파 센서 인터록
- 휠 엔코더와 폐루프 PID

## 프로젝트 구조

```text
firmware/
├─ mobile_retrieval_robot/
│  ├─ App/
│  │  ├─ Inc/
│  │  │  ├─ bluetooth.h
│  │  │  ├─ servo_driver.h
│  │  │  ├─ robot_arm.h
│  │  │  ├─ drive_4wd.h
│  │  │  └─ teaching_storage.h
│  │  └─ Src/
│  │     ├─ bluetooth.c
│  │     ├─ servo_driver.c
│  │     ├─ robot_arm.c
│  │     ├─ drive_4wd.c
│  │     └─ teaching_storage.c
│  ├─ Core/
│  ├─ Drivers/
│  ├─ Middlewares/
│  ├─ cmake/
│  ├─ CMakeLists.txt
│  ├─ CMakePresets.json
│  ├─ mobile_retrieval_robot.ioc
│  └─ STM32F411xx_FLASH.ld
├─ .gitignore
└─ README.md
```

## 라이브러리 역할

| 라이브러리 | 역할 | 현재 상태 |
|---|---|---|
| `bluetooth` | UART DMA 스트림 처리, 11바이트 패킷 검증 | 구현 완료 |
| `servo_driver` | PCA9685 I2C 공용 서보 제어 | 구현 완료 |
| `robot_arm` | 6관절 수동 제어와 비동기 시퀀스 재생 | 구현 완료 |
| `drive_4wd` | L298N 방향 GPIO, TIM3 PWM과 타임아웃 | 구현 완료 |
| `teaching_storage` | 12개 시퀀스와 CRC 포함 Flash 저장 | 구현 완료 |

## Bluetooth 규격

펌웨어는 앱을 기준으로 고정 11바이트 바이너리 패킷만 지원합니다.

```text
[0xAA, Mode, Data0, Data1, Data2, Data3, Data4, Data5, Data6,
 Checksum, 0x55]
```

- Checksum: Byte 1~8 합의 하위 8비트
- Mode 0: 차량
- Mode 1: 로봇팔 6관절과 출력 활성화/차단
- Mode 2: 티칭과 재생
- 무효 패킷 이후 다음 `0xAA`를 검색해 동기화 복구
- UART 오류 시 태스크 문맥에서 circular DMA 재시작

로봇팔 이동 또는 티칭 재생을 시작하면 차량 PWM을 즉시 0으로 만들고
`arm_motion_inhibit`를 설정합니다. 이동과 웨이포인트 유지가 끝날 때까지
일반 주행 명령은 적용하지 않으며, 이후 들어오는 새 주행 패킷부터 허용합니다.

세부 바이트 배열은 `docs/protocol/bluetooth-protocol.md`를 기준으로 합니다.

## CubeMX 설정 점검

| 항목 | 현재 생성 상태 | 설계 기준 |
|---|---|---|
| USART1 | 9600, 8-N-1 | 일치 |
| USART1 RX DMA | DMA2 Stream2, Circular | 일치 |
| I2C1 | PB8/PB9, 100 kHz | 일치 |
| TIM3 | PSC=4, ARR=999 | 20 kHz, 일치 |
| Motor GPIO/PWM 초기값 | 0 | 일치 |
| BluetoothTask | AboveNormal, 384 words | 일치 |
| ArmTask | Normal, 384 words | 일치 |
| DriveTask | High, 256 words | 일치 |

### 클록 설정

NUCLEO 기본 HSE bypass 8 MHz에 맞춰 `HSE_VALUE=8 MHz`, `PLLM=4`,
`PLLN=100`, `PLLP=2`로 설정했습니다. SYSCLK와 APB2 TIM 클록은 100 MHz입니다.
서보 PWM 50 Hz는 PCA9685가 생성합니다.

### 아직 남은 하드웨어 확인

- 링커 FLASH는 384 KiB로 제한되어 Sector 7을 앱 링크 영역에서 제외함
- Wrist Tilt의 실제 펄스폭과 모든 관절의 조립 후 안전 각도
- L298N 좌우 모터의 실제 전진 극성

Shoulder는 `633/1466/2322 us`, Elbow는 `633/1500/2388 us`,
Wrist Rotate는 `500/1500/2533 us`의 측정 환산값을 사용합니다.
Wrist Tilt만 `500/1500/2500 us`를 임시 기준으로 사용하며 실제 끝단과
방향을 확인한 뒤 안전 펄스폭으로 교체합니다.

앱과 USART2 모니터에서 Base부터 Wrist Rotate까지는 `-90~+90도`입니다.
Gripper는 `0%` 최대 열림, `100%` 최대 닫힘으로 표시합니다. Bluetooth와
Flash에는 각도 또는 퍼센트를 변환한 `0~180` 값이 저장됩니다. Base,
Elbow, Wrist Tilt와 Gripper는 조립 방향에 맞춰 출력 방향을 반전합니다. Base의
측정값은 `500/1500/2500 us`, Gripper의 측정값은
`1140/1490/1840 us`입니다.

일반 관절의 수동 이동, 홈 복귀와 티칭 재생은 `1도/20 ms`, 약 `50도/s`
이하로 보간합니다. Gripper는 원래 속도인 패킷 `1단계/15 ms`로 이동합니다.
홈/주행 준비 자세는 Base `0도`, Shoulder `-50도`, Elbow `+90도`,
Wrist Tilt `0도`, Wrist Rotate `+60도`, Gripper 최대 열림 `0%`이며
패킷값은 `[90,40,180,90,150,0]`입니다.
출력을 껐다 다시 켤 때는 마지막 명령 위치에서 홈까지 천천히 이동합니다.
단, 일반 서보에는 위치 피드백이 없으므로 전원 투입 후 첫 활성화는 실제
시작 위치를 알 수 없고 첫 홈 이동의 물리 속도를 보장할 수 없습니다.

Gripper 패킷 의미가 변경되어 티칭 Flash 저장 버전은 2입니다. 버전 1 데이터는
안전을 위해 불러오지 않으므로 앱에서 새 의미로 웨이포인트를 다시 저장해야 합니다.

## Teleplot 로봇팔 모니터

USART2는 `115200 baud`, 8-N-1로 설정되어 있으며 NUCLEO ST-LINK Virtual
COM Port를 통해 Teleplot에 연결합니다. 펌웨어는 저우선순위
`telemetryTask`에서 100 ms마다 `>이름:값` 형식으로 출력하므로 로봇팔 제어
태스크의 서보 보간을 UART 전송으로 막지 않습니다.

관절마다 다음 그래프 변수가 생성됩니다.

```text
arm.base.current
arm.base.target
arm.base.pulse_us
```

`base` 대신 `shoulder`, `elbow`, `wrist_tilt`, `wrist_rotate`, `gripper`가
각 관절 이름으로 사용됩니다. Gripper의 현재값과 목표값 이름에는
`current_percent`, `target_percent`를 사용합니다.

상태 변수:

| 변수 | 의미 |
|---|---|
| `arm.enabled` | 0 출력 차단, 1 출력 활성화 |
| `arm.state_code` | 0 차단, 1 대기, 2 이동, 3 웨이포인트 유지 |
| `arm.state` | `DISABLED`, `IDLE`, `MOVING`, `HOLDING` 텍스트 |
| `arm.sequence_active` | 티칭 시퀀스 재생 여부 |
| `arm.waypoint` | 재생 중인 웨이포인트 번호, 재생 중이 아니면 0 |

Teleplot에서 각 관절의 `current`와 `target`을 같은 그래프에 추가하면 목표
추종과 이동 속도를 확인할 수 있습니다. `pulse_us`는 PCA9685에 마지막으로
명령한 펄스폭이며 출력 차단 중에는 0으로 표시합니다. 일반 서보에는 위치
센서 피드백이 없으므로 `current`는 실제 측정 위치가 아니라 펌웨어가 마지막으로
출력한 명령 위치입니다.

## 빌드

```bash
cd firmware/mobile_retrieval_robot
cmake --preset Debug
cmake --build --preset Debug
```

## 구현 전 확인

1. `AGENTS.md`에서 1차 범위와 안전 원칙을 확인합니다.
2. `docs/hardware/pin-map.md`에서 핀 배치를 확인합니다.
3. `docs/protocol/bluetooth-protocol.md`에서 앱 기준 11바이트 패킷을 확인합니다.
4. HSE bypass 8 MHz와 PLLM=4 설정을 확인합니다.
5. 서보 보정값과 모터 극성은 실제 하드웨어 측정 전까지 `TODO`로 둡니다.
