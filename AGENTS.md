# 모바일 물품 회수 로봇 프로젝트 요약

이 문서는 저장소의 전체 설계, 현재 구현 상태, 하드웨어 구성과 펌웨어 작업 원칙을 한곳에서 확인하기 위한 요약 문서입니다. 세부 배선과 통신 형식은 `docs/` 아래의 원본 문서를 우선 확인합니다.

## 1. 프로젝트 개요

- 목표: 스마트폰으로 제어하는 5자유도 로봇팔과 4WD 이동 플랫폼 구현
- 기준 보드: NUCLEO-F411RE
- MCU: STM32F411RETx
- 개발 환경: VS Code STM32 확장
- 빌드 시스템: STM32CubeMX 네이티브 CMake + GCC
- 운영체제: FreeRTOS CMSIS-RTOS2

현재 `firmware/mobile_retrieval_robot`에는 STM32CubeMX가 생성한 초기 펌웨어와 아래 5개 애플리케이션 라이브러리의 빈 헤더·소스 파일만 있습니다.

- `bluetooth`
- `servo_driver`
- `robot_arm`
- `drive_4wd`
- `teaching_storage`

빈 소스 파일과 `App/Inc` 경로는 최상위 `CMakeLists.txt`에 등록되어 있습니다. 애플리케이션 함수 선언과 구현은 아직 없습니다.

## 2. 1차 펌웨어 범위

1차 단계에서 구현할 기능:

- HC-05 Bluetooth 제어
- PCA9685 기반 서보 제어
- 그리퍼를 포함한 로봇팔 6채널 제어
- L298N 기반 4WD 좌우 모터 제어
- 내부 Flash를 이용한 티칭 자세 저장과 재생
- Bluetooth 명령 타임아웃 시 차량 정지

후속 단계로 남길 기능:

- IMU
- 초음파 센서
- 휠 엔코더
- 폐루프 휠 PID
- 역기구학과 `ARM,XYZ` 제어

## 3. 하드웨어 구성

| 구분 | 부품 | 수량 | 용도 |
|---|---|---:|---|
| 제어 | NUCLEO-F411RE | 1 | 전체 시스템 제어 |
| 통신 | HC-05 B36 | 1 | Bluetooth Classic SPP |
| 서보 제어 | PCA9685 | 1 | 50 Hz 서보 PWM 생성 |
| 고토크 서보 | MG996R | 3 | Base, Shoulder, Elbow |
| 소형 서보 | SG90 | 3 | Wrist Tilt, Wrist Rotate, Gripper |
| 모터 드라이버 | L298N | 1 | 좌우 DC 모터 그룹 제어 |
| DC 기어모터 | DC 모터 | 4 | 좌우 2개씩 한 그룹으로 연결 |

서보와 모터 전원은 STM32 보드에서 직접 공급하지 않습니다. 제어부와 구동부의 GND는 공통으로 연결하고, 고전류 전원 경로는 별도로 분기합니다.

## 4. 핀 배치

| 기능 | 핀 |
|---|---|
| I2C1 SCL | PB8 |
| I2C1 SDA | PB9 |
| USART1 TX / HC-05 RX | PA9 |
| USART1 RX / HC-05 TX | PA10 |
| TIM3 CH1 / L298N ENA | PA6 |
| TIM3 CH2 / L298N ENB | PA7 |
| MOTOR_L_IN1 | PB6 |
| MOTOR_L_IN2 | PB7 |
| MOTOR_R_IN1 | PA8 |
| MOTOR_R_IN2 | PB10 |
| USART2 TX 디버그 | PA2 |
| USART2 RX 디버그 | PA3 |
| SWD | PA13, PA14 |

## 5. 클록과 주변장치 기준

### 시스템 클록

- HSE: NUCLEO 기본 하드웨어의 8 MHz Bypass Clock Source
- SYSCLK: 100 MHz
- HCLK: 100 MHz
- PCLK1: 50 MHz
- PCLK2: 100 MHz

### TIM3 모터 PWM

- 주파수: 20 kHz
- Prescaler: 4
- Auto-reload: 999
- 초기 Pulse: 0
- PWM 범위: 0~999

### USART1 Bluetooth

- 9600 baud
- 8 data bits
- No parity
- 1 stop bit
- RX DMA circular 또는 Receive-to-Idle DMA 사용
- ISR에서는 수신 데이터 보관만 수행하고 명령 파싱과 장치 제어를 하지 않음

## 6. Bluetooth 프로토콜

현재 `docs/protocol/bluetooth-protocol.md`는 줄바꿈으로 끝나는 ASCII 명령 형식을 정의합니다.

```text
CATEGORY,COMMAND,VALUE...\n
```

주요 명령:

```text
ARM,J,<joint>,<angle>
ARM,HOME
TEACH,START
TEACH,ADD,<duration_ms>
TEACH,STOP
TEACH,SAVE,<slot>
SEQ,PLAY,<slot>
SEQ,PAUSE
SEQ,STOP
SEQ,CLEAR,<slot>
DRIVE,<left_speed>,<right_speed>
ESTOP
STATUS?
PING
```

이전 펌웨어 설계에는 10바이트 바이너리 패킷 초안도 존재했습니다. ASCII 규격과 바이너리 규격을 혼합 구현하지 않으며, 실제 파서 구현 전에 사용할 규격을 문서에서 하나로 확정해야 합니다. 현재 저장소에서는 `docs/protocol/bluetooth-protocol.md`를 우선 기준으로 봅니다.

## 7. FreeRTOS 구조

### 태스크

| 태스크 | 권장 우선순위 | 초기 스택 | 역할 |
|---|---|---:|---|
| `bluetoothTask` | AboveNormal | 384 words | DMA 수신 처리, 명령 검증, 큐 전달 |
| `armTask` | Normal | 384 words | 로봇팔 제어, 티칭 재생, Flash 작업 |
| `driveTask` | High | 256 words | 모터 제어와 통신 타임아웃 감시 |

### RTOS 객체

- `armQueue`: 길이 8
- `driveQueue`: 길이 4
- `i2cMutex`
- `flashMutex`

Mutex는 ISR에서 사용하지 않습니다. Flash 삭제와 기록은 반드시 태스크 문맥에서 실행합니다.

## 8. 라이브러리 역할

```text
ArmTask
└─ robot_arm
   └─ servo_driver
      └─ HAL I2C / PCA9685

DriveTask
└─ drive_4wd
   └─ TIM3 / GPIO / L298N

BluetoothTask
└─ bluetooth
   └─ armQueue / driveQueue

ArmTask
└─ teaching_storage
   └─ 내부 Flash Sector 7
```

## 9. 서보 제어 원칙

- PCA9685 기본 I2C 주소: `0x40`
- PCA9685 PWM 주파수: 50 Hz
- 채널 0~5를 Base, Shoulder, Elbow, Wrist Tilt, Wrist Rotate, Gripper 순서로 사용
- STM32 TIM 채널로 서보 PWM을 직접 만들지 않음
- 실제 서보 위치는 읽을 수 없으므로 마지막 명령 각도를 상태로 관리
- 관절별 최소각, 최대각, 홈, 방향, 최소·최대 펄스를 적용
- 측정되지 않은 보정값은 임의로 결정하지 않고 `TODO`로 남김
- 저장된 자세로 부팅 직후 자동 이동하지 않음

## 10. 차량 안전 원칙

- GPIO와 PWM은 0으로 초기화
- Bluetooth 차량 명령이 300~500 ms 동안 없으면 정지
- 앱 ESTOP은 일반 명령보다 우선 처리
- 물리 비상정지 장치는 모터와 서보 전원을 독립적으로 차단
- `driveTask`에서 긴 지연을 사용하지 않음
- 로봇팔 전개 시 차량 속도 제한값은 실제 시험 후 확정

## 11. 티칭 데이터 저장

- STM32 내부 Flash Sector 7 사용
- 시작 주소: `0x08060000`
- 크기: 128 KiB
- 애플리케이션 링커 FLASH 길이: 384 KiB로 제한 예정
- 저장 형식에 `magic`, `version`, `frame count`, `frames`, `CRC` 포함
- 자세는 RAM에 먼저 추가하고 명시적인 저장 명령에서만 Flash에 기록
- 기록 후 Flash 내용을 다시 읽어 검증
- 매 조이스틱 입력마다 Flash에 기록하지 않음
- 실제 위치가 아닌 명령된 각도를 저장

## 12. 코딩 원칙

- C11 호환 임베디드 C 사용
- RTOS 시작 후 동적 메모리 할당은 가급적 사용하지 않음
- HAL과 RTOS 함수 반환값 확인
- ISR 콜백은 짧게 유지
- ISR에서 패킷 파싱, 모터 제어, Flash 작업을 하지 않음
- 고정 폭 정수형 사용
- CubeMX USER CODE 영역 보존
- 애플리케이션 코드는 `App/Inc`, `App/Src`에 배치
- 새 소스는 최상위 CMake target에 등록
- `cmake/stm32cubemx` 생성 파일은 필요한 경우가 아니면 수정하지 않음
- 확인되지 않은 하드웨어 값은 임의로 정하지 않고 `TODO`로 기록

## 13. 권장 구현 순서

1. CMake configure와 초기 빌드 확인
2. 공용 메시지 타입과 RTOS 객체 정의
3. `drive_4wd` 구현
4. `servo_driver` 구현
5. `robot_arm` 구현
6. Bluetooth DMA 스트림 파서 구현
7. 큐와 태스크 연결
8. 차량 통신 타임아웃 구현
9. 티칭 RAM 모델 구현
10. Flash 저장과 검증 구현
11. 호스트에서 테스트 가능한 프로토콜 함수 작성

## 14. 관련 문서

- 프로젝트 소개: `README.md`
- 펌웨어 안내: `firmware/README.md`
- 부품 목록: `docs/hardware/bom.md`
- 핀맵: `docs/hardware/pin-map.md`
- Bluetooth 프로토콜: `docs/protocol/bluetooth-protocol.md`
- 협업 지침: `docs/collaboration/CONTRIBUTING.md`
