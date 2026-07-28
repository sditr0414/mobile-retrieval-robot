# 모바일 물품 회수 로봇 프로젝트 요약

이 문서는 저장소의 전체 설계, 현재 구현 상태, 하드웨어 구성과 펌웨어 작업 원칙을 한곳에서 확인하기 위한 요약 문서입니다. 세부 배선과 통신 형식은 `docs/` 아래의 원본 문서를 우선 확인합니다.

## 1. 프로젝트 개요

- 목표: 스마트폰으로 제어하는 5자유도 로봇팔과 4WD 이동 플랫폼 구현
- 기준 보드: NUCLEO-F411RE
- MCU: STM32F411RETx
- 개발 환경: VS Code STM32 확장
- 빌드 시스템: STM32CubeMX 네이티브 CMake + GCC
- 운영체제: FreeRTOS CMSIS-RTOS2

최상위 `app`에는 Flutter Android 제어 앱, `simulator`에는 PyBullet
시뮬레이터가 있습니다. 로봇의 실측 모델 값은 확인 전까지 임의로 정하지 않고
TODO로 남깁니다. 출력 부품과 서보 질량은 `simulator/models/mass_properties.json`
기준으로 확정했으며, 링크별 무게중심과 관성텐서는 아직 TODO입니다.

현재 `firmware/mobile_retrieval_robot`에는 STM32CubeMX 생성 코드와 아래 5개
애플리케이션 라이브러리가 구현되어 있습니다.

- `bluetooth`
- `servo_driver`
- `robot_arm`
- `drive_4wd`
- `teaching_storage`

소스 파일과 `App/Inc` 경로는 최상위 `CMakeLists.txt`에 등록되어 있습니다.
Bluetooth circular DMA 파서, 6채널 로봇팔, 4WD와 12개 티칭 시퀀스의
Flash 저장이 FreeRTOS 태스크에 연결되어 있습니다. 실제 관절 보정값은
`docs/hardware/servo-calibration.md`에 기록하며 측정되지 않은 값은 TODO로
유지합니다.

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
- DMA 수신 버퍼: 64 또는 128바이트
- 앱 기준 고정 11바이트 바이너리 패킷 스트림 파싱
- ISR에서는 수신 데이터 보관만 수행하고 명령 파싱과 장치 제어를 하지 않음

## 6. Bluetooth 프로토콜

Bluetooth 규격은 **Flutter 앱을 기준으로 한 고정 11바이트 바이너리
패킷**입니다. ASCII 문자열 명령은 구현하지 않습니다.

```text
[Header, Mode, Data0, Data1, Data2, Data3, Data4, Data5, Data6,
 Checksum, Tail]
```

- `Header`: `0xAA`
- `Tail`: `0x55`
- `Checksum`: Byte 1부터 Byte 8까지의 합을 256으로 나눈 나머지
- 유효 Mode: `0x00` Drive, `0x01` Robot Arm, `0x02` Teaching
- 스트림 동기화가 깨지면 다음 `0xAA`를 검색해 복구
- Header, Tail, Mode, Checksum과 각 Mode의 값 범위를 모두 검증
- 잘못된 패킷은 장치를 제어하지 않고 폐기

### Mode 0: Drive

```text
[0xAA, 0x00, L_DIR, L_PWM, R_DIR, R_PWM, Control, 0x00, 0x00,
 checksum, 0x55]
```

- 방향: 앱 기준 `0` 전진, `1` 후진
- PWM: `0~255`, TIM3의 `0~999` 범위로 변환
- PWM이 `0`이면 해당 모터 정지
- `Control=0` 일반, `1` E-STOP 잠금, `2` E-STOP 해제
- E-STOP과 해제 패킷의 방향·PWM은 모두 `0`
- Data5와 Data6은 반드시 `0x00`

### Mode 1: Robot Arm

```text
[0xAA, 0x01, Base, Shoulder, Elbow, WristTilt, WristRotate, Gripper,
 Control, checksum, 0x55]
```

- Data0~Data5는 6개 관절의 명령 각도
- `Control=0` 이동, `1` 지정 홈 자세로 출력 활성화, `2` 출력 차단
- 부팅 직후 출력은 꺼져 있으며 보정 설정 6개가 모두 등록되어야 활성화
- 실제 측정 위치가 아니라 마지막으로 요청된 각도
- 관절별 보정 범위를 벗어나면 패킷을 적용하지 않음

### Mode 2: Teaching

앱은 12개 시퀀스와 시퀀스당 최대 30개 웨이포인트를 사용합니다.

- `Command=2`: 선택한 시퀀스 재생
- `Command=3`: 선택한 시퀀스를 RAM과 Flash에서 초기화
- `Command=4`: START, 전반부 3각도, 후반부 3각도, COMMIT 순서로 업로드
- 모든 웨이포인트 조각을 받은 경우에만 Sector 7에 저장
- 저장 결과는 같은 11바이트 형식의 ACK로 앱에 응답
- 저장된 시퀀스를 부팅 직후 자동 실행하지 않음

세부 바이트 배열과 검증 규칙의 기준 문서는 `docs/protocol/bluetooth-protocol.md`입니다.

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
- 최종 6채널 제어에서는 STM32 TIM으로 서보 PWM을 직접 만들지 않음
- 실제 서보 위치는 읽을 수 없으므로 마지막 명령 각도를 상태로 관리
- 관절별 최소각, 최대각, 홈, 방향, 최소·최대 펄스를 적용
- Base부터 Wrist Rotate까지 앱과 시리얼 표시 각도는 `-90~+90도`를 사용
- Gripper는 앱과 시리얼에서 `0%` 최대 열림, `100%` 최대 닫힘으로 표시
- Bluetooth 패킷과 Flash에는 각도 또는 Gripper 퍼센트를 변환한 `0~180`을 저장
- Base는 조립 방향에 맞춰 펌웨어 출력 방향을 반전
- Gripper도 조립 방향에 맞춰 펌웨어 출력 방향을 반전
- 홈 자세는 관절 `0도`, Gripper `0%`이며 패킷값은 `[90,90,90,90,90,0]`
- 수동 이동, 홈 복귀와 티칭 재생은 `1도/20 ms`, 약 `50도/s`로 속도를 제한
- Shoulder는 측정값을 환산한 633/1466/2322 us 사용
- Elbow는 측정값을 환산한 633/1500/2388 us 사용
- Wrist Rotate는 측정값을 환산한 500/1500/2533 us 사용
- Wrist Tilt만 수동 보정을 위해 500/1500/2500 us를 임시 기준으로 사용
- 측정된 Gripper 보정값은 최대 닫힘 1140 us, 열림 1840 us
- 저장된 자세로 부팅 직후 자동 이동하지 않음
- 일반 서보에는 위치 피드백이 없으므로 전원 투입 후 첫 활성화의 실제 시작
  위치와 이동 속도는 보장할 수 없음

## 10. 차량 안전 원칙

- GPIO와 PWM은 0으로 초기화
- Bluetooth 차량 명령이 300~500 ms 동안 없으면 정지
- 앱 E-STOP은 잠금 상태로 유지하며 일반 명령보다 우선 처리
- E-STOP 한 패킷으로 차량과 PCA9685 서보 출력을 함께 차단
- Flash 삭제와 기록 중에는 차량 명령을 적용하지 않음
- 물리 비상정지 장치는 모터와 서보 전원을 독립적으로 차단
- `driveTask`에서 긴 지연을 사용하지 않음
- 로봇팔 전개 시 차량 속도 제한값은 실제 시험 후 확정

## 11. 티칭 데이터 저장

- STM32 내부 Flash Sector 7 사용
- 시작 주소: `0x08060000`
- 크기: 128 KiB
- 애플리케이션 링커 FLASH 길이: 384 KiB로 제한됨
- 저장 형식에 `magic`, `version`, 12개 시퀀스, 웨이포인트 수와 `CRC` 포함
- 업로드 자세는 임시 RAM에 먼저 모으고 COMMIT 명령에서만 Flash에 기록
- 기록 후 Flash 내용을 다시 읽어 검증
- 매 관절 이동 명령마다 Flash에 기록하지 않음
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
- 스마트폰 앱 안내: `app/README.md`
- PyBullet 시뮬레이터 안내: `simulator/README.md`
- 펌웨어 안내: `firmware/README.md`
- 부품 목록: `docs/hardware/bom.md`
- 핀맵: `docs/hardware/pin-map.md`
- 서보 보정값: `docs/hardware/servo-calibration.md`
- Bluetooth 프로토콜: `docs/protocol/bluetooth-protocol.md`
- 협업 지침: `docs/collaboration/CONTRIBUTING.md`
