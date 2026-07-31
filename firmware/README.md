# STM32F411 펌웨어

NUCLEO-F411RE용 STM32CubeMX CMake 프로젝트입니다. HAL과 FreeRTOS
CMSIS-RTOS2를 사용합니다.

## 구현

| 모듈 | 역할 |
|---|---|
| `bluetooth` | USART1 circular DMA와 16바이트 프레임 검증 |
| `servo_driver` | PCA9685 50 Hz PWM |
| `robot_arm` | 6채널 S-curve 이동과 시퀀스 재생 |
| `drive_4wd` | L298N, 타임아웃과 상대 Yaw PID |
| `teaching_storage` | 티칭·설정 Sector 7 저장 |
| `imu` | MPU6050 보정·필터·상대 Yaw |

```text
mobile_retrieval_robot/
├─ App/Inc, App/Src   애플리케이션 모듈
├─ Core/              CubeMX 사용자 코드
├─ Drivers/           STM32 HAL
├─ Middlewares/       FreeRTOS
├─ cmake/             CubeMX CMake
├─ CMakeLists.txt
├─ CMakePresets.json
└─ mobile_retrieval_robot.ioc
```

## CubeMX 기준

| 항목 | 설정 |
|---|---|
| HSE / SYSCLK | 8 MHz bypass / 100 MHz |
| USART1 | PA9/PA10, 9600 8-N-1 |
| RX DMA | DMA2 Stream2, Circular, 128바이트 버퍼 |
| I2C1 | PB8/PB9, 100 kHz |
| TIM2 | PSC=4, ARR=999, PA0/PA1, 20 kHz |
| 모터 초기 출력 | GPIO/PWM 모두 0 |

PCA9685가 서보 PWM 50 Hz를 생성합니다. PA2/PA3와 MPU6050 INT는 사용하지
않습니다.

## 실행 구조

| 태스크 | 우선순위 | 역할 |
|---|---|---|
| Bluetooth | AboveNormal | DMA 파싱과 큐 전달 |
| Arm | Normal | 로봇팔, 티칭, 설정, Flash |
| Drive | High | 모터, 타임아웃, PID |
| IMU | BelowNormal | 3초 보정과 20 ms 측정 |
| Status | Low | PID·IMU 상태 송신 |

PCA9685와 MPU6050은 `i2cMutex`, Flash는 `flashMutex`로 직렬화합니다. ISR은
파싱, 제어와 Flash 작업을 하지 않습니다.

## 제어와 안전

- 부팅 시 E-STOP 잠금, 차량 정지, 서보 출력 OFF.
- 차량 명령 500 ms 타임아웃.
- 로봇팔·티칭·Flash·IMU 보정 중 차량 인터록.
- 모터 역전 전 중립 정지.
- PCA9685 오류 시 1초마다 재초기화하며 다른 기능은 유지.
- Assert 전 차량 GPIO/PWM을 0으로 설정.

로봇팔은 20 ms 5차 S-curve와 앱 속도 `50~100%`를 사용합니다. 원점은
`[90,90,90,90,90,0]`, 이동 자세는 티칭 시퀀스 9입니다. 시퀀스 9 재생은
현재 Gripper를 유지합니다. 위치 피드백이 없어 첫 활성화의 실제 시작 위치와
물리 속도는 보장하지 않습니다.

서보 보정값은 Sector 7을 우선합니다. Flash가 비었을 때 일반 관절은
`700/1500/2300 µs`, Gripper는 `1200/1500/1800 µs` 안전 초기값을 사용하며
실측 끝점을 뜻하지 않습니다.

티칭 화면의 편집본은 업로드 임시 버퍼에서 재생하므로 Sector 7을 지우거나
기록하지 않습니다. 저장된 시퀀스 조회와 차량 전용 동작은 부팅 시 Flash에서
읽은 RAM 원본을 사용합니다.

MPU6050은 AD0=GND(`0x68`), 20 ms polling, 2 Hz 필터와 Z축 0.15 dps
deadband를 사용합니다. PID는 직진·후진 구간의 상대 Yaw에만 적용합니다.

## Flash

- Sector 7: `0x08060000`, 128 KiB.
- 링커 FLASH: 384 KiB.
- 형식 버전 8, CRC와 기록 후 재검증.
- 이름을 포함한 12개 시퀀스, PID, 서보 3점 보정, 호환용 이동 자세 5바이트.
- 버전 2~7은 웨이포인트와 설정을 보존하고 기본 이름을 추가하며, 버전 1은
  Gripper 의미가 달라 거부.
- 설정 조각은 RAM에 모두 받은 뒤 COMMIT에서 한 번 기록.
- Sector 삭제·기록 중 전원을 끄면 저장 이미지가 무효화될 수 있으므로 완료 ACK 후
  전원을 차단.

정확한 패킷과 저장 ACK는
[Bluetooth 프로토콜](../docs/protocol/bluetooth-protocol.md)을 따릅니다.

## 빌드

```powershell
cd firmware/mobile_retrieval_robot
cmake --preset Debug
cmake --build --preset Debug
```

## 남은 실기 튜닝

- Wrist Tilt 포함 관절 안전 끝점
- 실제 차체 PID 계수

전원, MPU6050 장착 방향과 L298N 전진 극성은 실물에 적용되어 있습니다.
구체적인 전원 모델·출력값은 저장소 문서에 기록하지 않았습니다.

관련 문서: [핀맵](../docs/hardware/pin-map.md),
[서보 보정](../docs/hardware/servo-calibration.md),
[IMU](../docs/hardware/imu.md).
