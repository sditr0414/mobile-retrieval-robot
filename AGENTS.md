# 프로젝트 작업 기준

이 문서는 저장소 전체의 구현 기준입니다. 세부 값은 아래 기준 문서를 우선합니다.

- 핀·전원: `docs/hardware/pin-map.md`
- 부품: `docs/hardware/bom.md`
- 서보 실측값: `docs/hardware/servo-calibration.md`
- PCA9685·S-curve: `docs/hardware/servo-control.md`
- IMU: `docs/hardware/imu.md`
- 상대 Yaw PID: `docs/hardware/yaw-pid.md`
- Bluetooth 바이트 배열: `docs/protocol/bluetooth-protocol.md`

## 프로젝트

- 보드: NUCLEO-F411RE / STM32F411RETx
- 펌웨어: STM32CubeMX CMake, HAL, FreeRTOS CMSIS-RTOS2
- 앱: Flutter Android, HC-05 Bluetooth Classic SPP
- 시뮬레이터: PyBullet
- 기능: 6채널 로봇팔, 4WD, 티칭, MPU6050 상대 Yaw PID
- 발전 방향: 장애물 감지, 휠 엔코더, 절대 방향 보정, 역기구학, XYZ 제어

```text
app/        Flutter 앱
firmware/   STM32 펌웨어
simulator/  PyBullet 시뮬레이터
docs/       설계 기준
```

## 하드웨어 기준

| 장치 | 연결 |
|---|---|
| PCA9685 | I2C1 PB8/PB9 |
| MPU6050 | I2C3 PA8/PC9 |
| HC-05 | USART1 PA9/PA10, 9600 8-N-1 |
| L298N ENA/ENB | TIM2 PA0/PA1, 20 kHz |
| L298N 방향 | PC0, PC1, PC2, PC3 |
| PCA9685 채널 0~5 | Base, Shoulder, Elbow, Wrist Tilt, Wrist Rotate, Gripper |

- HSE bypass 8 MHz, SYSCLK 100 MHz.
- PCA9685는 50 Hz이며 `VCC`는 논리 전원, `V+`는 XL4015 서보 전원입니다.
- 서보용·차량용 2S 팩의 양극은 분리하고 GND만 공통으로 연결합니다.
- 서보·모터 고전류는 STM32 보드를 경유하지 않습니다.
- PA0/PA1 PWM을 사용하므로 L298N ENA/ENB 점퍼를 제거합니다.
- PCA9685 `V+`–GND에 1000 µF 전해 커패시터를 사용합니다.
- BMS, 퓨즈와 물리 비상정지는 현재 구성에 없습니다.
- STM32 VIN용 DC-DC와 MPU6050 전원은 실물에 적용되어 있으며, 모델·출력값은 문서에 미기록 상태입니다.

## Bluetooth 기준

- 앱이 규격의 기준입니다.
- 프레임은 `[0xAA, Mode, Data0..Data11, Checksum, 0x55]` 고정 16바이트입니다.
- Checksum은 Byte 1~13 합의 하위 8비트입니다.
- Mode: `0` Drive, `1` Arm, `2` Teaching, `3` Settings.
- USART1 RX는 128바이트 circular DMA를 사용합니다.
- ISR은 데이터와 오류 플래그만 남기고 파싱·제어·Flash 작업을 하지 않습니다.
- 무효 프레임은 폐기하고 다음 `0xAA`에서 동기화를 복구합니다.

바이트 위치, 범위와 ACK는 `docs/protocol/bluetooth-protocol.md`만 수정합니다.

## 동작 기준

- 부팅 시 E-STOP 잠금, 차량 PWM 0, 서보 출력 OFF.
- 유효한 주행 명령이 500 ms 없으면 차량 정지.
- 로봇팔 이동·티칭·Flash 작업·IMU 보정 중 차량 정지.
- 인터록 해제 전 수신한 주행 명령은 재사용하지 않음.
- 모터 방향 전환 전 한 패킷 동안 중립 정지.
- 로봇팔 원점은 5관절 `0°`, Gripper `0%`.
- 이동 자세는 티칭 시퀀스 9이며 재생 중 현재 Gripper 유지.
- 시퀀스 10/11/12는 잡기 위치/잡기/놓기.
- 시퀀스 1~8 이름은 변경 가능하고 9~12 이름은 용도와 함께 고정합니다.
- 티칭 화면 재생은 앱 편집본을 임시 RAM에 전송하며 Flash를 쓰지 않습니다.
- 저장하지 않고 다른 시퀀스를 선택하면 편집본을 버리고 Flash 원본을 자동 조회합니다.
- 차량 화면의 9~12번 전용 동작은 항상 Flash 저장본을 재생합니다.
- 저장본 재생은 마지막 자세 조회부터 작업을 잠그며 조회 실패 시 재생하지 않습니다.
- 일반 관절은 앱 `-90~+90°`, Gripper는 열림 `0%`~닫힘 `100%`.
- 서보는 20 ms S-curve와 앱 속도 `50~100%`를 사용합니다.
- 서보 위치는 측정값이 아니라 마지막 명령값입니다.
- 시퀀스 완료 ACK 뒤 마지막 명령 자세를 수동 제어 화면에 반영합니다.
- 앱 주행 PWM은 최대 `220`이며 PID 직진 판정 범위는 기본 `0.10`, 유지
  히스테리시스는 `+0.04`입니다.

## IMU와 PID

- MPU6050: AD0=GND(`0x68`), INT 미사용, 20 ms polling.
- 부팅·재연결 후 정지 상태에서 3초 자이로 영점 보정.
- 2 Hz 저역통과 필터, Z축 0.15 dps deadband.
- 직진·후진 구간 시작 시 현재 상대 Yaw를 목표로 사용.
- 회전·정지·방향 전환·오류·E-STOP에서는 PID를 초기화합니다.
- 자력계가 없으므로 절대 Yaw를 보정하지 않습니다.

## Flash

- Sector 7: `0x08060000`, 128 KiB.
- 앱 링커 FLASH: 384 KiB.
- 저장 형식: 버전 8, CRC 포함.
- 데이터: 이름을 포함한 12개 시퀀스, PID, 서보 3점 보정, 호환용 이동 자세
  5바이트.
- 조각을 RAM에 모두 받은 뒤 한 번만 기록하고 다시 읽어 검증합니다.
- 이동 명령마다 Flash에 기록하지 않습니다.
- 이전 버전 변환 규칙은 기존 데이터 보존을 위해 삭제하지 않습니다.

## FreeRTOS

| 태스크 | 우선순위 | 스택 | 역할 |
|---|---|---:|---|
| Bluetooth | AboveNormal | 384 words | DMA 파싱·큐 전달 |
| Arm | Normal | 384 words | 서보·티칭·Flash |
| Drive | High | 256 words | 모터·타임아웃·PID |
| IMU | BelowNormal | 512 words | 보정·20 ms 측정 |
| Status | Low | 256 words | PID·IMU 상태 전송 |

`armQueue` 8, `driveQueue` 4, PCA9685용 `i2cMutex`, MPU6050용
`imuI2cMutex`, `flashMutex`를 사용합니다. Mutex와 Flash 작업은 태스크
문맥에서만 수행합니다.

## 코드 변경 원칙

- 애플리케이션 코드는 `App/Inc`, `App/Src`에 둡니다.
- 새 소스는 CMake target에 등록합니다.
- CubeMX USER CODE 영역과 생성 구조를 보존합니다.
- C11, 고정 폭 정수, 반환값 확인, RTOS 이후 동적 할당 최소화를 지킵니다.
- 하드웨어 실측값을 추정하지 말고 TODO 또는 앱 보정값으로 남깁니다.
- 앱·펌웨어의 16바이트 배열, 안전 인터록과 Flash 버전을 함께 확인합니다.
- 변경 범위에 맞는 가장 작은 분석·테스트·빌드만 실행합니다.

## 남은 실기 튜닝

- Wrist Tilt 포함 관절별 안전 끝점
- 실제 차체의 IMU 필터와 PID 계수

전원, MPU6050 장착 방향과 L298N 전진 극성은 실물에 적용되어 있습니다.
구체적인 전원 모델·출력값은 저장소 문서에 기록하지 않았습니다.
