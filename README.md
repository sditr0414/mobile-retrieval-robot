# Mobile Retrieval Robot

> **STM32F411 기반 5자유도 로봇팔 + 4WD 물품 회수·운반 이동 로봇**

스마트폰 앱에서 Bluetooth로 로봇팔과 이동 플랫폼을 제어하는 팀 프로젝트입니다.

## 최상위 폴더

```text
mobile-retrieval-robot/
├─ firmware/   STM32F411 제어 코드
├─ docs/       부품·배선·통신·협업 지침
└─ README.md   프로젝트 전체 안내
```

## 1차 펌웨어 범위

현재 1차 펌웨어에서는 아래 기능만 구현합니다.

- HC-05 Bluetooth 명령 수신
- PCA9685 기반 로봇팔 서보 6채널 제어
- L298N 기반 4WD 좌우 모터 제어
- Bluetooth 연결 종료 또는 명령 타임아웃 시 차량 정지

초음파 센서와 IMU는 후속 단계에서 추가합니다. 휠 엔코더는 사용하지 않습니다.

## 주요 기능

### 로봇팔

- 6채널 서보 수동 제어
- 저장 동작 실행
- 실행 중 티칭
- 전원이 꺼져도 유지되는 티칭 데이터 저장
- 조이스틱 기반 XYZ 말단 위치 제어 및 역기구학
- 후속 단계에서 전방 초음파 센서 기반 전개 안전 인터록 추가

### 4WD 차량

- 조이스틱 기반 전진·후진·회전·속도 제어
- Bluetooth 연결 종료 또는 명령 타임아웃 시 자동 정지
- 후속 단계에서 IMU yaw 기반 방향 안정화 추가
- 휠 엔코더와 엔코더 기반 거리·위치 제어는 사용하지 않음

## 시스템 구성

### 1차 구성

```text
Smartphone App
    │ Bluetooth Classic SPP
    ▼
HC-05 ── UART ── STM32F411
                    ├─ I2C ── PCA9685 ── Servo × 6
                    └─ PWM/GPIO ── L298N ── DC Motor × 4
```

### 후속 구성

```text
STM32F411
├─ IMU → 방향 안정화
└─ Ultrasonic Sensor → 전방 20 cm 미만 로봇팔 전개 금지
```

## 로봇팔 전개 안전 조건

초음파 센서를 추가한 뒤 로봇팔 전개 명령은 아래 조건을 모두 만족할 때만 실행합니다.

- 차량 정지 상태
- Bluetooth 비상정지 해제 상태
- 초음파 측정값이 유효함
- 전방 거리 20 cm 이상
- 연속 측정값이 일정 횟수 이상 안전 판정

설정값:

```text
전개 허용 거리: 20 cm 이상
전개 금지 거리: 20 cm 미만
측정 주기: 50~100 ms
안전 판정: 연속 3회 이상
센서 오류·시간 초과: 전개 금지
```

## 문서 바로가기

- [프로젝트 부품 목록(BOM)](docs/hardware/bom.md)
- [STM32 펌웨어 안내](firmware/README.md)
- [협업 가이드](docs/collaboration/CONTRIBUTING.md)
- [협업 도구 안내](docs/collaboration/collaboration-tools.md)
- [Pull Request 작성 양식](docs/collaboration/pull-request-template.md)
- [STM32 권장 핀맵](docs/hardware/pin-map.md)
- [Bluetooth 명령 프로토콜](docs/protocol/bluetooth-protocol.md)

## 작업 시작

```bash
git clone https://github.com/sditr0414/mobile-retrieval-robot.git
cd mobile-retrieval-robot
git switch develop
git pull
git switch -c feature/작업이름
```

기능 작업은 `firmware/`, 설계와 협업 지침은 `docs/`에서 관리합니다.

## 안전 원칙

- 서보와 모터 전원을 STM32 보드에서 직접 공급하지 않습니다.
- 모든 GND는 공통으로 연결하되 고전류 경로는 스타 접지로 분리합니다.
- 차량 제어에는 300~500 ms 통신 타임아웃과 비상정지를 구현합니다.
- 로봇팔을 펼친 상태에서는 차량 최고 속도를 제한합니다.
- 배선 변경 후 전원을 넣기 전에 다른 팀원이 검토합니다.
