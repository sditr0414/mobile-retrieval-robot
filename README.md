# Mobile Retrieval Robot

> **STM32F411 기반 5자유도 로봇팔 + 4WD 물품 회수·운반 이동 로봇**

스마트폰 앱에서 Bluetooth로 로봇팔과 이동 플랫폼을 제어하는 팀 프로젝트입니다.

## 최상위 폴더

```text
mobile-retrieval-robot/
├─ firmware/   STM32F411 제어 코드
├─ docs/       배선·통신·협업 지침
└─ README.md   프로젝트 전체 안내
```

## 주요 기능

### 로봇팔

- 6채널 서보 수동 제어
- 저장 동작 실행
- 실행 중 티칭
- 전원이 꺼져도 유지되는 티칭 데이터 저장
- 조이스틱 기반 XYZ 말단 위치 제어 및 역기구학

### 4WD 차량

- 조이스틱 기반 전진·후진·회전·속도 제어
- 좌우 바퀴 속도 PID
- IMU yaw 기반 직진 및 방향 안정화
- Bluetooth 연결 종료 또는 명령 타임아웃 시 자동 정지

## 시스템 구성

```text
Smartphone App
    │ Bluetooth Classic SPP
    ▼
HC-05 ── UART ── STM32F411
                    ├─ I2C ── PCA9685 ── Servo × 6
                    ├─ PWM/GPIO ── L298N ── DC Motor × 4
                    ├─ I2C ── IMU
                    └─ Encoder Input ── Wheel Encoders
```

## 문서 바로가기

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
