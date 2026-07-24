# Mobile Retrieval Robot

STM32F411 기반 **5자유도 로봇팔 + 4WD 이동 플랫폼**을 스마트폰 앱으로 제어하는 물품 회수·운반 로봇 프로젝트입니다.

## 목표 기능

### 로봇팔
- 6채널 서보 수동 제어: Base, Shoulder, Arm, Wrist UD, Wrist Rotation, Gripper
- 동작 저장 및 재생
- 실행 중 티칭
- STM32 내부 Flash 또는 외부 메모리를 이용한 영구 저장
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
                    ├─ PWM/GPIO ── Motor Driver ── DC Motor × 4
                    ├─ I2C ── IMU
                    └─ Encoder Input ── Wheel Encoders
```

## 저장소 구조

```text
app/          스마트폰 앱 및 AIA 프로젝트
firmware/     STM32CubeIDE 펌웨어
mechanical/   3D 프린팅 파일, 조립 자료, 치수
hardware/     배선도, 핀맵, 전원 설계
protocol/     Bluetooth 명령 규격
 docs/        시스템 설계 및 시험 문서
```

## 브랜치 전략

- `main`: 통합 시험을 통과한 안정 버전
- `develop`: 다음 통합 버전
- `feature/<기능>`: 개인 기능 개발
- `fix/<문제>`: 오류 수정
- `docs/<문서>`: 문서 변경

직접 `main`에 기능 코드를 올리지 않고 Pull Request를 통해 검토합니다.

## 빠른 시작

1. 저장소를 Clone합니다.
2. `develop`에서 자신의 기능 브랜치를 생성합니다.
3. 담당 폴더에서 작업합니다.
4. 테스트 결과와 변경 내용을 Pull Request에 작성합니다.
5. 최소 1명의 검토 후 병합합니다.

자세한 협업 방법은 [CONTRIBUTING.md](CONTRIBUTING.md)를 참고하세요.

## 안전 원칙

- 서보와 모터 전원을 STM32 보드에서 직접 공급하지 않습니다.
- 모든 전원 GND는 공통으로 연결하되, 고전류 경로는 스타 접지로 분리합니다.
- 차량 제어에는 300~500 ms 통신 타임아웃과 비상정지를 구현합니다.
- 로봇팔을 펼친 상태에서는 차량 최고 속도를 제한합니다.

## 문서

- [시스템 아키텍처](docs/system-architecture.md)
- [배선 및 전원 설계](hardware/wiring.md)
- [Bluetooth 명령 프로토콜](protocol/bluetooth-protocol.md)
- [협업 규칙](CONTRIBUTING.md)
