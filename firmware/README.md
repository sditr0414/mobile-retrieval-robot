# STM32F411 제어 펌웨어

STM32CubeIDE 기반 메인 제어 코드가 들어가는 폴더입니다.

## 담당 기능

- HC-05 UART 통신 및 명령 파싱
- PCA9685 기반 6채널 서보 제어
- 로봇팔 수동 제어, 티칭, 동작 저장·재생
- 내부 Flash 기반 영구 저장
- XYZ 조이스틱 입력과 역기구학
- 4WD 좌우 모터 PWM 및 방향 제어
- 엔코더 속도 PID와 IMU 방향 안정화
- Bluetooth 타임아웃 및 비상정지

## 권장 구조

```text
firmware/
├─ Core/
│  ├─ Inc/
│  └─ Src/
├─ Drivers/
├─ Middlewares/
├─ mobile_retrieval_robot.ioc
├─ .gitignore
└─ README.md
```

## 시작 전 확인

1. `docs/hardware/pin-map.md`의 핀 배치를 확인합니다.
2. `docs/protocol/bluetooth-protocol.md`의 명령 형식을 확인합니다.
3. 기능별 브랜치를 만들어 작업합니다.
4. 실제 하드웨어 시험 결과를 Pull Request에 기록합니다.
