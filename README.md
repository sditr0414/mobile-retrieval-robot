# Mobile Retrieval Robot

> **STM32F411 기반 5자유도 로봇팔 + 4WD 물품 회수·운반 이동 로봇**

스마트폰 앱에서 Bluetooth로 로봇팔과 이동 플랫폼을 제어하는 팀 프로젝트입니다.

## 0. 협업 도구

| 도구 | 용도 | 링크 |
|---|---|---|
| Slack | 실시간 질문, 진행 상황 공유 | [프로젝트 채널](https://app.slack.com/client/T0BDSNGQP2Q/C0BEHBXNNPJ) |
| GitHub | 코드, 문서, Issue, Pull Request | [저장소](https://github.com/sditr0414/mobile-retrieval-robot) |
| GitHub 권한 | 팀원 초대 및 접근 권한 관리 | [Collaborators 설정](https://github.com/sditr0414/mobile-retrieval-robot/settings/access) |
| Notion | 설계, 부품, 역할, 일정, 시험 기록 | [프로젝트 문서](https://app.notion.com/p/265dbc6c27e283deb63801b9fa29193f) |

## 1. 프로젝트 목표

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

## 2. 시스템 구성

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

## 3. 담당 영역

| 영역 | 담당 업무 | 주 작업 위치 |
|---|---|---|
| 앱·통신 | App Inventor, Bluetooth 명령 | `app/`, `protocol/` |
| 로봇팔 하드웨어 | 3D 출력, 조립, 영점·제한각 | `mechanical/`, `hardware/` |
| 로봇팔 제어 | PCA9685, IK, 티칭, Flash | `firmware/` |
| 차량 하드웨어 | 차체, 모터, 전원 배선 | `hardware/`, `mechanical/` |
| 차량 제어 | 조이스틱, 엔코더, PID, IMU | `firmware/` |

## 4. 저장소 구조

```text
app/          스마트폰 앱과 AIA 프로젝트
firmware/     STM32CubeIDE 펌웨어
hardware/     배선도, 핀맵, 전원 설계
mechanical/   3D 프린팅 파일, 조립 자료, 치수
protocol/     Bluetooth 명령 규격
docs/         협업, 시스템 설계, 시험 기록
```

## 5. 처음 시작하는 팀원

1. 저장소 접근 권한을 요청합니다.
2. VS Code에서 저장소를 Clone합니다.
3. `develop`을 최신 상태로 받습니다.
4. 작업별 `feature/...` 브랜치를 만듭니다.
5. 작은 단위로 Commit하고 GitHub에 Push합니다.
6. `develop`을 대상으로 Pull Request를 만듭니다.
7. 다른 팀원이 검토한 뒤 병합합니다.

```bash
git clone https://github.com/sditr0414/mobile-retrieval-robot.git
cd mobile-retrieval-robot
git switch develop
git pull
git switch -c feature/작업이름
```

자세한 절차는 [협업 가이드](CONTRIBUTING.md)와 [협업 도구 안내](docs/collaboration-tools.md)를 확인하세요.

## 6. 브랜치 규칙

- `main`: 통합 시험을 통과한 안정 버전
- `develop`: 기능을 모으는 개발 기준 브랜치
- `feature/<기능>`: 새 기능 개발
- `fix/<문제>`: 오류 수정
- `docs/<문서>`: 문서 변경
- `test/<시험>`: 시험 코드와 통합 검증

`main`과 `develop`에는 직접 기능 코드를 Push하지 않고 Pull Request로 병합합니다.

## 7. 핵심 문서

- [협업 가이드](CONTRIBUTING.md)
- [협업 도구 안내](docs/collaboration-tools.md)
- [시스템 아키텍처](docs/system-architecture.md)
- [핀맵](hardware/pin-map.md)
- [배선 및 전원 설계](hardware/wiring.md)
- [Bluetooth 명령 프로토콜](protocol/bluetooth-protocol.md)

## 8. 안전 원칙

- 서보와 모터 전원을 STM32 보드에서 직접 공급하지 않습니다.
- 모든 GND는 공통으로 연결하되 고전류 경로는 스타 접지로 분리합니다.
- 차량 제어에는 300~500 ms 통신 타임아웃과 비상정지를 구현합니다.
- 로봇팔을 펼친 상태에서는 차량 최고 속도를 제한합니다.
- 배선 변경 후 전원을 넣기 전에 다른 팀원이 한 번 더 검토합니다.
