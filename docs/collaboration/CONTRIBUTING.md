# 협업 가이드

GitHub 작업은 **개인 브랜치 → Commit → Push → Pull Request → 검토 → develop 병합** 순서로 진행합니다.

## 최초 설정

```bash
git clone https://github.com/sditr0414/mobile-retrieval-robot.git
cd mobile-retrieval-robot
git switch develop
git pull
```

## 기능 브랜치 만들기

```bash
git switch -c feature/기능이름
```

예시:

```text
feature/arm-servo-control
feature/teaching-memory
feature/vehicle-pid
fix/bluetooth-timeout
docs/power-wiring
```

## Commit 규칙

| 접두어 | 용도 |
|---|---|
| `feat:` | 새 기능 |
| `fix:` | 오류 수정 |
| `docs:` | 문서 수정 |
| `test:` | 시험 코드·결과 |
| `refactor:` | 구조 개선 |
| `chore:` | 설정 작업 |

## Pull Request

- 대상 브랜치: `develop`
- 작업 목적과 주요 변경 내용을 작성합니다.
- 하드웨어 연결 조건과 시험 결과를 기록합니다.
- 최소 한 명의 검토 후 병합합니다.

## 코드 검토 기준

- `docs/hardware/pin-map.md`와 핀이 일치하는가
- `docs/protocol/bluetooth-protocol.md`와 명령 형식이 일치하는가
- 모터 타임아웃과 비상정지가 유지되는가
- 서보 제한각과 영점 보정이 적용되는가
- Flash에 지나치게 자주 기록하지 않는가
- 자동 생성 빌드 파일이 포함되지 않았는가

## 담당 영역

| 영역 | 주 작업 위치 |
|---|---|
| 스마트폰 제어 앱 | `app/` |
| PyBullet 시뮬레이터·로봇 모델 | `simulator/` |
| STM32 및 로봇팔 제어 | `firmware/` |
| 차량 주행·PID·IMU | `firmware/` |
| 핀맵·배선·전원 | `docs/hardware/` |
| Bluetooth 명령 | `docs/protocol/` |
| 협업 절차 | `docs/collaboration/` |
