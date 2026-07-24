# 협업 가이드

## 기본 작업 흐름

```bash
git clone https://github.com/sditr0414/mobile-retrieval-robot.git
cd mobile-retrieval-robot
git checkout develop
git pull
git checkout -b feature/기능이름
```

작업 후:

```bash
git add .
git commit -m "feat: 변경 내용"
git push -u origin feature/기능이름
```

GitHub에서 `feature/...` → `develop` Pull Request를 생성합니다.

## 커밋 메시지

- `feat:` 기능 추가
- `fix:` 오류 수정
- `docs:` 문서 수정
- `refactor:` 구조 개선
- `test:` 시험 코드 및 결과
- `chore:` 설정과 기타 작업

예시:

```text
feat: add PCA9685 servo angle control
fix: stop motors after bluetooth timeout
docs: add NUCLEO-F411RE pin map
```

## 담당 영역

| 담당 | 주 작업 폴더 | 주요 책임 |
|---|---|---|
| 앱·통신 | `app/`, `protocol/` | App Inventor, Bluetooth 패킷 |
| 로봇팔 하드웨어 | `mechanical/`, `hardware/` | 출력물, 서보 조립, 영점 |
| 로봇팔 제어 | `firmware/` | 관절 제어, IK, 티칭, Flash |
| 차량 하드웨어 | `hardware/`, `mechanical/` | 모터, 전원, 차체 배선 |
| 차량 제어 | `firmware/` | 조이스틱 혼합, 엔코더, PID, IMU |

공용 인터페이스 변경은 `protocol/` 또는 `hardware/` 문서를 먼저 수정하고 팀원 검토를 받습니다.

## Pull Request 규칙

PR 본문에 다음을 기록합니다.

- 변경 목적
- 변경 파일
- 시험 방법과 결과
- 하드웨어 연결 조건
- 알려진 문제
- 관련 Issue

펌웨어 PR은 빌드 성공 여부를, 하드웨어 PR은 배선 사진 또는 측정값을 포함하는 것을 권장합니다.

## 코드 검토 기준

- 핀 번호가 `hardware/pin-map.md`와 일치하는가
- Bluetooth 명령이 `protocol/bluetooth-protocol.md`와 일치하는가
- 모터 타임아웃과 비상정지가 유지되는가
- 서보 각도 제한과 영점 보정이 적용되는가
- Flash에 과도하게 반복 기록하지 않는가
- 전원 차단 후 안전 상태로 복귀하는가

## 충돌 방지

- 한 파일을 여러 명이 동시에 크게 수정하지 않습니다.
- 작업 전 `develop`을 최신 상태로 갱신합니다.
- 큰 기능은 Issue를 먼저 만들고 담당자를 표시합니다.
- 공통 헤더와 핀맵 변경은 별도 PR로 분리합니다.
