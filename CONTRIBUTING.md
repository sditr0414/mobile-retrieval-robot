# 협업 가이드

GitHub을 처음 사용하는 팀원을 위한 최소 작업 절차입니다. 작업은 **개인 브랜치 → Commit → Push → Pull Request → 검토 → develop 병합** 순서로 진행합니다.

## 0. 협업 도구

- [Slack 프로젝트 채널](https://app.slack.com/client/T0BDSNGQP2Q/C0BEHBXNNPJ)
- [GitHub 저장소](https://github.com/sditr0414/mobile-retrieval-robot)
- [GitHub 접근 권한 설정](https://github.com/sditr0414/mobile-retrieval-robot/settings/access)
- [Notion 프로젝트 문서](https://app.notion.com/p/265dbc6c27e283deb63801b9fa29193f)

## 1. 최초 1회 설정

1. GitHub 계정을 만듭니다.
2. 저장소 관리자가 `Settings → Collaborators`에서 팀원을 초대합니다.
3. Git과 VS Code를 설치합니다.
4. VS Code에서 GitHub에 로그인합니다.
5. Git 사용자 정보를 설정합니다.

```bash
git config --global user.name "본인 이름"
git config --global user.email "GitHub 등록 이메일"
```

## 2. 저장소 Clone

VS Code에서 `Ctrl+Shift+P`를 누르고 `Git: Clone`을 선택한 뒤 아래 주소를 입력합니다.

```text
https://github.com/sditr0414/mobile-retrieval-robot.git
```

터미널에서는 다음 명령을 사용할 수 있습니다.

```bash
git clone https://github.com/sditr0414/mobile-retrieval-robot.git
cd mobile-retrieval-robot
code .
```

## 3. 작업 시작

항상 최신 `develop`에서 새 브랜치를 만듭니다.

```bash
git switch develop
git pull
git switch -c feature/기능이름
```

브랜치 예시:

```text
feature/app-bluetooth
feature/arm-servo-control
feature/teaching-memory
feature/vehicle-joystick
feature/vehicle-pid-imu
docs/power-wiring
fix/servo-angle-limit
```

## 4. 변경 확인과 Commit

VS Code 왼쪽의 Source Control 화면에서 변경 파일을 확인합니다.

1. Commit에 포함할 파일 옆 `+`를 눌러 Stage합니다.
2. Commit 메시지를 입력합니다.
3. Commit 버튼을 누릅니다.

권장 접두어:

| 접두어 | 용도 |
|---|---|
| `feat:` | 새 기능 |
| `fix:` | 오류 수정 |
| `docs:` | 문서 수정 |
| `test:` | 시험 코드·결과 |
| `refactor:` | 구조 개선 |
| `chore:` | 설정·기타 작업 |

예시:

```text
feat: add PCA9685 servo control
fix: stop motors after bluetooth timeout
docs: update STM32 pin map
```

## 5. GitHub에 Push

새 브랜치에서 처음 Push할 때:

```bash
git push -u origin feature/기능이름
```

그다음부터:

```bash
git push
```

VS Code에서는 `Publish Branch`, `Push`, 또는 `Sync Changes` 버튼을 사용할 수 있습니다.

## 6. Pull Request

GitHub에서 다음 기준으로 Pull Request를 만듭니다.

```text
base: develop
compare: feature/기능이름
```

PR 본문에는 다음을 작성합니다.

- 작업 목적
- 변경 파일
- 테스트 방법과 결과
- 하드웨어 연결 조건
- 알려진 문제
- 관련 Issue 번호

최소 한 명이 검토한 뒤 `develop`에 병합합니다.

## 7. 작업 종료 후 다음 작업 준비

```bash
git switch develop
git pull
```

병합된 기존 개인 브랜치는 GitHub와 로컬에서 삭제해도 됩니다.

```bash
git branch -d feature/기능이름
```

## 8. 담당 영역

| 담당 | 주 작업 폴더 | 주요 책임 |
|---|---|---|
| 앱·통신 | `app/`, `protocol/` | App Inventor, Bluetooth 패킷 |
| 로봇팔 하드웨어 | `mechanical/`, `hardware/` | 출력물, 서보 조립, 영점 |
| 로봇팔 제어 | `firmware/` | 관절 제어, IK, 티칭, Flash |
| 차량 하드웨어 | `hardware/`, `mechanical/` | 모터, 전원, 차체 배선 |
| 차량 제어 | `firmware/` | 조이스틱 혼합, 엔코더, PID, IMU |

공용 인터페이스를 변경할 때는 `protocol/` 또는 `hardware/` 문서를 먼저 수정하고 팀원 검토를 받습니다.

## 9. 코드 검토 기준

- 핀 번호가 `hardware/pin-map.md`와 일치하는가
- Bluetooth 명령이 `protocol/bluetooth-protocol.md`와 일치하는가
- 모터 타임아웃과 비상정지가 유지되는가
- 서보 각도 제한과 영점 보정이 적용되는가
- Flash에 지나치게 자주 기록하지 않는가
- 전원 차단 후 안전 상태로 복귀하는가
- 자동 생성 빌드 파일이 포함되지 않았는가

## 10. 충돌 방지 규칙

- `main`과 `develop`에서 직접 기능 작업을 하지 않습니다.
- 작업 전 `develop`을 최신 상태로 갱신합니다.
- 한 파일을 여러 사람이 동시에 크게 수정하지 않습니다.
- 큰 기능은 Issue를 먼저 만들고 담당자를 지정합니다.
- 핀맵과 통신 명령 변경은 별도 PR로 분리합니다.
- 충돌이 발생하면 해당 파일 담당자와 함께 해결합니다.

## 11. 초보자 연습

실제 펌웨어 작업 전에 각 팀원이 `docs/team/이름.md` 파일을 추가해 다음 과정을 한 번 연습합니다.

```text
Clone → develop 갱신 → 개인 브랜치 → 파일 수정 → Commit → Push → Pull Request → 검토 → 병합
```
