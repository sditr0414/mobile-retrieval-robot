# 협업 가이드

```text
개인 브랜치 → Commit → Push → Pull Request → 검토 → develop
```

## 시작

```powershell
git clone https://github.com/sditr0414/mobile-retrieval-robot.git
cd mobile-retrieval-robot
git switch develop
git pull
git switch -c feature/기능이름
```

브랜치 예: `feature/teaching`, `fix/bluetooth-timeout`, `docs/power-wiring`.

## Commit

| 접두어 | 용도 |
|---|---|
| `feat` | 기능 |
| `fix` | 오류 |
| `docs` | 문서 |
| `test` | 시험 |
| `refactor` | 구조 개선 |
| `chore` | 설정 |

## Pull Request

- 대상은 `develop`.
- 목적, 변경, 시험 결과와 남은 위험을 기록.
- 핀·전원 변경은 실제 연결 조건을 기록.
- 실행하지 못한 시험은 이유를 기록.
- 최소 한 명 검토 후 병합.

## 확인

- 핀맵과 `.ioc` 일치.
- 앱·펌웨어의 16바이트 배열 일치.
- E-STOP, 500 ms 타임아웃과 인터록 유지.
- 서보 방향·보정 범위 유지.
- Flash 버전·CRC·이전 데이터 변환 유지.
- ACK 전에 앱이 성공을 표시하지 않음.
- 생성된 빌드 결과물을 포함하지 않음.

```powershell
cd app
flutter analyze
flutter test
```

```powershell
cd firmware/mobile_retrieval_robot
cmake --preset Debug
cmake --build --preset Debug
```

변경한 영역에 필요한 검사만 실행합니다.
