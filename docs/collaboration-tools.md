# 협업 도구 안내

프로젝트에서 사용하는 도구와 기록 위치를 한곳에 정리합니다.

## 바로가기

| 도구 | 링크 | 기록할 내용 |
|---|---|---|
| Slack | [프로젝트 채널](https://app.slack.com/client/T0BDSNGQP2Q/C0BEHBXNNPJ) | 질문, 당일 진행 상황, 빠른 의사소통 |
| GitHub | [코드 저장소](https://github.com/sditr0414/mobile-retrieval-robot) | 코드, 문서, Issue, Pull Request |
| GitHub 권한 | [Collaborators 설정](https://github.com/sditr0414/mobile-retrieval-robot/settings/access) | 팀원 초대와 접근 권한 |
| Notion | [프로젝트 문서](https://app.notion.com/p/265dbc6c27e283deb63801b9fa29193f) | 확정 설계, 부품, 담당, 일정, 시험 결과 |

## 어떤 내용을 어디에 기록할까

### Slack

- 바로 답이 필요한 질문
- 오늘 진행한 작업
- 회의 시간과 간단한 일정 조율
- 장비 사용 여부와 현장 사진 공유

중요한 결정은 Slack에만 남기지 않고 Notion 또는 GitHub 문서에도 반영합니다.

### GitHub Issue

- 구현할 기능
- 발견한 오류
- 시험이 필요한 항목
- 담당자와 완료 조건

Issue 제목 예시:

```text
[ARM] PCA9685 서보 6채널 제어
[VEHICLE] 좌우 모터 속도 PID
[APP] Bluetooth 재연결 처리
[HARDWARE] 서보 전원 노이즈 확인
```

### GitHub Pull Request

- 변경한 코드와 문서
- 변경 이유
- 시험 방법과 결과
- 알려진 문제
- 관련 Issue

### Notion

- 최종 부품 목록
- 확정 핀맵과 배선도
- 역할 분담과 일정
- PID 튜닝 결과
- 서보 영점과 제한각
- 시연 절차와 최종 시험 결과

## 권장 일일 흐름

1. 작업 시작 전에 GitHub Issue와 Slack을 확인합니다.
2. `develop`을 Pull하고 개인 브랜치를 만듭니다.
3. 작업 중 질문은 Slack에 공유합니다.
4. 변경 사항을 Commit하고 Push합니다.
5. Pull Request를 만들고 팀원에게 검토를 요청합니다.
6. 확정된 설계나 측정값은 Notion에 반영합니다.

## 파일과 문서의 기준

- 코드의 최신 원본: GitHub
- 설계와 일정의 최신 원본: Notion
- 빠른 대화와 질문: Slack
- 해야 할 일과 오류 추적: GitHub Issue

같은 정보를 여러 곳에 복사하기보다는 원본 위치의 링크를 공유합니다.
