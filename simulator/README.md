# PyBullet 시뮬레이터

모바일 물품 회수 로봇을 PyBullet에서 시험하기 위한 시뮬레이터입니다.

현재는 PyBullet GUI에 바닥 평면과 초기 `robot_arm.urdf`를 표시합니다.
Base, Shoulder, Elbow, Wrist Tilt, Wrist Rotate 관절은 화면의 슬라이더로
`-90~90도` 범위에서 움직일 수 있습니다.

URDF는 캡처 이미지의 치수와 STL 형상을 사용한 초기 시각화 모델입니다. 출력
부품과 서보 질량은 측정되어 `models/mass_properties.json`에 기록했습니다.
링크별 무게중심, 관성, 충돌 형상, 실제 관절 제한과 그리퍼 링크의 회전 중심은
확인되지 않았으므로 TODO로 남겨 두었습니다.

Windows에서는 공식 `pybullet` 패키지의 사전 빌드 파일이 없어 C++ 컴파일이
필요합니다. 이를 피하기 위해 Windows용 wheel을 제공하는 호환 패키지
`pybullet-arm64`를 사용합니다. 패키지 이름과 관계없이 코드는 동일하게
`import pybullet`을 사용합니다. 다른 운영체제에서는 공식 `pybullet`을 설치합니다.

## 실행

PowerShell에서 다음 명령을 실행합니다.

```powershell
cd simulator
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python main.py
```

바닥과 로봇팔이 표시되면 PyBullet 실행 환경이 정상입니다. 오른쪽 슬라이더로
5개 관절을 움직일 수 있으며, 창을 닫으면 프로그램이 종료됩니다.

## 폴더

```text
simulator/
├─ models/           로봇 URDF와 메시 파일
├─ main.py           PyBullet 실행 진입점
├─ requirements.txt  Python 의존성
└─ README.md         실행 방법과 TODO
```

## TODO

- 차체 크기와 질량 측정
- 바퀴 반지름, 폭과 축 위치 측정
- 각 서보의 실제 고정 링크와 링크별 무게중심 측정
- 실측 질량과 STL 형상으로 링크별 관성텐서 계산
- 각 관절의 회전축, 방향과 제한각 확정
- 실측값을 반영한 URDF 작성
- 펌웨어와 동일한 6채널 관절 순서 적용
- 앱 또는 테스트 스크립트에서 11바이트 명령 연동
