# PyBullet 시뮬레이터

앱·펌웨어와 같은 관절 순서, 방향과 Mode 1 패킷을 PC에서 확인하는 독립 검증
도구입니다.

## 설치와 실행

```powershell
cd simulator
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python main.py
```

```powershell
python main.py --pose travel
python main.py --emit-packets --speed 50
python main.py --direct --duration 0.2 --no-realtime-sleep
```

Windows는 `pybullet-arm64`, 다른 운영체제는 `pybullet`을 사용하도록
`requirements.txt`에 조건이 설정되어 있습니다.

## 기준

| 대상 | 값 |
|---|---|
| 관절 | Base, Shoulder, Elbow, Wrist Tilt, Wrist Rotate, Gripper |
| 앱 범위 | 5관절 `-90~+90°`, Gripper `0~100%` |
| URDF 부호 | `[-1,-1,+1,-1,-1]` |
| 원점 | `[0,0,0,0,0]`, Gripper `0%` |
| 이동 자세 | `[0,-50,90,0,60]`, Gripper `0%`로 시작 |

실제 펌웨어의 이동 자세는 티칭 시퀀스 9이며 현재 Gripper를 유지합니다.
시뮬레이터의 범위와 이동 자세는 실물 안전 범위가 아닙니다.

`protocol.py`는 다음 고정 16바이트 Mode 1 프레임을 생성합니다.

```text
[AA,01,Base,Shoulder,Elbow,WristTilt,WristRotate,Gripper,
 Control,TransactionId,Speed,00,00,00,Checksum,55]
```

## 검증

```powershell
python -m unittest discover -s tests -v
python tools\validate_model.py
python main.py --direct --duration 0.1 --pose travel --emit-packets --no-realtime-sleep
```

모델 검사는 URDF, 메시, 질량, 관성텐서, 관절 설정과 DIRECT 로딩을 확인합니다.

## 활용과 확장

- 질량은 실측값, 링크 길이는 캡처 치수를 5 mm 단위로 정리한 값입니다.
- 무게중심·관성·충돌 형상은 관절과 패킷 검증용 근사값입니다.
- 기본 중력은 0이며 `drive_pid.py`로 펌웨어 PID 계산을 재현합니다.
- 실측 동역학, 차량 모델과 역기구학을 추가해 검증 범위를 넓힐 수 있습니다.

모델 세부값은 [models/README.md](models/README.md)를 확인합니다.
