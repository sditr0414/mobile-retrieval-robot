# Bluetooth 11바이트 바이너리 프로토콜

이 문서는 Flutter 앱과 STM32 펌웨어가 함께 사용하는 Bluetooth Classic SPP
규격입니다. 명령의 기준은 `app/lib/main.dart`입니다.

## 공통 패킷

- UART: 9600 baud, 8 data bits, no parity, 1 stop bit
- 고정 길이: 11바이트
- ASCII 문자열과 줄바꿈 명령은 사용하지 않음

```text
[Header, Mode, Data0, Data1, Data2, Data3, Data4, Data5, Data6,
 Checksum, Tail]
```

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | Header | `0xAA` |
| 1 | Mode | `0x00` Drive, `0x01` Arm, `0x02` Teaching |
| 2~8 | Data0~Data6 | Mode별 데이터 |
| 9 | Checksum | Byte 1~8 합의 하위 8비트 |
| 10 | Tail | `0x55` |

```c
checksum = (byte[1] + byte[2] + byte[3] + byte[4] +
            byte[5] + byte[6] + byte[7] + byte[8]) & 0xFF;
```

Header, Tail, Mode, Checksum 또는 Mode별 값이 하나라도 잘못되면 패킷 전체를
적용하지 않습니다.

## Mode 0: 차량

```text
[AA, 00, L_DIR, L_PWM, R_DIR, R_PWM, Control, 00, 00, checksum, 55]
```

| 필드 | 값 |
|---|---|
| `L_DIR`, `R_DIR` | `0` 전진, `1` 후진 |
| `L_PWM`, `R_PWM` | `0~255`; `0`이면 해당 모터 정지 |
| `Control` | `0` 일반, `1` E-STOP 잠금, `2` E-STOP 해제 |
| Data5~Data6 | 반드시 `0x00` |

앱 PWM은 TIM3의 `0~999`로 선형 변환합니다. 유효한 차량 명령이 500 ms 동안
들어오지 않거나 다른 Mode로 바뀌면 양쪽 모터를 정지합니다. 앱은 주행 화면에서
100 ms마다 현재 값을 다시 보내 타임아웃을 갱신합니다.

- E-STOP과 해제 패킷은 방향과 PWM을 모두 `0`으로 보내야 합니다.
- E-STOP을 받으면 펌웨어가 잠금 상태를 유지하고 일반 주행 명령을 무시합니다.
- E-STOP 한 패킷으로 차량을 정지하고 PCA9685 서보 출력을 함께 끕니다.
- 앱에서 사용자가 확인한 뒤 `Control=2`를 보내야 주행 명령을 다시 받습니다.

예: 좌우 PWM 120으로 전진

```text
AA 00 00 78 00 78 00 00 00 F0 55
```

## Mode 1: 로봇팔

```text
[AA, 01, Base, Shoulder, Elbow, WristTilt, WristRotate, Gripper,
 Control, checksum, 55]
```

| Data | 관절 | PCA9685 채널 |
|---:|---|---:|
| 0 | Base | 0 |
| 1 | Shoulder | 1 |
| 2 | Elbow | 2 |
| 3 | Wrist Tilt | 3 |
| 4 | Wrist Rotate | 4 |
| 5 | Gripper | 5 |
| 6 | Control | - |

- Base부터 Wrist Rotate까지 앱 표시 각도는 `-90~+90도`입니다.
- Data0~Data4에는 `앱 표시 각도 + 90`을 적용한 `0~180` 값을 넣습니다.
- Gripper는 앱에서 `0%` 최대 열림, `100%` 최대 닫힘으로 표시합니다.
- Data5에는 `Gripper 퍼센트 × 180 ÷ 100`으로 변환한 `0~180`을 넣습니다.
- Base는 조립 방향 때문에 펌웨어에서 출력 방향을 반전합니다.
- Gripper도 펌웨어에서 출력 방향을 반전해 `0%=1840 us`,
  `100%=1140 us`로 출력합니다.
- Data6은 `0` 이동, `1` 지정 자세로 출력 활성화, `2` 출력 차단입니다.
- 부팅 직후에는 서보 출력을 끄며, 활성화 명령 전에는 이동을 거부합니다.
- 앱의 홈 자세는 관절 `0도`, Gripper `0%`이며 패킷값은
  `[90,90,90,90,90,0]`입니다.
- 수동 이동, 홈 복귀와 티칭 재생은 최대 `1도/20 ms`, 약 `50도/s`로 보간합니다.
- 6개 관절에 사용할 보정 설정이 모두 등록된 경우에만 출력을 활성화합니다.
- Shoulder, Elbow와 Wrist Rotate는 실측 위치를 환산한 관절별 펄스폭을
  사용하며, Wrist Tilt만 수동 확인 전까지 임시 `500/1500/2500 us`를 사용합니다.
- 실제 위치를 읽는 값이 아니라 마지막으로 요청한 위치입니다.
- 한 값이라도 범위를 벗어나면 6개 관절 모두 적용하지 않습니다.
- 수동 명령을 받으면 진행 중인 티칭 재생을 중지합니다.
- 위치 피드백이 없으므로 전원 투입 후 첫 활성화에서는 실제 시작 위치를 알 수
  없으며, 이후 재활성화부터 마지막 명령 위치를 기준으로 홈까지 보간합니다.

예: 모든 관절을 원점 0도, Gripper를 홈 0%로 지정

```text
AA 01 5A 5A 5A 5A 5A 00 00 C3 55
```

### 로봇팔 상태 ACK

활성화·비활성화 명령과 실패한 이동 명령에는 다음 형식으로 응답합니다.

```text
[AA,01,Command,Status,Reason,00,00,00,00,checksum,55]
```

- `Command`: 요청한 Control 값
- `Status`: `1` 성공, `0` 실패
- `Reason`: `0` 성공, `1` 잘못된 값, `2` 미보정 관절, `3` 비활성 상태,
  `4` 다른 동작 실행 중, `5` PCA9685 통신 오류

## Mode 2: 티칭

앱은 12개 시퀀스를 사용하며, 각 시퀀스에는 최대 30개 웨이포인트를 저장할
수 있습니다. 웨이포인트 한 개는 6개 관절 각도입니다.

### 재생

```text
[AA, 02, 02, Sequence, 00, 00, 00, 00, 00, checksum, 55]
```

- `Sequence`: `1~12`
- 저장된 웨이포인트를 첫 번째부터 재생
- 빈 시퀀스는 실행하지 않음
- 부팅할 때 Flash 데이터는 읽지만 자동 재생하지 않음

### 초기화

```text
[AA, 02, 03, Sequence, 00, 00, 00, 00, 00, checksum, 55]
```

재생을 중지하고 선택한 시퀀스를 RAM과 Flash에서 비웁니다.

### 앱에서 Flash로 업로드

업로드는 START, 각 웨이포인트의 전반부와 후반부, COMMIT 순서입니다.

```text
START: [AA,02,04,01,Sequence,Count,00,00,00,checksum,55]
FIRST: [AA,02,04,02,Sequence,Index,A0,A1,A2,checksum,55]
SECOND:[AA,02,04,03,Sequence,Index,A3,A4,A5,checksum,55]
COMMIT:[AA,02,04,04,Sequence,00,00,00,00,checksum,55]
```

- `Count`: `1~30`
- `Index`: `0~Count-1`
- `A0~A4`: 앱 표시 각도 `-90~+90`에 90을 더한 `0~180`
- `A5`: Gripper `0~100%`를 선형 변환한 `0~180`
- START에서 임시 버퍼를 초기화합니다.
- 모든 Index의 FIRST와 SECOND가 도착한 경우에만 COMMIT을 허용합니다.
- Flash Sector 7을 기록한 뒤 전체 내용을 다시 비교합니다.

### 저장 ACK

```text
[AA,02,04,Sequence,Status,00,00,00,00,checksum,55]
```

- `Status=1`: 저장과 검증 성공
- `Status=0`: 저장 실패

앱은 ACK도 스트림에서 11바이트 단위로 복구하고 체크섬을 확인합니다.

## 스트림 처리

USART1 RX는 128바이트 circular DMA를 사용합니다.

1. DMA는 수신 바이트를 버퍼에 저장합니다.
2. `bluetoothTask`가 DMA write 위치까지 새 바이트를 읽습니다.
3. `0xAA`를 찾고 11바이트가 모일 때까지 기다립니다.
4. 패킷 전체를 검증한 후 `armQueue` 또는 `driveQueue`로 전달합니다.
5. 잘못된 패킷은 다음 `0xAA`부터 동기화를 다시 맞춥니다.
6. ISR에서는 패킷 파싱, 모터·서보 제어와 Flash 작업을 하지 않습니다.
7. UART 오류가 나면 콜백은 플래그만 남기고 태스크에서 circular DMA를 재시작합니다.

## 아직 확인할 하드웨어 항목

- L298N 좌우 모터의 실제 전진 극성
- Shoulder, Elbow, Wrist Tilt, Wrist Rotate의 최소·중앙·최대 펄스폭
- 조립 상태에서 허용할 관절별 안전 각도
