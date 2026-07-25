# Bluetooth 명령 프로토콜

전송 방식은 Bluetooth Classic SPP이며, 모든 명령은 ASCII 문자열과 줄바꿈 `\n`으로 종료합니다.

## 공통 규칙

```text
CATEGORY,COMMAND,VALUE...\n
```

- 정수 값 사용
- 잘못된 명령은 무시
- 차량 명령 수신이 500 ms 이상 끊기면 정지
- `ESTOP`은 다른 명령보다 우선 처리

## 로봇팔

```text
ARM,J,1,90
ARM,J,2,120
ARM,XYZ,5,-3,0
ARM,HOME
```

| 명령 | 설명 |
|---|---|
| `ARM,J,<joint>,<angle>` | 단일 관절 목표각 |
| `ARM,XYZ,<dx>,<dy>,<dz>` | 말단 목표좌표 증분 |
| `ARM,HOME` | 홈 자세 |

## 티칭 및 재생

```text
TEACH,START
TEACH,ADD,1000
TEACH,STOP
TEACH,SAVE,1
SEQ,PLAY,1
SEQ,PAUSE
SEQ,STOP
SEQ,CLEAR,1
```

## 차량

```text
DRIVE,120,120
DRIVE,-120,-120
DRIVE,-100,100
DRIVE,0,0
```

형식:

```text
DRIVE,<left_speed>,<right_speed>
```

속도 범위는 초기 구현에서 `-255~255`로 통일합니다.

## 제어 및 상태

```text
ESTOP
STATUS?
PING
```

STM32 응답 예시:

```text
OK,ARM,J,1,90
OK,DRIVE,120,120
ERR,RANGE
STATUS,BAT=7.6,YAW=12.4,MODE=MANUAL
PONG
```

프로토콜이 변경되면 이 문서를 먼저 수정하고 앱과 펌웨어 PR을 연결합니다.
