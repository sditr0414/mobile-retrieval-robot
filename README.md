# Mobile Retrieval Robot

STM32F411 기반 5자유도 로봇팔과 4WD 이동 플랫폼입니다. Flutter 앱으로
HC-05를 통해 차량, 로봇팔, 티칭과 상대 Yaw PID를 제어합니다.

## 구성

```text
Smartphone ── Bluetooth ── HC-05 ── UART ── STM32F411
                                               ├─ I2C ── PCA9685 ── Servo × 6
                                               ├─ I2C ── MPU6050
                                               └─ PWM/GPIO ── L298N ── DC Motor × 4
```

| 폴더 | 내용 |
|---|---|
| `app/` | Flutter Android 제어 앱 |
| `firmware/` | STM32CubeMX CMake 펌웨어 |
| `simulator/` | PyBullet 로봇팔 시뮬레이터 |
| `docs/` | 하드웨어, 프로토콜, 협업 문서 |

## 구현 상태

- 고정 16바이트 Bluetooth 명령·ACK
- PCA9685 6채널 서보와 S-curve 이동
- L298N 4WD 제어, E-STOP, 500 ms 통신 타임아웃
- 이름을 포함한 12개 티칭 시퀀스와 PID·서보 보정값의 Sector 7 저장
- MPU6050 상대 Yaw와 직진·후진 방향 안정화 PID
- Flutter 차량·로봇팔·티칭 UI
- PyBullet 관절·패킷·PID 검증

제외 범위: 초음파 센서, 휠 엔코더, 절대 Yaw, 역기구학과 XYZ 제어.

## 시작

```powershell
git clone https://github.com/sditr0414/mobile-retrieval-robot.git
cd mobile-retrieval-robot
git switch develop
```

- 앱: [app/README.md](app/README.md)
- 펌웨어: [firmware/README.md](firmware/README.md)
- 시뮬레이터: [simulator/README.md](simulator/README.md)
- 부품과 전원: [docs/hardware/bom.md](docs/hardware/bom.md)
- 핀맵: [docs/hardware/pin-map.md](docs/hardware/pin-map.md)
- 서보 보정: [docs/hardware/servo-calibration.md](docs/hardware/servo-calibration.md)
- IMU: [docs/hardware/imu.md](docs/hardware/imu.md)
- Bluetooth 규격: [docs/protocol/bluetooth-protocol.md](docs/protocol/bluetooth-protocol.md)
- 협업: [docs/collaboration/CONTRIBUTING.md](docs/collaboration/CONTRIBUTING.md)

## 안전

- 서보와 모터 전원은 STM32에서 공급하지 않습니다.
- 서보용·차량용 2S 팩의 양극은 분리하고 GND만 공통으로 연결합니다.
- 고전류 귀환은 STM32 보드를 경유하지 않습니다.
- 현재 BMS, 퓨즈와 물리 비상정지는 없습니다. 앱 E-STOP은 전원을 물리적으로
  차단하지 않습니다.
- 전원을 넣기 전에 배선, 관절 간섭과 작업 공간을 확인합니다.
