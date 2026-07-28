# 로봇 모델

PyBullet에서 사용할 URDF와 메시 파일을 보관합니다.

로봇팔 부품별 STL 파일은 영문 `snake_case` 이름으로 정리했습니다. 부품과
서보의 질량은 측정되었으며 `mass_properties.json`에도 같은 값을 기록합니다.
링크별 무게중심과 관성텐서는 아직 확인되지 않았으므로 URDF에는 임의의 값을
넣지 않습니다.

## STL 파일

```text
bar_1.stl
bar_2.stl
base.stl
base_cover.stl
central_shaft.stl
forearm.stl
gear_1.stl
gear_2.stl
gripper_base.stl
gripper_cover.stl
gripper_finger_1.stl
gripper_finger_2.stl
servo_retainer.stl
shaft_cover.stl
upper_arm.stl
wrist.stl
```

## 기준 이미지

```text
left_side_view.png
right_side_view.png
top_view.png
bottom_view.png
gripper_front_view.png
```

## 실측 질량

| 부품 | 질량 |
|---|---:|
| `bar_1` | 2.51 g |
| `bar_2` | 2.53 g |
| `base` | 72.29 g |
| `base_cover` | 9.85 g |
| `central_shaft` | 52.59 g |
| `forearm` | 44.10 g |
| `gear_1` | 3.21 g |
| `gear_2` | 2.96 g |
| `gripper_base` | 12.44 g |
| `gripper_cover` | 3.10 g |
| `gripper_finger_1` | 6.13 g |
| `gripper_finger_2` | 6.12 g |
| `servo_retainer` | 0.60 g |
| `shaft_cover` | 6.57 g |
| `upper_arm` | 31.17 g |
| `wrist` | 13.05 g |

출력 부품 합계는 269.22 g입니다.

| 관절 | 서보 | 개당 질량 |
|---|---|---:|
| Base | MG996R | 55 g |
| Shoulder | MG996R | 55 g |
| Elbow | MG996R | 55 g |
| Wrist Tilt | SG90 | 9 g |
| Wrist Rotate | SG90 | 9 g |
| Gripper | SG90 | 9 g |

서보 합계는 192.00 g이며, 나열된 출력 부품과 서보의 합계는 461.22 g입니다.
볼트, 너트, 서보 혼과 배선처럼 목록에 없는 부품은 이 합계에 포함되지 않습니다.

URDF 링크 질량은 각 서보가 실제로 고정된 링크, `shaft_cover`,
`gripper_cover`, `servo_retainer`의 결합 위치와 링크별 무게중심을 확인한 뒤
적용합니다. 질량만 임의의 관성텐서와 함께 넣지는 않습니다.

## 링크 길이

캡처에서 측정한 값을 가장 가까운 5 mm 단위로 정리해 초기 URDF에 사용합니다.

| 구간 | 측정값 | URDF 기준값 |
|---|---:|---:|
| 베이스 높이 | 약 61.0 mm | 60 mm |
| 베이스와 Shoulder 사이 | 약 39.2 mm | 40 mm |
| Shoulder와 Elbow 사이 | 약 120.0 mm | 120 mm |
| Elbow와 Wrist 사이 | 약 119.7 mm | 120 mm |
| Wrist와 Gripper 끝 사이 | 약 125.0 mm | 125 mm |

## STL 방향 보정

- Base: 수직축 기준 180도
- Base cover: 중심 높이 0 mm에 고정
- Central shaft: 중심 높이 60 mm, X축 기준 -90도
- Upper arm: 링크 축 기준 왼쪽 90도, 앞뒤 180도
- Wrist 연결부: 현재 방향에서 Y축 기준 180도
- Gripper: 링크 축 기준 왼쪽 90도

## TODO

- 각 STL의 단위와 원점 확인
- 고정 부품과 움직이는 링크 분류
- 관절 중심, 회전축과 회전 방향 확인
- 관절별 최소·최대 각도 확인
- 각 서보가 고정된 실제 링크와 링크별 무게중심 확인
- 실측 질량과 STL 형상으로 링크별 관성텐서 계산
- `shaft_cover.stl`, `gripper_cover.stl`, `servo_retainer.stl`의
  정확한 결합 위치 확인
- 그리퍼 기어와 4절 링크의 회전 중심 확인
- 측정값을 반영해 `robot_arm.urdf`의 TODO 교체
