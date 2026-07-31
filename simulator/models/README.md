# 로봇팔 모델

`robot_arm.urdf`와 영문 `snake_case` STL을 보관합니다. 질량은 실측값이며
무게중심·관성텐서와 충돌 형상은 PyBullet용 근사값입니다.

## 질량

| 구분 | 합계 |
|---|---:|
| 출력 부품 16개 | 269.22 g |
| MG996R 3개 | 165.00 g |
| SG90 3개 | 27.00 g |
| URDF 전체 | 461.22 g |

볼트, 너트, 혼과 배선은 포함하지 않습니다. 부품별 값은
`mass_properties.json`, 검증 상태는 `model_parameters.json`이 기준입니다.

## 링크 길이

| 구간 | URDF |
|---|---:|
| 베이스 높이 | 60 mm |
| Base–Shoulder | 40 mm |
| Shoulder–Elbow | 120 mm |
| Elbow–Wrist | 120 mm |
| Wrist–Gripper 끝 | 125 mm |

## STL 방향

- Base: 수직축 180°
- Base cover: 높이 0 mm
- Central shaft: 높이 60 mm, X축 -90°
- Upper arm: 링크축 왼쪽 90°, 앞뒤 180°
- Wrist: Y축 180°
- Gripper: 링크축 왼쪽 90°

Base와 Base cover는 고정부품입니다. Gripper는 반대 방향으로 회전하는 두
링크로 근사합니다. 관절 제한, 토크와 충돌 안전 판단에는 사용하지 않습니다.

기준 이미지는 `left_side_view.png`, `right_side_view.png`, `top_view.png`,
`bottom_view.png`, `back_view.png`, `gripper_front_view.png`입니다.
