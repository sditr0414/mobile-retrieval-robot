"""현재 펌웨어와 동일한 고정 16바이트 로봇팔 패킷을 만든다."""

from __future__ import annotations

from collections.abc import Sequence


PACKET_SIZE = 16
HEADER = 0xAA
TAIL = 0x55
MODE_ARM = 0x01
CHECKSUM_INDEX = 14


def _clamp_int(value: float, lower: int, upper: int) -> int:
    """반올림한 값을 패킷에서 허용하는 정수 범위로 제한한다."""
    return max(lower, min(upper, round(value)))


def calculate_checksum(packet: Sequence[int]) -> int:
    """Mode부터 Data11까지 더한 값의 하위 8비트를 반환한다."""
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"packet must contain {PACKET_SIZE} bytes")
    return sum(packet[1:CHECKSUM_INDEX]) & 0xFF


def encode_arm_packet(
    app_angles_deg: Sequence[float],
    gripper_percent: float,
    *,
    control: int = 0,
    transaction_id: int = 1,
    speed_percent: int = 50,
) -> bytes:
    """앱 각도 5개와 그리퍼 값을 펌웨어 Mode 1 패킷으로 변환한다."""
    if len(app_angles_deg) != 5:
        raise ValueError("exactly five arm joint angles are required")
    if control not in (0, 1, 2, 3):
        raise ValueError("control must be between 0 and 3")
    if not 1 <= transaction_id <= 0xFF:
        raise ValueError("transaction_id must be between 1 and 255")
    if not 50 <= speed_percent <= 100:
        raise ValueError("speed_percent must be between 50 and 100")

    joint_data = [_clamp_int(angle + 90.0, 0, 180) for angle in app_angles_deg]
    gripper_data = _clamp_int(gripper_percent * 1.8, 0, 180)
    if control == 3:
        # 이동 자세 복귀는 펌웨어가 Flash 자세와 현재 Gripper를 조합한다.
        # 프로토콜 규격에 따라 앱이 보낸 관절 필드는 모두 0이어야 한다.
        joint_data = [0] * 5
        gripper_data = 0
    data = [
        *joint_data,
        gripper_data,
        control,
        transaction_id,
        speed_percent,
        0,
        0,
        0,
    ]
    packet = [HEADER, MODE_ARM, *data, 0, TAIL]
    packet[CHECKSUM_INDEX] = calculate_checksum(packet)
    return bytes(packet)


def is_valid_packet(packet: Sequence[int]) -> bool:
    """로봇팔 명령의 프레임, 필드 범위와 체크섬을 검사한다."""
    if (
        len(packet) != PACKET_SIZE
        or packet[0] != HEADER
        or packet[1] != MODE_ARM
        or packet[CHECKSUM_INDEX] != calculate_checksum(packet)
        or packet[15] != TAIL
    ):
        return False

    return (
        all(0 <= value <= 180 for value in packet[2:8])
        and packet[8] in (0, 1, 2, 3)
        and (packet[8] != 3 or list(packet[2:8]) == [0] * 6)
        and 1 <= packet[9] <= 255
        and 50 <= packet[10] <= 100
        and list(packet[11:14]) == [0, 0, 0]
    )


def format_hex(packet: Sequence[int]) -> str:
    """패킷을 시리얼 모니터에서 읽기 쉬운 16진수 문자열로 만든다."""
    return " ".join(f"{value:02X}" for value in packet)
