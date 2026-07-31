"""현재 펌웨어 좌표계로 로봇팔을 시험하는 PyBullet 시뮬레이터."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

from motion_profile import MotionLimits, OnlineJerkLimitedAxis
from protocol import encode_arm_packet, format_hex


ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "robot_config.json"


def parse_args() -> argparse.Namespace:
    """명령행 실행 모드와 초기 자세를 읽는다."""
    parser = argparse.ArgumentParser(description="Mobile retrieval robot arm simulator")
    parser.add_argument(
        "--pose",
        choices=("origin", "travel"),
        default="origin",
        help="startup pose matching firmware terminology",
    )
    parser.add_argument("--direct", action="store_true", help="run without a GUI")
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="stop after this many seconds; DIRECT defaults to one second",
    )
    parser.add_argument(
        "--emit-packets",
        action="store_true",
        help="print a fixed 16-byte Mode 1 packet when the target changes",
    )
    parser.add_argument(
        "--speed",
        type=int,
        choices=range(50, 101),
        default=50,
        metavar="50..100",
        help="speed byte used only by emitted firmware packets",
    )
    parser.add_argument(
        "--no-realtime-sleep",
        action="store_true",
        help="run faster than wall clock; useful with --direct",
    )
    return parser.parse_args()


def load_config() -> dict[str, Any]:
    """저장소의 좌표계, 자세와 시뮬레이션 값을 읽는다."""
    with CONFIG_PATH.open("r", encoding="utf-8") as file:
        return json.load(file)


def app_to_urdf_deg(app_deg: float, profile: dict[str, Any]) -> float:
    """앱 표시 각도를 현재 펌웨어 조립 방향의 URDF 각도로 바꾼다."""
    return profile["app_sign"] * app_deg + profile["app_offset_deg"]


def interpolate(
    value: float,
    in_low: float,
    in_high: float,
    out_low: float,
    out_high: float,
) -> float:
    """한 범위의 값을 다른 범위로 선형 변환한다."""
    ratio = (value - in_low) / (in_high - in_low)
    ratio = max(0.0, min(1.0, ratio))
    return out_low + ratio * (out_high - out_low)


def build_joint_axis(
    initial_rad: float,
    config: dict[str, Any],
) -> OnlineJerkLimitedAxis:
    """시뮬레이션 전용 한계로 부드럽게 움직이는 관절 축을 만든다."""
    profile = config["motion_profile"]
    lower_deg, upper_deg = config["ui_limits_deg"]
    limits = MotionLimits(
        max_velocity=math.radians(profile["max_velocity_deg_s"]),
        max_acceleration=math.radians(profile["max_acceleration_deg_s2"]),
        max_jerk=math.radians(profile["max_jerk_deg_s3"]),
        position_tolerance=math.radians(0.1),
        velocity_tolerance=math.radians(0.2),
        acceleration_tolerance=math.radians(1.0),
    )
    return OnlineJerkLimitedAxis(
        initial_rad,
        limits,
        math.radians(lower_deg),
        math.radians(upper_deg),
    )


def build_gripper_axis(
    initial_rad: float,
    lower_rad: float,
    upper_rad: float,
    config: dict[str, Any],
) -> OnlineJerkLimitedAxis:
    """그리퍼 한쪽 링크에 사용할 부드러운 운동 축을 만든다."""
    profile = config["gripper"]["motion_profile"]
    return OnlineJerkLimitedAxis(
        initial_rad,
        MotionLimits(
            profile["max_velocity_rad_s"],
            profile["max_acceleration_rad_s2"],
            profile["max_jerk_rad_s3"],
            position_tolerance=0.001,
            velocity_tolerance=0.005,
            acceleration_tolerance=0.02,
        ),
        min(lower_rad, upper_rad),
        max(lower_rad, upper_rad),
    )


def add_joint_sliders(
    bullet: Any,
    config: dict[str, Any],
    initial_app: list[float],
) -> dict[str, int]:
    """논리 관절 5개와 그리퍼 퍼센트 슬라이더를 만든다."""
    lower, upper = config["ui_limits_deg"]
    sliders: dict[str, int] = {}
    for index, joint_name in enumerate(config["joint_order"]):
        profile = config["joint_profiles"][joint_name]
        sliders[joint_name] = bullet.addUserDebugParameter(
            profile["app_name"],
            lower,
            upper,
            initial_app[index],
        )
    sliders["gripper_percent"] = bullet.addUserDebugParameter(
        config["gripper"]["app_name"],
        0.0,
        100.0,
        config["poses"]["startup_gripper_percent"],
    )
    return sliders


def joint_indices_by_name(bullet: Any, robot_id: int) -> dict[str, int]:
    """URDF 관절 이름을 PyBullet 관절 번호에 연결한다."""
    indices: dict[str, int] = {}
    for index in range(bullet.getNumJoints(robot_id)):
        name = bullet.getJointInfo(robot_id, index)[1].decode("utf-8")
        if name in indices:
            raise ValueError(f"duplicate URDF joint: {name}")
        indices[name] = index
    return indices


def main() -> int:
    """PyBullet 환경을 열어 관절 목표를 부드럽게 추종한다."""
    args = parse_args()
    config = load_config()
    if args.duration is not None and args.duration <= 0.0:
        raise ValueError("--duration must be greater than zero")
    duration = 1.0 if args.direct and args.duration is None else args.duration

    try:
        import pybullet as bullet
        import pybullet_data
    except ImportError as exc:
        print(
            "PyBullet is not installed. Run `python -m pip install -r requirements.txt`.",
            file=sys.stderr,
        )
        raise SystemExit(2) from exc

    client_id = bullet.connect(bullet.DIRECT if args.direct else bullet.GUI)
    if client_id < 0:
        raise RuntimeError("PyBullet에 연결하지 못했습니다.")

    try:
        simulation = config["simulation"]
        physics_dt = float(simulation["physics_dt_s"])
        control_dt = float(simulation["control_dt_s"])
        steps_per_control = round(control_dt / physics_dt)
        if not math.isclose(
            steps_per_control * physics_dt,
            control_dt,
            abs_tol=1.0e-12,
        ):
            raise ValueError("control_dt_s must be a multiple of physics_dt_s")

        bullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        bullet.setPhysicsEngineParameter(
            fixedTimeStep=physics_dt,
            numSolverIterations=100,
            physicsClientId=client_id,
        )
        bullet.setGravity(
            0.0,
            0.0,
            float(simulation["gravity_m_s2"]),
            physicsClientId=client_id,
        )
        bullet.loadURDF("plane.urdf", physicsClientId=client_id)
        urdf_path = ROOT / config["model"]["urdf"]
        robot_id = bullet.loadURDF(
            str(urdf_path),
            useFixedBase=bool(config["model"]["fixed_base"]),
            flags=(
                bullet.URDF_MAINTAIN_LINK_ORDER
                | bullet.URDF_USE_INERTIA_FROM_FILE
            ),
            physicsClientId=client_id,
        )
        if robot_id < 0:
            raise RuntimeError(f"URDF를 불러오지 못했습니다: {urdf_path}")

        indices = joint_indices_by_name(bullet, robot_id)
        required = [
            *config["joint_order"],
            "gripper_finger_1_joint",
            "gripper_finger_2_joint",
        ]
        missing = [name for name in required if name not in indices]
        if missing:
            raise ValueError(f"URDF missing joints: {', '.join(missing)}")

        pose_key = "origin_app_deg" if args.pose == "origin" else "travel_app_deg"
        initial_app = [float(value) for value in config["poses"][pose_key]]
        initial_urdf = [
            math.radians(
                app_to_urdf_deg(
                    initial_app[index],
                    config["joint_profiles"][joint_name],
                )
            )
            for index, joint_name in enumerate(config["joint_order"])
        ]
        axes = {
            joint_name: build_joint_axis(initial_urdf[index], config)
            for index, joint_name in enumerate(config["joint_order"])
        }
        for joint_name, position in zip(
            config["joint_order"],
            initial_urdf,
            strict=True,
        ):
            bullet.resetJointState(
                robot_id,
                indices[joint_name],
                position,
                physicsClientId=client_id,
            )

        gripper = config["gripper"]
        finger_1_axis = build_gripper_axis(
            gripper["finger_1_open_rad"],
            gripper["finger_1_open_rad"],
            gripper["finger_1_closed_rad"],
            config,
        )
        finger_2_axis = build_gripper_axis(
            gripper["finger_2_open_rad"],
            gripper["finger_2_closed_rad"],
            gripper["finger_2_open_rad"],
            config,
        )
        bullet.resetJointState(
            robot_id,
            indices["gripper_finger_1_joint"],
            finger_1_axis.state.position,
            physicsClientId=client_id,
        )
        bullet.resetJointState(
            robot_id,
            indices["gripper_finger_2_joint"],
            finger_2_axis.state.position,
            physicsClientId=client_id,
        )

        sliders: dict[str, int] = {}
        if not args.direct:
            bullet.resetDebugVisualizerCamera(
                cameraDistance=0.62,
                cameraYaw=40.0,
                cameraPitch=-22.0,
                cameraTargetPosition=[0.08, 0.0, 0.20],
                physicsClientId=client_id,
            )
            sliders = add_joint_sliders(bullet, config, initial_app)
        app_targets = list(initial_app)
        gripper_percent = float(config["poses"]["startup_gripper_percent"])
        transaction_id = 1
        last_packet_target: tuple[float, ...] | None = None
        status_text_id = -1
        status = "Ready"
        start = time.perf_counter()
        next_deadline = start
        step_count = 0

        while bullet.isConnected(client_id):
            elapsed = time.perf_counter() - start
            if duration is not None and elapsed >= duration:
                break

            if step_count % steps_per_control == 0:
                if sliders:
                    gripper_percent = bullet.readUserDebugParameter(
                        sliders["gripper_percent"]
                    )
                    app_targets = [
                        bullet.readUserDebugParameter(sliders[joint_name])
                        for joint_name in config["joint_order"]
                    ]
                    model_targets = [
                        math.radians(
                            app_to_urdf_deg(
                                app_targets[index],
                                config["joint_profiles"][joint_name],
                            )
                        )
                        for index, joint_name in enumerate(config["joint_order"])
                    ]
                    status = "Joint control"
                else:
                    model_targets = [
                        axes[name].target for name in config["joint_order"]
                    ]

                for name, target in zip(
                    config["joint_order"],
                    model_targets,
                    strict=True,
                ):
                    axes[name].set_target(target)
                finger_1_axis.set_target(
                    interpolate(
                        gripper_percent,
                        0.0,
                        100.0,
                        gripper["finger_1_open_rad"],
                        gripper["finger_1_closed_rad"],
                    )
                )
                finger_2_axis.set_target(
                    interpolate(
                        gripper_percent,
                        0.0,
                        100.0,
                        gripper["finger_2_open_rad"],
                        gripper["finger_2_closed_rad"],
                    )
                )

                packet_target = tuple(
                    round(value, 2) for value in (*app_targets, gripper_percent)
                )
                if args.emit_packets and packet_target != last_packet_target:
                    packet = encode_arm_packet(
                        app_targets,
                        gripper_percent,
                        transaction_id=transaction_id,
                        speed_percent=args.speed,
                    )
                    print(format_hex(packet))
                    transaction_id = 1 if transaction_id >= 255 else transaction_id + 1
                    last_packet_target = packet_target

                for name in config["joint_order"]:
                    state = axes[name].update(control_dt)
                    bullet.setJointMotorControl2(
                        robot_id,
                        indices[name],
                        bullet.POSITION_CONTROL,
                        targetPosition=state.position,
                        force=float(simulation["arm_force_nm"]),
                        positionGain=float(simulation["position_gain"]),
                        velocityGain=float(simulation["velocity_gain"]),
                        physicsClientId=client_id,
                    )
                for name, axis in (
                    ("gripper_finger_1_joint", finger_1_axis),
                    ("gripper_finger_2_joint", finger_2_axis),
                ):
                    state = axis.update(control_dt)
                    bullet.setJointMotorControl2(
                        robot_id,
                        indices[name],
                        bullet.POSITION_CONTROL,
                        targetPosition=state.position,
                        force=float(simulation["gripper_force_nm"]),
                        positionGain=float(simulation["position_gain"]),
                        velocityGain=float(simulation["velocity_gain"]),
                        physicsClientId=client_id,
                    )

                if sliders:
                    status_text_id = bullet.addUserDebugText(
                        (
                            f"{status}\n"
                            "Joint and motion limits are simulation-only."
                        ),
                        [0.0, 0.0, 0.42],
                        textColorRGB=[0.1, 0.1, 0.1],
                        textSize=1.2,
                        replaceItemUniqueId=status_text_id,
                        physicsClientId=client_id,
                    )

            bullet.stepSimulation(physicsClientId=client_id)
            step_count += 1
            if not args.no_realtime_sleep:
                next_deadline += physics_dt
                remaining = next_deadline - time.perf_counter()
                if remaining > 0.0:
                    time.sleep(remaining)
        return 0
    finally:
        if bullet.isConnected(client_id):
            bullet.disconnect(client_id)


if __name__ == "__main__":
    raise SystemExit(main())
