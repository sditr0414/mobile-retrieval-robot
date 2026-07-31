"""URDF, 설정 파일과 PyBullet DIRECT 로딩을 함께 검증한다."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "models" / "robot_arm.urdf"
CONFIG = ROOT / "robot_config.json"
MASS = ROOT / "models" / "mass_properties.json"

EXPECTED_JOINTS = {
    "base_yaw_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_tilt_joint",
    "wrist_rotate_joint",
    "gripper_finger_1_joint",
    "gripper_finger_2_joint",
    "tcp_fixed_joint",
}


def positive_definite_inertia(inertia: ET.Element) -> bool:
    """관성텐서의 선행 주행렬식이 모두 양수인지 확인한다."""
    ixx = float(inertia.attrib["ixx"])
    ixy = float(inertia.attrib["ixy"])
    ixz = float(inertia.attrib["ixz"])
    iyy = float(inertia.attrib["iyy"])
    iyz = float(inertia.attrib["iyz"])
    izz = float(inertia.attrib["izz"])
    minor_1 = ixx
    minor_2 = ixx * iyy - ixy * ixy
    determinant = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - ixz * iyz)
        + ixz * (ixy * iyz - ixz * iyy)
    )
    return minor_1 > 0.0 and minor_2 > 0.0 and determinant > 0.0


def xml_validation() -> None:
    """모델 참조, 메시 경로, 질량 합계와 패킷 방향 설정을 검사한다."""
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    mass = json.loads(MASS.read_text(encoding="utf-8"))
    tree = ET.parse(URDF)
    robot = tree.getroot()
    if robot.tag != "robot":
        raise ValueError("root element is not <robot>")

    links = {link.attrib["name"] for link in robot.findall("link")}
    joints: set[str] = set()
    modeled_mass = 0.0
    for link in robot.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            raise ValueError(f"missing inertial data: {link.attrib['name']}")
        link_mass = float(inertial.find("mass").attrib["value"])
        if link_mass <= 0.0:
            raise ValueError(f"invalid mass: {link.attrib['name']}")
        if not positive_definite_inertia(inertial.find("inertia")):
            raise ValueError(f"invalid inertia: {link.attrib['name']}")
        modeled_mass += link_mass
        for mesh in link.findall("visual/geometry/mesh"):
            mesh_path = URDF.parent / mesh.attrib["filename"]
            if not mesh_path.exists():
                raise FileNotFoundError(mesh_path)

    for joint in robot.findall("joint"):
        name = joint.attrib["name"]
        if name in joints:
            raise ValueError(f"duplicate joint: {name}")
        joints.add(name)
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        if parent not in links or child not in links:
            raise ValueError(f"invalid link reference in {name}")

    if joints != EXPECTED_JOINTS:
        raise ValueError(f"unexpected joints: {sorted(joints ^ EXPECTED_JOINTS)}")
    if config["joint_order"] != [
        "base_yaw_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist_tilt_joint",
        "wrist_rotate_joint",
    ]:
        raise ValueError("joint_order does not match firmware channel order")
    expected_signs = [-1.0, -1.0, 1.0, -1.0, -1.0]
    actual_signs = [
        config["joint_profiles"][name]["app_sign"]
        for name in config["joint_order"]
    ]
    if actual_signs != expected_signs:
        raise ValueError("joint directions do not match current firmware")
    if mass["totals"]["listed_assembly"] != 461.22:
        raise ValueError("unexpected measured assembly mass")
    if not math.isclose(modeled_mass, 0.461221, abs_tol=2.0e-6):
        raise ValueError(f"unexpected modeled mass: {modeled_mass}")
    if config["limits_verified"] is not False:
        raise ValueError("unmeasured hardware limits must remain unverified")

    print(f"XML validation OK: {len(links)} links, {len(joints)} joints")
    print(f"Modeled mass OK: {modeled_mass:.6f} kg")
    print("Joint limits: simulation UI only; not physical safety limits")


def pybullet_validation() -> None:
    """설치된 PyBullet에서 URDF를 GUI 없이 실제로 불러온다."""
    try:
        import pybullet as bullet
    except ImportError:
        print("PyBullet not installed; skipped DIRECT load test")
        return

    client = bullet.connect(bullet.DIRECT)
    try:
        robot = bullet.loadURDF(
            str(URDF),
            useFixedBase=True,
            flags=(
                bullet.URDF_MAINTAIN_LINK_ORDER
                | bullet.URDF_USE_INERTIA_FROM_FILE
            ),
            physicsClientId=client,
        )
        if robot < 0:
            raise RuntimeError("PyBullet failed to load the URDF")
        print(f"PyBullet DIRECT load OK: {bullet.getNumJoints(robot)} joints")
    finally:
        bullet.disconnect(client)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml-only", action="store_true")
    args = parser.parse_args()
    xml_validation()
    if not args.xml_only:
        pybullet_validation()


if __name__ == "__main__":
    main()
