import time
from pathlib import Path

import math
import pybullet as pybullet
import pybullet_data


SIMULATION_STEP_SECONDS = 1.0 / 240.0
MODEL_PATH = Path(__file__).parent / "models" / "robot_arm.urdf"
CONTROLLED_JOINTS = (
    "base_yaw_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_tilt_joint",
    "wrist_rotate_joint",
)


def main():
    """PyBullet GUI를 열고 기본 시뮬레이션 환경을 실행한다."""
    client_id = pybullet.connect(pybullet.GUI)
    if client_id < 0:
        raise RuntimeError("PyBullet GUI에 연결하지 못했습니다.")

    try:
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        # TODO: 질량과 관성을 측정한 뒤 중력을 적용한다.
        pybullet.setGravity(0.0, 0.0, 0.0)
        pybullet.setTimeStep(SIMULATION_STEP_SECONDS)
        pybullet.loadURDF("plane.urdf")

        robot_id = pybullet.loadURDF(
            str(MODEL_PATH),
            useFixedBase=True,
            flags=pybullet.URDF_MAINTAIN_LINK_ORDER,
        )
        pybullet.resetDebugVisualizerCamera(
            cameraDistance=0.55,
            cameraYaw=40.0,
            cameraPitch=-20.0,
            cameraTargetPosition=[0.08, 0.0, 0.18],
        )

        sliders = {}
        for joint_index in range(pybullet.getNumJoints(robot_id)):
            joint_info = pybullet.getJointInfo(robot_id, joint_index)
            joint_name = joint_info[1].decode("utf-8")
            if joint_name in CONTROLLED_JOINTS:
                slider_id = pybullet.addUserDebugParameter(
                    joint_name,
                    -90.0,
                    90.0,
                    0.0,
                )
                sliders[joint_index] = slider_id

        while pybullet.isConnected():
            for joint_index, slider_id in sliders.items():
                angle_deg = pybullet.readUserDebugParameter(slider_id)
                pybullet.setJointMotorControl2(
                    robot_id,
                    joint_index,
                    pybullet.POSITION_CONTROL,
                    targetPosition=math.radians(angle_deg),
                    force=1.0,
                )

            pybullet.stepSimulation()
            time.sleep(SIMULATION_STEP_SECONDS)
    finally:
        if pybullet.isConnected():
            pybullet.disconnect()


if __name__ == "__main__":
    main()
