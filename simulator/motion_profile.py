"""슬라이더 목표가 바뀌어도 속도를 끊지 않는 저크 제한 운동 프로파일."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MotionLimits:
    """한 축에 적용할 속도·가속도·저크와 정착 오차 한계다."""

    max_velocity: float
    max_acceleration: float
    max_jerk: float
    position_tolerance: float = 1.0e-4
    velocity_tolerance: float = 1.0e-3
    acceleration_tolerance: float = 1.0e-2

    def __post_init__(self) -> None:
        values = (
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.position_tolerance,
            self.velocity_tolerance,
            self.acceleration_tolerance,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("all motion limits must be positive finite values")


@dataclass
class MotionState:
    """매 제어 주기에 갱신되는 위치·속도·가속도 상태다."""

    position: float
    velocity: float = 0.0
    acceleration: float = 0.0
    settled: bool = True


class OnlineJerkLimitedAxis:
    """새 목표에서도 현재 속도를 보존하는 온라인 완화형 S-curve 축이다."""

    def __init__(
        self,
        initial_position: float,
        limits: MotionLimits,
        lower_position: float = -math.inf,
        upper_position: float = math.inf,
    ) -> None:
        if lower_position >= upper_position:
            raise ValueError("lower_position must be below upper_position")
        self.limits = limits
        self.lower_position = lower_position
        self.upper_position = upper_position
        self.target = self._clamp(initial_position)
        self.state = MotionState(position=self.target)

    def _clamp(self, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("position must be finite")
        return max(self.lower_position, min(self.upper_position, value))

    def set_target(self, target: float) -> None:
        """현재 속도를 초기화하지 않고 새 목표 위치만 갱신한다."""
        self.target = self._clamp(target)
        self.state.settled = False

    @staticmethod
    def _approach(current: float, target: float, maximum_delta: float) -> float:
        delta = max(-maximum_delta, min(maximum_delta, target - current))
        return current + delta

    def update(self, dt: float) -> MotionState:
        """한 제어 주기만큼 위치를 갱신하고 내부 상태를 반환한다."""
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be a positive finite value")

        error = self.target - self.state.position
        limits = self.limits
        if (
            abs(error) <= limits.position_tolerance
            and abs(self.state.velocity) <= limits.velocity_tolerance
            and abs(self.state.acceleration) <= limits.acceleration_tolerance
        ):
            self.state.position = self.target
            self.state.velocity = 0.0
            self.state.acceleration = 0.0
            self.state.settled = True
            return self.state

        direction = 1.0 if error >= 0.0 else -1.0
        stopping_speed = math.sqrt(max(0.0, 2.0 * limits.max_acceleration * abs(error)))
        reference_velocity = direction * min(limits.max_velocity, stopping_speed)
        requested_acceleration = (reference_velocity - self.state.velocity) / dt
        requested_acceleration = max(
            -limits.max_acceleration,
            min(limits.max_acceleration, requested_acceleration),
        )
        self.state.acceleration = self._approach(
            self.state.acceleration,
            requested_acceleration,
            limits.max_jerk * dt,
        )

        previous_velocity = self.state.velocity
        self.state.velocity += self.state.acceleration * dt
        self.state.velocity = max(
            -limits.max_velocity,
            min(limits.max_velocity, self.state.velocity),
        )
        next_position = self.state.position + 0.5 * (
            previous_velocity + self.state.velocity
        ) * dt

        # 목표를 지나친 경우 진동하지 않도록 목표에서 정착시킨다.
        if (self.target - self.state.position) * (self.target - next_position) <= 0.0:
            self.state.position = self.target
            self.state.velocity = 0.0
            self.state.acceleration = 0.0
            self.state.settled = True
        else:
            self.state.position = self._clamp(next_position)
            self.state.settled = False
        return self.state
