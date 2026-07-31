"""펌웨어의 상대 Yaw PID 계산을 물리 차체 없이 재현한다."""

from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_yaw_error(error_deg: float) -> float:
    """각도 오차를 -180~+180도 최단 방향으로 정규화한다."""
    while error_deg > 180.0:
        error_deg -= 360.0
    while error_deg < -180.0:
        error_deg += 360.0
    return error_deg


@dataclass(frozen=True)
class PidGains:
    """Flash에 저장되는 P/I/D 실수 계수다."""

    kp: float
    ki: float
    kd: float

    def __post_init__(self) -> None:
        values = (self.kp, self.ki, self.kd)
        if (
            any(
                not math.isfinite(value) or not 0.0 <= value <= 100.0
                for value in values
            )
            or all(value == 0.0 for value in values)
        ):
            raise ValueError("PID gains must be 0..100 and not all zero")


@dataclass(frozen=True)
class PidResult:
    """한 샘플에서 계산된 PID 상태와 최종 PWM이다."""

    target_yaw_deg: float
    current_yaw_deg: float
    error_deg: float
    output: float
    correction: float
    left_pwm: int
    right_pwm: int
    target_updated: bool


class RelativeYawPid:
    """직진 구간이 시작될 때 현재 Yaw를 목표로 잡는 펌웨어 동등 제어기다."""

    def __init__(
        self,
        gains: PidGains,
        *,
        correction_sign: float = -1.0,
        integral_limit: float = 50.0,
        output_limit: float = 80.0,
    ) -> None:
        if correction_sign not in (-1.0, 1.0):
            raise ValueError("correction_sign must be -1 or 1")
        if integral_limit <= 0.0 or output_limit <= 0.0:
            raise ValueError("PID limits must be positive")
        self.gains = gains
        self.correction_sign = correction_sign
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.reset()

    def reset(self) -> None:
        """정지·회전·안전 인터록과 같은 새 주행 구간을 준비한다."""
        self.target_yaw_deg = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.target_valid = False

    @staticmethod
    def _clamp_pwm(value: float) -> int:
        return max(0, min(255, int(value)))

    def update(
        self,
        current_yaw_deg: float,
        dt_sec: float,
        *,
        left_pwm: int,
        right_pwm: int,
        reverse: bool = False,
        refresh_target: bool = False,
    ) -> PidResult:
        """새 Yaw 샘플로 좌우 PWM 보정값을 한 번 계산한다."""
        if not math.isfinite(current_yaw_deg):
            raise ValueError("current_yaw_deg must be finite")
        if not math.isfinite(dt_sec) or not 0.0 < dt_sec <= 0.2:
            raise ValueError("dt_sec must be in 0..0.2 seconds")
        if not 0 <= left_pwm <= 255 or not 0 <= right_pwm <= 255:
            raise ValueError("PWM must be in 0..255")

        target_updated = refresh_target or not self.target_valid
        if target_updated:
            self.target_yaw_deg = current_yaw_deg
            self.integral = 0.0
            self.previous_error = 0.0
            self.target_valid = True

        error = wrap_yaw_error(self.target_yaw_deg - current_yaw_deg)
        derivative = (error - self.previous_error) / dt_sec
        self.integral += self.gains.ki * error * dt_sec
        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral),
        )
        output = (
            self.gains.kp * error
            + self.integral
            + self.gains.kd * derivative
        )
        output = max(-self.output_limit, min(self.output_limit, output))
        correction = self.correction_sign * output
        if reverse:
            correction = -correction

        self.previous_error = error
        return PidResult(
            target_yaw_deg=self.target_yaw_deg,
            current_yaw_deg=current_yaw_deg,
            error_deg=error,
            output=output,
            correction=correction,
            left_pwm=self._clamp_pwm(left_pwm + correction),
            right_pwm=self._clamp_pwm(right_pwm - correction),
            target_updated=target_updated,
        )
