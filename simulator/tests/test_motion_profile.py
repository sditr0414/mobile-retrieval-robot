import unittest

from motion_profile import MotionLimits, OnlineJerkLimitedAxis


class MotionProfileTests(unittest.TestCase):
    def test_limits_and_convergence(self):
        dt = 0.02
        limits = MotionLimits(1.0, 2.0, 10.0, 1e-4, 1e-3, 1e-2)
        axis = OnlineJerkLimitedAxis(0.0, limits, -2.0, 2.0)
        axis.set_target(1.0)

        for _ in range(2000):
            state = axis.update(dt)
            self.assertLessEqual(abs(state.velocity), limits.max_velocity + 1e-9)
            self.assertLessEqual(
                abs(state.acceleration),
                limits.max_acceleration + 1e-9,
            )
            if state.settled:
                break
        self.assertTrue(axis.state.settled)
        self.assertAlmostEqual(axis.state.position, 1.0, places=6)

    def test_retarget_preserves_velocity(self):
        limits = MotionLimits(1.0, 2.0, 10.0)
        axis = OnlineJerkLimitedAxis(0.0, limits)
        axis.set_target(1.0)
        for _ in range(20):
            axis.update(0.02)
        velocity = axis.state.velocity
        axis.set_target(-0.5)
        self.assertAlmostEqual(axis.state.velocity, velocity)

    def test_target_is_clamped_to_axis_range(self):
        axis = OnlineJerkLimitedAxis(
            0.0,
            MotionLimits(1.0, 2.0, 10.0),
            -0.5,
            0.5,
        )
        axis.set_target(1.0)
        self.assertEqual(axis.target, 0.5)


if __name__ == "__main__":
    unittest.main()
