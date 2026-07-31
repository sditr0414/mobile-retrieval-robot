import unittest

from drive_pid import PidGains, RelativeYawPid, wrap_yaw_error


class DrivePidTests(unittest.TestCase):
    def test_wrap_uses_shortest_error(self):
        self.assertAlmostEqual(wrap_yaw_error(358.0), -2.0)
        self.assertAlmostEqual(wrap_yaw_error(-358.0), 2.0)

    def test_first_sample_sets_relative_target(self):
        pid = RelativeYawPid(PidGains(1.0, 0.0, 0.05))
        result = pid.update(37.0, 0.02, left_pwm=120, right_pwm=120)
        self.assertTrue(result.target_updated)
        self.assertEqual(result.error_deg, 0.0)
        self.assertEqual((result.left_pwm, result.right_pwm), (120, 120))

    def test_forward_and_reverse_use_opposite_correction(self):
        forward = RelativeYawPid(PidGains(1.0, 0.0, 0.0))
        reverse = RelativeYawPid(PidGains(1.0, 0.0, 0.0))
        forward.update(0.0, 0.02, left_pwm=120, right_pwm=120)
        reverse.update(0.0, 0.02, left_pwm=120, right_pwm=120, reverse=True)

        forward_result = forward.update(10.0, 0.02, left_pwm=120, right_pwm=120)
        reverse_result = reverse.update(
            10.0,
            0.02,
            left_pwm=120,
            right_pwm=120,
            reverse=True,
        )
        self.assertEqual(forward_result.correction, -reverse_result.correction)

    def test_output_and_pwm_are_saturated(self):
        pid = RelativeYawPid(PidGains(100.0, 100.0, 100.0))
        pid.update(0.0, 0.02, left_pwm=250, right_pwm=250)
        result = pid.update(90.0, 0.02, left_pwm=250, right_pwm=250)
        self.assertEqual(abs(result.output), 80.0)
        self.assertTrue(0 <= result.left_pwm <= 255)
        self.assertTrue(0 <= result.right_pwm <= 255)


if __name__ == "__main__":
    unittest.main()
