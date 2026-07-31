import unittest

from protocol import (
    PACKET_SIZE,
    calculate_checksum,
    encode_arm_packet,
    is_valid_packet,
)


class ProtocolTests(unittest.TestCase):
    def test_origin_packet_matches_firmware_layout(self):
        packet = encode_arm_packet(
            [0.0, 0.0, 0.0, 0.0, 0.0],
            0.0,
            transaction_id=7,
            speed_percent=50,
        )
        self.assertEqual(len(packet), PACKET_SIZE)
        self.assertEqual(
            list(packet),
            [
                0xAA,
                0x01,
                90,
                90,
                90,
                90,
                90,
                0,
                0,
                7,
                50,
                0,
                0,
                0,
                0xFC,
                0x55,
            ],
        )
        self.assertEqual(packet[14], calculate_checksum(packet))
        self.assertTrue(is_valid_packet(packet))

    def test_travel_pose_keeps_gripper_value(self):
        packet = encode_arm_packet(
            [0.0, -50.0, 90.0, 0.0, 60.0],
            75.0,
            transaction_id=255,
            speed_percent=100,
        )
        self.assertEqual(list(packet[2:8]), [90, 40, 180, 90, 150, 135])
        self.assertEqual(list(packet[8:14]), [0, 255, 100, 0, 0, 0])
        self.assertTrue(is_valid_packet(packet))

    def test_invalid_control_is_rejected(self):
        with self.assertRaises(ValueError):
            encode_arm_packet([0.0] * 5, 0.0, control=4)

    def test_travel_control_uses_reserved_zero_angles(self):
        packet = encode_arm_packet(
            [30.0, -20.0, 10.0, 40.0, -50.0],
            75.0,
            control=3,
        )

        self.assertEqual(list(packet[2:8]), [0] * 6)
        self.assertEqual(packet[8], 3)
        self.assertTrue(is_valid_packet(packet))

    def test_transaction_zero_is_rejected(self):
        with self.assertRaises(ValueError):
            encode_arm_packet([0.0] * 5, 0.0, transaction_id=0)

    def test_reserved_field_is_rejected(self):
        packet = bytearray(encode_arm_packet([0.0] * 5, 0.0))
        packet[11] = 1
        packet[14] = calculate_checksum(packet)
        self.assertFalse(is_valid_packet(packet))


if __name__ == "__main__":
    unittest.main()
