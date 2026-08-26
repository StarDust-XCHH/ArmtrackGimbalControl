import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / "tools"))

from gimbal_protocol import (
    command_pitch_position,
    command_pitch_speed,
    command_pose,
    command_track,
    command_yaw_position,
    command_yaw_speed,
    command_tick,
    describe_line,
    parse_angle,
    status_from_line,
)


class ProtocolTests(unittest.TestCase):
    def test_angle_ranges_and_raw_values(self):
        self.assertEqual(parse_angle("359.9", 0, 3599), 3599)
        self.assertEqual(parse_angle("x2300", 2300, 4050), 2300)
        self.assertEqual(parse_angle("330.05", 2300, 4050), 3301)
        self.assertEqual(parse_angle("330.09", 2300, 4050), 3301)
        self.assertEqual(command_yaw_position("130"), "yawpos 130.0")
        self.assertEqual(command_pitch_position("405"), "pitch 405.0")

    def test_yaw_angle_rejected(self):
        for value in ("-0.1", "360", "x3600"):
            with self.assertRaises(ValueError):
                command_yaw_position(value)

    def test_speed_ranges(self):
        self.assertEqual(command_yaw_speed(100), "yaw 100")
        self.assertEqual(command_pitch_speed(-30), "pitchspd -30")
        with self.assertRaises(ValueError):
            command_yaw_speed(101)
        with self.assertRaises(ValueError):
            command_pitch_speed(31)

    def test_compound_commands(self):
        self.assertEqual(command_track(10, "330"), "track 10 330.0")
        self.assertEqual(command_pose("130", "330"), "pose 130.0 330.0")

    def test_status_and_tick_parsing(self):
        status = status_from_line("yaw_cur:130.0,yaw_tgt:130.0,pitch_cur:330.0,pitch_tgt:330.0,homed:1,state:3")
        self.assertIsNotNone(status)
        self.assertEqual(status.yaw_current, "130.0")
        self.assertIn("sequence=4", describe_line("TOCK 4 1234"))
        self.assertEqual(command_tick(4), "tick 4")


if __name__ == "__main__":
    unittest.main()
