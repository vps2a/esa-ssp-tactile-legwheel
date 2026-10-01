import math
import unittest

from legwheel_dataset.frame_playback import FramePose, interpolate_frame_pose


class TestFramePlayback(unittest.TestCase):
    def test_empty_and_outside_packet_range_have_no_pose(self):
        poses = [FramePose(100, 1.0, -0.1), FramePose(200, 2.0, -0.2)]

        self.assertIsNone(interpolate_frame_pose([], 150))
        self.assertIsNone(interpolate_frame_pose(poses, 99))
        self.assertIsNone(interpolate_frame_pose(poses, 201))

    def test_exact_packet_timestamp_preserves_saved_pose(self):
        poses = [FramePose(100, 1.0, -0.1), FramePose(200, 2.0, -0.2)]

        self.assertEqual(interpolate_frame_pose(poses, 100), poses[0])
        self.assertEqual(interpolate_frame_pose(poses, 200), poses[1])

    def test_interpolates_unwrapped_theta_y_and_theta_p(self):
        poses = [
            FramePose(100, 2.0 * math.pi - 0.1, -0.1),
            FramePose(200, 2.0 * math.pi + 0.1, -0.3),
        ]

        result = interpolate_frame_pose(poses, 150)

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.theta_y_rad, 2.0 * math.pi)
        self.assertAlmostEqual(result.theta_p_rad, -0.2)
        self.assertEqual(result.timestamp_ns, 150)

    def test_single_pose_is_returned_only_at_its_timestamp(self):
        poses = [FramePose(100, 1.0, -0.1)]

        self.assertEqual(interpolate_frame_pose(poses, 100), poses[0])
        self.assertIsNone(interpolate_frame_pose(poses, 101))


if __name__ == "__main__":
    unittest.main()
