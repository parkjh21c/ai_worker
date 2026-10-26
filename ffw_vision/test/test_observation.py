import unittest

from ffw_vision.camera_config import CameraConfig
from ffw_vision.frame_registry import FrameInfo
from ffw_vision.observation import make_selected_observation
from ffw_vision_msgs.msg import PixelObservation

CONFIG = CameraConfig('head', '/zed/zed_node/left/image_rect_color', 'rectified',
                      '/zed/zed_node/left/camera_info', '', 'zed_left_camera_optical_frame')
FRAME = FrameInfo(1792740000, 123456789, 'zed_left_camera_optical_frame', 672, 376, 0)


class TestSelectedObservation(unittest.TestCase):

    def test_fields_come_from_the_selected_frame(self):
        msg = make_selected_observation(CONFIG, FRAME, 10.0, 20.0, 'basket')
        self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec),
                         (1792740000, 123456789))
        self.assertEqual(msg.header.frame_id, 'zed_left_camera_optical_frame')
        self.assertEqual((msg.camera_id, msg.image_topic),
                         ('head', '/zed/zed_node/left/image_rect_color'))
        self.assertEqual((msg.image_width, msg.image_height), (672, 376))
        self.assertEqual(msg.point_type, 'selected_point')
        self.assertEqual(msg.source_mode, PixelObservation.SOURCE_MANUAL)
        self.assertEqual(msg.status, PixelObservation.STATUS_VALID)
        self.assertEqual((msg.u, msg.v, msg.target_label), (10.0, 20.0, 'basket'))

    def test_corner_pixels_are_valid(self):
        make_selected_observation(CONFIG, FRAME, 0.0, 0.0)
        make_selected_observation(CONFIG, FRAME, 671.0, 375.0)

    def test_rejects_outside_or_non_finite(self):
        for u, v in ((672.0, 0.0), (0.0, 376.0), (-1.0, 0.0), (float('nan'), 0.0),
                     (0.0, float('inf'))):
            with self.subTest(u=u, v=v), self.assertRaises(ValueError):
                make_selected_observation(CONFIG, FRAME, u, v)


if __name__ == '__main__':
    unittest.main()
