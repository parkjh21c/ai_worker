import unittest

from ffw_vision.camera_config import CameraConfig, make_camera_config
from ffw_vision.input_check import InputMonitor, NS_PER_MS, NS_PER_SEC

HEAD = {
    'image_topic': '/zed/zed_node/left/image_rect_color',
    'stream_type': 'rectified',
    'camera_info_topic': '/zed/zed_node/left/camera_info',
    'depth_topic': '',
    'frame_id': 'zed_left_camera_optical_frame',
}
FRAME = HEAD['frame_id']


def make_monitor():
    return InputMonitor(CameraConfig('head', **HEAD), timeout_ns=NS_PER_SEC)


class TestCameraConfig(unittest.TestCase):

    def test_valid(self):
        config = make_camera_config('head', HEAD)
        self.assertEqual(config.camera_id, 'head')
        self.assertEqual(config.depth_topic, '')

    def test_rejects_empty_field(self):
        for field in ('image_topic', 'stream_type', 'camera_info_topic', 'frame_id'):
            with self.subTest(field=field), self.assertRaises(ValueError):
                make_camera_config('head', {**HEAD, field: ''})

    def test_rejects_unknown_values(self):
        with self.assertRaises(ValueError):
            make_camera_config('head', {**HEAD, 'stream_type': 'depth'})
        with self.assertRaises(ValueError):
            make_camera_config('tail', HEAD)


class TestInputMonitor(unittest.TestCase):

    def test_paired_frames_ok(self):
        monitor = make_monitor()
        now = 10 * NS_PER_SEC
        for i in range(3):
            stamp = now + i * 33 * NS_PER_MS
            monitor.on_camera_info(stamp, FRAME, 672, 376)
            monitor.on_image(stamp, FRAME, 672, 376, 'bgra8', stamp + 5 * NS_PER_MS)
        result = monitor.report(now + 100 * NS_PER_MS)
        self.assertEqual(result['state'], 'OK')
        self.assertEqual((result['paired'], result['unpaired']), (3, 0))
        self.assertAlmostEqual(result['max_interval_ms'], 33.0)
        self.assertAlmostEqual(result['latency_max_ms'], 5.0)

    def test_image_before_its_camera_info_is_pending_then_paired(self):
        monitor = make_monitor()
        now = 10 * NS_PER_SEC
        monitor.on_camera_info(now, FRAME, 672, 376)
        monitor.on_image(now, FRAME, 672, 376, 'bgra8', now)
        monitor.on_image(now + 33 * NS_PER_MS, FRAME, 672, 376, 'bgra8', now + 33 * NS_PER_MS)
        first = monitor.report(now + 40 * NS_PER_MS)
        self.assertEqual((first['paired'], first['pending'], first['unpaired']), (1, 1, 0))

        monitor.on_camera_info(now + 33 * NS_PER_MS, FRAME, 672, 376)
        second = monitor.report(now + 50 * NS_PER_MS)
        self.assertEqual((second['paired'], second['pending'], second['unpaired']), (1, 0, 0))

    def test_unpaired_image_reports_gap(self):
        monitor = make_monitor()
        now = 10 * NS_PER_SEC
        monitor.on_camera_info(now, FRAME, 672, 376)
        monitor.on_camera_info(now + 40 * NS_PER_MS, FRAME, 672, 376)
        monitor.on_image(now + 10 * NS_PER_MS, FRAME, 672, 376, 'bgra8', now + 20 * NS_PER_MS)
        result = monitor.report(now + 50 * NS_PER_MS)
        self.assertEqual(result['state'], 'MISMATCH')
        self.assertEqual(result['unpaired'], 1)
        self.assertAlmostEqual(result['max_unpaired_gap_ms'], 10.0)

    def test_frame_id_and_size_mismatch(self):
        monitor = make_monitor()
        now = 10 * NS_PER_SEC
        monitor.on_camera_info(now, FRAME, 672, 376)
        monitor.on_image(now, 'other_frame', 1344, 376, 'bgra8', now)
        result = monitor.report(now)
        self.assertEqual(result['state'], 'MISMATCH')
        self.assertEqual((result['frame_id_errors'], result['size_errors']), (1, 1))

    def test_missing_camera_info(self):
        monitor = make_monitor()
        monitor.on_image(0, FRAME, 672, 376, 'bgra8', 0)
        result = monitor.report(10 * NS_PER_MS)
        self.assertEqual(result['state'], 'NO_CAMERA_INFO')
        self.assertEqual(result['pending'], 1)

    def test_no_input_after_timeout(self):
        monitor = make_monitor()
        self.assertEqual(monitor.report(0)['state'], 'NO_INPUT')
        monitor.on_image(0, FRAME, 672, 376, 'bgra8', 0)
        self.assertEqual(monitor.report(2 * NS_PER_SEC)['state'], 'NO_INPUT')


if __name__ == '__main__':
    unittest.main()
