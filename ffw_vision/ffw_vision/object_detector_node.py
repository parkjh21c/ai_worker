import math

from ffw_vision.blob_detector import detect, DetectorParams
from ffw_vision.camera_config import declare_camera_parameters, load_camera_config
from ffw_vision.depth_stats import color_from_msg, depth_from_msg
from ffw_vision.input_check import stamp_to_ns
from ffw_vision.observation import make_object_array
from ffw_vision.overlay import draw_blobs
from ffw_vision_msgs.msg import ObjectObservationArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

BUFFER_SIZE = 30
LOG_PERIOD_SEC = 2.0


def image_msg(header, bgr):
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    msg.data = bgr.tobytes()
    return msg


def blob_summary(b):
    # h = top surface median / top level in mm, f = local floor, shift = color blob center offset
    shift = math.hypot(b.u - b.color_box[0], b.v - b.color_box[1]) if b.color_box else 0.0
    return (f'({b.u:.0f},{b.v:.0f} {b.length:.0f}x{b.width:.0f} {b.pose_label}'
            f' h={b.height * 1000:.0f}/{b.top_height * 1000:.0f} f={b.floor:.3f}'
            f' shift={shift:.1f} a={b.area}{" AMBIGUOUS" if b.ambiguous else ""})')


class ObjectDetectorNode(Node):
    """Rule-based detector: color + aligned depth in, ObjectObservationArray out."""

    def __init__(self):
        super().__init__('object_detector')
        declare_camera_parameters(self)
        self.declare_parameter('target_label', 'object')
        self.declare_parameter('depth_scale', 0.001)  # meters per 16UC1 unit
        self.declare_parameter('publish_debug_image', True)
        defaults = DetectorParams()
        self.declare_parameter('roi', list(defaults.roi))
        for name, value in vars(defaults).items():
            if name != 'roi':
                self.declare_parameter(name, value)
        self.config = load_camera_config(self)
        values = {name: self.get_parameter(name).value for name in vars(defaults)}
        values['roi'] = tuple(values['roi'])
        self.params = DetectorParams(**values)
        self.target_label = self.get_parameter('target_label').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.images = {}
        self.depths = {}

        self.array_pub = self.create_publisher(
            ObjectObservationArray, 'object_observations', 10)
        self.debug_pub = None
        if self.get_parameter('publish_debug_image').value:
            self.debug_pub = self.create_publisher(Image, 'debug_image', qos_profile_sensor_data)
        self.create_subscription(
            Image, self.config.image_topic, self.on_image, qos_profile_sensor_data)
        if self.config.depth_topic:
            self.create_subscription(
                Image, self.config.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.get_logger().info(
            f'detecting {self.target_label} with {self.params} on {self.config}')

    def on_image(self, msg):
        if not self.config.depth_topic:
            self.process(msg, None)
            return
        self.store(self.images, msg)

    def on_depth(self, msg):
        self.store(self.depths, msg)

    def store(self, buffer, msg):
        stamp = stamp_to_ns(msg.header.stamp)
        buffer[stamp] = msg
        if len(buffer) > BUFFER_SIZE:
            del buffer[min(buffer)]
        if stamp in self.images and stamp in self.depths:
            self.process(self.images.pop(stamp), self.depths.pop(stamp))

    def process(self, color_msg, depth_msg):
        try:
            color = color_from_msg(color_msg)
            depth = depth_from_msg(depth_msg, self.depth_scale) if depth_msg else None
            blobs, floor = detect(color, depth, self.params)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=LOG_PERIOD_SEC)
            return
        self.array_pub.publish(make_object_array(
            self.config, color_msg.header, color_msg.width, color_msg.height, blobs,
            self.target_label))
        if self.debug_pub is not None:
            self.debug_pub.publish(image_msg(
                color_msg.header, draw_blobs(color, blobs, self.params.roi, floor)))
        summary = ' '.join(blob_summary(b) for b in blobs)
        self.get_logger().info(
            f'floor={floor:.3f}m n={len(blobs)} {summary}', throttle_duration_sec=LOG_PERIOD_SEC)


def main(args=None):
    # Ctrl+C raises KeyboardInterrupt only; shutdown happens once below
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = ObjectDetectorNode()
    except (ValueError, TypeError) as error:
        get_logger('object_detector').error(str(error))
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
