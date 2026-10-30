import math
import threading
import warnings

from ffw_grasp.grasp_planner import (
    fit_basket, GraspParams, matrix_to_quat, merge_planes, merge_targets, pick_target,
    plan_grasp, quat_to_matrix, Target)
from ffw_grasp.overlay import draw_basket, draw_grasp
from ffw_vision.camera_config import declare_camera_parameters, load_camera_config
from ffw_vision.depth_stats import color_from_msg, depth_from_msg
from ffw_vision.input_check import stamp_to_ns
from ffw_vision.seg_detector import pixel_rays
from ffw_vision_msgs.msg import ObjectObservation, ObjectObservationArray
from geometry_msgs.msg import PoseArray, PoseStamped
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import Trigger
import tf2_ros

BUFFER_SIZE = 30
TF_TIMEOUT = Duration(seconds=0.5)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SPIN_TIMEOUT_SEC = 0.1


def image_msg(header, bgr):
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    msg.data = bgr.tobytes()
    return msg


def declare_dataclass(node, defaults):
    """Declare one parameter per field and build the dataclass from their values."""
    for name, value in vars(defaults).items():
        node.declare_parameter(name, value)
    return type(defaults)(**{name: node.get_parameter(name).value for name in vars(defaults)})


def target_from_msg(o):
    return Target(o.u, o.v, o.length, o.width, o.angle, o.angle_period, o.pose_label,
                  o.top_height, float(o.score) if o.has_score else 0.0)


def transform_parts(tf):
    q, t = tf.transform.rotation, tf.transform.translation
    return quat_to_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])


def pose_msg(frame_id, stamp, position, rotation):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
    o = msg.pose.orientation
    o.x, o.y, o.z, o.w = matrix_to_quat(rotation)
    return msg


def plane_rotation(z, x):
    """Columns x, y, z with z up the plane normal and x along x (both base frame)."""
    x = x - (x @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def basket_summary(basket, reason, params):
    if basket is None:
        return reason
    cut = f' cut {basket.cut}' if basket.cut else ''
    return (f'basket rim {basket.length_m * 1000:.0f}x{basket.width_m * 1000:.0f}mm'
            f' (known {params.basket_length * 1000:.0f}x{params.basket_width * 1000:.0f}){cut}')


def grasp_summary(g):
    top = (f'{g.target.pose_label} top {g.length_m * 1000:.0f}x{g.width_m * 1000:.0f}mm'
           f' h {g.target.top_height * 1000:.0f}mm')
    if g.chosen is None:
        return f'{top}: {g.reason}'
    x, y, z = g.position
    return (f'{top} {g.chosen.name} open {g.chosen.opening * 1000:.0f}mm'
            f' tilt {math.degrees(g.chosen.tilt):.0f}deg'
            f' grasp ({x:.3f}, {y:.3f}, {z:.3f}) tip z {g.tip[2]:.3f} hover z {g.hover[2]:.3f}'
            f' wrist turn {math.degrees(g.wrist_turn):+.0f}deg table z {g.table_z:.3f}')


def bias_summary(p):
    return (f'tip below ee {p.tip_below_end_effector * 1000:.0f}mm, camera bias x/y/z'
            f' {p.camera_x_bias * 1000:+.0f}/{p.camera_y_bias * 1000:+.0f}/'
            f'{p.camera_z_bias * 1000:+.0f}mm, arm z bias {p.arm_z_bias * 1000:+.0f}mm')


def overlay_text(g):
    if g.chosen is None:
        return g.reason
    return (f'{g.target.pose_label} {g.chosen.name} open {g.chosen.opening * 1000:.0f}mm'
            f' tilt {math.degrees(g.chosen.tilt):.0f} turn {math.degrees(g.wrist_turn):+.0f}deg')


class GraspPlannerNode(Node):
    """On plan: object_observations + aligned depth + TF -> grasp and hover poses (no motion)."""

    def __init__(self):
        super().__init__('grasp_planner')
        declare_camera_parameters(self)
        self.declare_parameter('observation_topic', '')  # empty = ffw_vision's for camera_id
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'end_effector_r_link')
        self.declare_parameter('depth_scale', 0.001)  # meters per 16UC1 unit
        self.declare_parameter('frames', 10)  # observations merged per plan
        self.declare_parameter('min_frames', 7)
        self.declare_parameter('timeout_sec', 5.0)
        self.declare_parameter('max_center_jitter_px', 3.0)
        self.declare_parameter('max_camera_motion', 0.002)  # m between first and last frame
        self.declare_parameter('max_camera_turn_deg', 0.5)
        self.config = load_camera_config(self)
        self.params = declare_dataclass(self, GraspParams())
        observation_topic = (self.get_parameter('observation_topic').value
                             or f'/ffw_vision/{self.config.camera_id}/object_observations')
        if not self.config.depth_topic or not self.config.camera_info_topic:
            raise ValueError('grasp_planner needs depth_topic and camera_info_topic')
        names = ('base_frame', 'end_effector_frame', 'depth_scale', 'frames', 'min_frames',
                 'timeout_sec', 'max_center_jitter_px', 'max_camera_motion',
                 'max_camera_turn_deg')
        for name in names:
            setattr(self, name, self.get_parameter(name).value)

        self.lock = threading.Lock()
        self.done = threading.Event()
        self.collecting = False
        self.frames_seen = []  # (observations, depth, color or None)
        self.depths = {}
        self.colors = {}
        self.camera_info = None
        self.rays = None  # (key, rays) cached per CameraInfo
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.grasp_pub = self.create_publisher(PoseStamped, 'grasp_pose', LATCHED)
        self.hover_pub = self.create_publisher(PoseStamped, 'hover_pose', LATCHED)
        self.opening_pub = self.create_publisher(Float64, 'gripper_opening', LATCHED)
        self.debug_pub = self.create_publisher(Image, 'debug_image', LATCHED)
        self.basket_pub = self.create_publisher(PoseStamped, 'basket_pose', LATCHED)
        self.table_pub = self.create_publisher(PoseStamped, 'table_pose', LATCHED)
        # every usable candidate x free tilt, chosen first; info rows: opening, tilt, 0=A 1=B, turn
        self.options_pub = self.create_publisher(PoseArray, 'grasp_candidates', LATCHED)
        self.info_pub = self.create_publisher(Float64MultiArray, 'grasp_candidate_info', LATCHED)
        inputs = ReentrantCallbackGroup()
        self.create_subscription(
            ObjectObservationArray, observation_topic, self.on_observations, 10,
            callback_group=inputs)
        self.create_subscription(
            Image, self.config.depth_topic, self.on_depth, qos_profile_sensor_data,
            callback_group=inputs)
        self.create_subscription(
            Image, self.config.image_topic, self.on_color, qos_profile_sensor_data,
            callback_group=inputs)
        self.create_subscription(
            CameraInfo, self.config.camera_info_topic, self.on_camera_info,
            qos_profile_sensor_data, callback_group=inputs)
        self.create_service(Trigger, 'plan', self.on_plan,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(
            f'planning from {observation_topic} into {self.base_frame} for '
            f'{self.end_effector_frame} with {self.params} on {self.config}')

    def on_camera_info(self, msg):
        self.camera_info = msg

    def on_depth(self, msg):
        self.store(self.depths, msg)

    def on_color(self, msg):
        self.store(self.colors, msg)

    def store(self, buffer, msg):
        with self.lock:
            buffer[stamp_to_ns(msg.header.stamp)] = msg
            if len(buffer) > BUFFER_SIZE:
                del buffer[min(buffer)]

    def on_observations(self, msg):
        stamp = stamp_to_ns(msg.header.stamp)
        with self.lock:
            depth = self.depths.pop(stamp, None)
            color = self.colors.pop(stamp, None)
            if not self.collecting or depth is None:
                return
            self.frames_seen.append((msg, depth, color))
            if len(self.frames_seen) >= self.frames:
                self.collecting = False
                self.done.set()

    def rays_for(self, info):
        key = (info.width, info.height, tuple(info.k), tuple(info.d))
        if self.rays is None or self.rays[0] != key:
            self.rays = (key, pixel_rays(info.width, info.height, info.k, info.d))
        return self.rays[1]

    def lookup(self, target, source, stamp):
        return transform_parts(self.tf_buffer.lookup_transform(target, source, stamp, TF_TIMEOUT))

    def on_plan(self, request, response):
        with self.lock:
            self.frames_seen = []
            self.done.clear()
            self.collecting = True
        self.done.wait(self.timeout_sec)
        with self.lock:
            self.collecting = False
            frames = self.frames_seen
        try:
            response.message = self.plan(frames)
            response.success = True
            self.get_logger().info(response.message)
        except (ValueError, tf2_ros.TransformException) as error:
            response.success = False
            response.message = f'{len(frames)} frames: {error}'
            self.get_logger().warn(response.message)
        return response

    def plan(self, frames):
        info = self.camera_info
        if info is None:
            raise ValueError('no camera_info yet')
        targets, planes, used = [], [], []
        for frame in frames:
            obs = frame[0]
            valid = [target_from_msg(o) for o in obs.objects
                     if o.status == ObjectObservation.STATUS_VALID]
            target = pick_target(valid)
            if target is not None and obs.has_support_plane:
                targets.append(target)
                planes.append(obs.support_plane)
                used.append(frame)
        target, kept = merge_targets(targets, self.max_center_jitter_px, self.min_frames)
        plane = merge_planes([planes[i] for i in kept])
        kept_frames = [used[i] for i in kept]
        first, last = kept_frames[0][0].header, kept_frames[-1][0].header
        size = (kept_frames[-1][0].image_width, kept_frames[-1][0].image_height)
        if (info.width, info.height) != size:
            raise ValueError(f'camera_info {info.width}x{info.height} does not match image')

        # camera must not move while the frames are merged
        r0, t0 = self.lookup(self.base_frame, first.frame_id, Time.from_msg(first.stamp))
        rotation, translation = self.lookup(self.base_frame, last.frame_id,
                                            Time.from_msg(last.stamp))
        moved = np.linalg.norm(translation - t0)
        turned = math.degrees(math.acos(np.clip((np.trace(r0.T @ rotation) - 1) / 2, -1, 1)))
        if moved > self.max_camera_motion or turned > self.max_camera_turn_deg:
            raise ValueError(f'camera moved {moved * 1000:.1f}mm {turned:.1f}deg while observing')
        ee_rotation, _ = self.lookup(self.base_frame, self.end_effector_frame, Time())

        depths = np.stack([depth_from_msg(f[1], self.depth_scale) for f in kept_frames])
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)  # all-NaN pixels stay NaN
            depth = np.nanmedian(depths, axis=0)
        grasp = plan_grasp(target, plane, depth, info.k, info.d, self.rays_for(info),
                           rotation, translation, ee_rotation[:, 1], self.params,
                           [targets[i] for i in kept])
        color = kept_frames[-1][2]
        image = color_from_msg(color) if color is not None else None
        basket, basket_reason = (fit_basket(image, depth, info.k, info.d, self.rays_for(info),
                                            plane, self.params)
                                 if image is not None else (None, 'no color image'))
        self.publish_scene(grasp, basket, plane, rotation, translation, last.stamp)
        summary = (f'{grasp_summary(grasp)} frames {len(kept)}/{len(frames)};'
                   f' {basket_summary(basket, basket_reason, self.params)}'
                   f' ({bias_summary(self.params)})')
        if image is not None:
            out = draw_grasp(image, grasp, info.k, info.d, self.params, overlay_text(grasp))
            if basket is not None:
                out = draw_basket(out, basket, plane[0], info.k, info.d, self.params)
            self.debug_pub.publish(image_msg(color.header, out))
        if grasp.chosen is None:
            raise ValueError(summary)
        self.grasp_pub.publish(pose_msg(self.base_frame, last.stamp, grasp.position,
                                        grasp.rotation))
        self.hover_pub.publish(pose_msg(self.base_frame, last.stamp, grasp.hover,
                                        grasp.rotation))
        self.opening_pub.publish(Float64(data=float(grasp.chosen.opening)))
        options = PoseArray()
        options.header.frame_id = self.base_frame
        options.header.stamp = last.stamp
        info = []
        for o in grasp.options:
            options.poses.append(pose_msg(self.base_frame, last.stamp, o.position,
                                          o.rotation).pose)
            info += [float(o.opening), float(o.tilt), 0.0 if o.name == 'A' else 1.0,
                     float(o.wrist_turn)]
        self.info_pub.publish(Float64MultiArray(data=info))
        self.options_pub.publish(options)
        return f'{summary}; {len(grasp.options)} options'

    def publish_scene(self, grasp, basket, plane, rotation, translation, stamp):
        """Table under the object and the basket bottom, true base frame (camera bias removed)."""
        p = self.params
        bias = np.array([p.camera_x_bias, p.camera_y_bias, p.camera_z_bias])
        up = rotation @ plane[0]
        table = rotation @ (grasp.center - plane[0] * grasp.target.top_height) + translation
        self.table_pub.publish(pose_msg(self.base_frame, stamp, table - bias,
                                        plane_rotation(up, np.array([1.0, 0.0, 0.0]))))
        if basket is not None:
            self.basket_pub.publish(pose_msg(
                self.base_frame, stamp, rotation @ basket.center + translation - bias,
                plane_rotation(up, rotation @ basket.axis)))


def main(args=None):
    # Ctrl+C raises KeyboardInterrupt only; shutdown happens once below
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = GraspPlannerNode()
    except (ValueError, TypeError) as error:
        get_logger('grasp_planner').error(str(error))
        rclpy.try_shutdown()
        return 1
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        # wait with a timeout: without signal handlers Ctrl+C is seen only between waits
        while rclpy.ok():
            executor.spin_once(timeout_sec=SPIN_TIMEOUT_SEC)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
