import threading
import time
import math

from builtin_interfaces.msg import Duration
from robotis_interfaces.msg import MoveL
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from stall import StallDetector
from transforms import quat_angle, rpy_to_quat
from collections import deque

import cv2
from cv_bridge import CvBridge

from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_seconds(stamp):
    """builtin_interfaces/Time as float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class RobotIO(Node):
    """Background observation of cameras, joints, TF and simulation clock."""

    def __init__(self, cfg):
        super().__init__('poc_codex_io')
        self.cfg = cfg
        io_cfg = cfg['io']
        self.cameras = dict(cfg['cameras'])
        self.jpeg_quality = int(cfg['run']['jpeg_quality'])
        self.ready_timeout_s = float(io_cfg['ready_timeout_s'])
        self.lookup_timeout_s = float(io_cfg['lookup_timeout_s'])
        self.image_timeout_s = float(io_cfg['image_timeout_s'])
        self.snapshot_tolerance_s = float(io_cfg['snapshot_tolerance_s'])
        self._image_history = int(io_cfg['image_history'])
        self._joint_history = int(io_cfg['joint_history'])

        self._cache_lock = threading.Lock()
        self._joint_cache = {}
        self._image_cache = {}
        self._sim_clock = None   # sim_s, received, advanced, rewinds

        # Only the calling thread converts images, so one bridge is enough.
        self._bridge = CvBridge()

        # spin_thread=False: the TF subscriptions live on this node and are
        # served by the same executor as everything else.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # BEST_EFFORT subscriptions connect to both RELIABLE (Gazebo bridge)
        # and BEST_EFFORT (real camera drivers) publishers.
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, qos_profile_sensor_data)
        self.create_subscription(
            Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        for camera, topic in self.cameras.items():
            self.create_subscription(
                Image, topic, self._image_callback(camera), qos_profile_sensor_data)

        self.pose_pub = {
            arm: self.create_publisher(MoveL, topic, 10)
            for arm, topic in cfg['cyclo']['goal_topic'].items()
        }
        self.gripper_pub = {
            arm: self.create_publisher(JointTrajectory, topic, 10)
            for arm, topic in cfg['cyclo']['gripper_topic'].items()
        }

        self.body_groups = {
            'head': ('head_joint1', 'head_joint2'),
            'lift': ('lift_joint',),
        }
        self.body_units = {
            'head_joint1': 'rad',
            'head_joint2': 'rad',
            'lift_joint': 'm',
        }

        limit_keys = {
            'head_joint1': 'head_joint1_rad',
            'head_joint2': 'head_joint2_rad',
            'lift_joint': 'lift_joint_m',
        }

        self.body_limits = {}
        for joint, key in limit_keys.items():
            lower, upper = map(float, cfg['limits'][key])
            if (
                not math.isfinite(lower)
                or not math.isfinite(upper)
                or lower > upper
            ):
                raise ValueError(f'invalid limits. {key}')
            self.body_limits[joint] = (lower, upper)

        # Optional configuration. These are initial tuning values
        motion_cfg = cfg.get('body_motion', {})
        self.body_move_time_s = float(motion_cfg.get('move_time_s', 3.0))

        tolerance = cfg['tolerance']
        self.body_tolerances = {
            'head_joint1': float(tolerance.get('head_joint1_rad', 0.01)),
            'head_joint2': float(tolerance.get('head_joint2_rad', 0.01)),
            'lift_joint': float(tolerance.get('lift_joint_m', 0.005)),
        }

        for value in (self.body_move_time_s, *self.body_tolerances.values()):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    'body motion duration and tolerances must be positive'
                )

        self.body_pub = {
            'head': self.create_publisher(
                JointTrajectory,
                motion_cfg.get(
                    'head_topic',
                    '/leader/joystick_controller_left/joint_trajectory',
                ),
                10,
            ),
            'lift': self.create_publisher(
                JointTrajectory,
                motion_cfg.get(
                    'lift_topic',
                    '/leader/joystick_controller_right/joint_trajectory',
                ),
                10,
            )
        }

        self._io_executor = SingleThreadedExecutor()
        self._io_executor.add_node(self)
        self._spin_thread = None
        self._spin_error = None
        self._closing = False
        self._closed = False

    # ---------------------------------------------------------------
    # callbacks, run on the spin thread
    # ---------------------------------------------------------------

    def _on_joint_state(self, msg):
        received = time.monotonic()
        stamp_s = stamp_seconds(msg.header.stamp)
        with self._cache_lock:
            for name, position in zip(msg.name, msg.position):
                history = self._joint_cache.get(name)
                if history is None:
                    history = self._joint_cache[name] = deque(maxlen=self._joint_history)
                history.append({'position': position, 'stamp_s': stamp_s, 'received': received})

    def _image_callback(self, camera):
        def on_image(msg):
            # Keep the raw message. Converting to JPEG here would block the
            # spin thread for every frame of every camera.
            entry = {'msg': msg, 'stamp_s': stamp_seconds(msg.header.stamp),
                     'received': time.monotonic()}
            with self._cache_lock:
                history = self._image_cache.get(camera)
                if history is None:
                    history = self._image_cache[camera] = deque(maxlen=self._image_history)
                history.append(entry)
        return on_image

    def _on_clock(self, msg):
        now = time.monotonic()
        sim_s = stamp_seconds(msg.clock)
        with self._cache_lock:
            clock = self._sim_clock
            if clock is None:
                self._sim_clock = {'sim_s': sim_s, 'received': now, 'advanced': now, 'rewinds': 0}
                return
            if sim_s > clock['sim_s']:
                clock['advanced'] = now
            elif sim_s < clock['sim_s']:
                # Gazebo was reset or restarted. Count it so callers can tell,
                # and drop history from the old run so a snapshot cannot match
                # frames or transforms whose stamps belong to it.
                clock['rewinds'] += 1
                clock['advanced'] = now
                self._image_cache.clear()
                self._joint_cache.clear()
                self.tf_buffer.clear()
            clock['sim_s'] = sim_s
            clock['received'] = now
    # ---------------------------------------------------------------
    # thread lifecycle
    # ---------------------------------------------------------------

    def _spin(self):
        try:
            self._io_executor.spin()
        except ExternalShutdownException:
            pass  # rclpy.shutdown() while spinning, for example Ctrl+C
        except Exception as exc:
            if not self._closing:
                self._spin_error = exc

    def start(self):
        """Start the background spin thread. Call once, after rclpy.init()."""
        if self._closing:
            raise RuntimeError('RobotIO is closing or already closed')
        if self._spin_thread is not None:
            raise RuntimeError('RobotIO.start() was already called')
        self._spin_thread = threading.Thread(
            target=self._spin, name='robot_io_spin', daemon=True)
        self._spin_thread.start()

    def _check_running(self):
        if self._closing:
            raise RuntimeError('RobotIO is closing or already closed')
        if self._spin_error is not None:
            raise RuntimeError(f'ROS spin thread failed: {self._spin_error!r}') from self._spin_error
        if self._spin_thread is None:
            raise RuntimeError('RobotIO.start() has not been called')
        if not self._spin_thread.is_alive():
            raise RuntimeError('ROS spin thread stopped (rclpy shut down?)')

    def close(self):
        """Stop callbacks before destroying the node; failed shutdown can be retried."""
        if self._closed:
            return
        self._closing = True
        stopped = self._io_executor.shutdown(timeout_sec=2.0)
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            if self._spin_thread.is_alive():
                raise RuntimeError('RobotIO spin thread did not stop; node was not destroyed')
        if not stopped:
            raise RuntimeError('RobotIO executor shutdown timed out; node was not destroyed')
        self.destroy_node()
        self._closed = True

    # ---------------------------------------------------------------
    # observation
    # ---------------------------------------------------------------

    def _missing_observations(self):
        missing = []
        required_joints = (
            tuple(self.cfg['cyclo']['gripper_joint'].values())
            + tuple(self.body_units)
        )

        with self._cache_lock:
            missing += [
                f'joint {joint}'
                for joint in required_joints
                if not self._joint_cache.get(joint)
            ]
            if self._sim_clock is None:
                missing.append('/clock')
        base = self.cfg['frames']['base']
        for ee in self.cfg['frames']['ee'].values():
            if not self.tf_buffer.can_transform(base, ee, Time()):
                missing.append(f'tf {base} -> {ee}')
        return missing

    def wait_ready(self, timeout_s=None):
        """Block until every camera, gripper joint, /clock and both hand transforms arrived once."""
        if timeout_s is None:
            timeout_s = self.ready_timeout_s
        deadline = time.monotonic() + timeout_s
        while True:
            self._check_running()
            missing = self._missing_observations()
            if not missing:
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f'observations not ready after {timeout_s:.1f} s, missing: {", ".join(missing)}')
            time.sleep(0.1)

    def get_ee_pose(self, arm, timeout_s=None):
        """Latest measured base_link -> end effector transform.

        Real SG2 coordinates, meters, quaternion (x, y, z, w). No Cyclo offset.
        stamp_s is the simulation time of the transform.
        """
        if timeout_s is None:
            timeout_s = self.lookup_timeout_s
        base = self.cfg['frames']['base']
        ee = self.cfg['frames']['ee'][arm]
        deadline = time.monotonic() + timeout_s
        while True:
            self._check_running()
            try:
                # Time() requests the latest available transform. Explicit TF
                # times, if needed, must come from simulation-stamped data.
                tf = self.tf_buffer.lookup_transform(base, ee, Time())
            except TransformException as exc:
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f'no transform {base} -> {ee} within {timeout_s:.1f} s ({exc})') from exc
                time.sleep(0.05)
                continue
            t, r = tf.transform.translation, tf.transform.rotation
            return {'frame': base, 'x': t.x, 'y': t.y, 'z': t.z,
                    'quat': [r.x, r.y, r.z, r.w],
                    'stamp_s': stamp_seconds(tf.header.stamp)}

    def get_arm_base_z(self, timeout_s=None):
        """Height of the lift-carried arm base in base_link, in meters
        
        The lift is a prismatic z joint whose child link carries both arms,
        so reach limits measured from this link stay valid at any lift height
        """
        if timeout_s is None:
            timeout_s = self.lookup_timeout_s
            
        base = self.cfg['frames']['base']
        arm_base = self.cfg['reach']['frame']
        deadline = time.monotonic() + timeout_s
        while True:
            self._check_running()
            try:
                tf = self.tf_buffer.lookup_transform(base, arm_base, Time())
            except TransformException as exc:
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f'no transform {base} -> {arm_base} within '
                        f'{timeout_s:.1f} s ({exc})') from exc
                time.sleep(0.05)
                continue
            return {'frame': base, 'link': arm_base,
                    'z': tf.transform.translation.z,
                    'stamp_s': stamp_seconds(tf.header.stamp)}

    def get_gripper(self, arm, timeout_s=None):
        """Gripper joint position in rad from /joint_states, with its stamp and age."""
        if timeout_s is None:
            timeout_s = self.lookup_timeout_s
        joint = self.cfg['cyclo']['gripper_joint'][arm]
        deadline = time.monotonic() + timeout_s
        while True:
            self._check_running()
            with self._cache_lock:
                history = self._joint_cache.get(joint)
                entry = history[-1] if history else None
            if entry is not None:
                return {'joint': joint, 'position_rad': entry['position'],
                        'stamp_s': entry['stamp_s'],
                        'age_s': time.monotonic() - entry['received']}
            if time.monotonic() >= deadline:
                raise RuntimeError(f'no /joint_states value for {joint} within {timeout_s:.1f} s')
            time.sleep(0.05)

    def get_body_state(self, group=None, timeout_s=None, after_received=None):
        """Read recent head/lift samples from the joint-state cache"""
        if timeout_s is None:
            timeout_s = self.lookup_timeout_s

        names = (
            tuple(self.body_units)
            if group is None
            else self.body_groups[group]
        )

        deadline = time.monotonic() + float(timeout_s)

        while True:
            self._check_running()
            now = time.monotonic()

            with self._cache_lock:
                samples = {}
                for joint in names:
                    history = self._joint_cache.get(joint)
                    if history:
                        samples[joint] = dict(history[-1])

            ready = all(
                joint in samples
                and math.isfinite(samples[joint]['position'])
                and now - samples[joint]['received'] <= self.lookup_timeout_s
                and (
                    after_received is None
                    or samples[joint]['received'] > after_received
                )
                for joint in names
            )


            if ready:
                return {
                    joint: {
                        'position': float(samples[joint]['position']),
                        'unit': self.body_units[joint],
                        'stamp_s': float(samples[joint]['stamp_s']),
                        'age_s': now - samples[joint]['received'],
                    }
                    for joint in names
                }

            if now >= deadline:
                raise TimeoutError(
                    f'no recent joint-state samples for {names}'
                )

            time.sleep(min(0.02, max(0.0, deadline - now)))

    def get_sim_clock(self):
        """Latest /clock and how long ago it was received and last moved forward."""
        self._check_running()
        with self._cache_lock:
            clock = None if self._sim_clock is None else dict(self._sim_clock)
        now = time.monotonic()
        if clock is None:
            return {'sim_s': None, 'received_age_s': None, 'advanced_age_s': None, 'rewinds': 0}
        return {'sim_s': clock['sim_s'],
                'received_age_s': now - clock['received'],
                'advanced_age_s': now - clock['advanced'],
                'rewinds': clock['rewinds']}

    def get_state(self):
        """Latest value of each source for both arms, plus the simulation clock. Raw units.

        Sources are sampled independently and retain their own timestamps.
        Camera data is queried separately with get_snapshot().
        """
        arms = {}
        for arm in self.cfg['frames']['ee']:
            arms[arm] = {'ee': self.get_ee_pose(arm), 'gripper': self.get_gripper(arm)}
        return {
            'frame': self.cfg['frames']['base'],
            'arms': arms,
            'body': self.get_body_state(),
            'clock': self.get_sim_clock(),
        }

    @staticmethod
    def _nearest(history, stamp_s):
        """Entry of history whose stamp is closest to stamp_s, or None if empty."""
        return min(history, key=lambda entry: abs(entry['stamp_s'] - stamp_s), default=None)

    def _state_at(self, stamp_s, joints):
        """Arm poses from TF and gripper samples at one simulation time.

        Returns (problem, arms). problem is None when every value was found.
        """
        base = self.cfg['frames']['base']
        tol = self.snapshot_tolerance_s
        arms = {}
        for arm, ee in self.cfg['frames']['ee'].items():
            try:
                # TF keeps sim-stamped samples and interpolates between them.
                # A stamp newer than the last /tf raises, and the caller retries.
                tf = self.tf_buffer.lookup_transform(base, ee, Time(seconds=stamp_s))
            except TransformException as exc:
                return f'tf {base} -> {ee} at {stamp_s:.3f} not available yet ({type(exc).__name__})', None
            joint = self.cfg['cyclo']['gripper_joint'][arm]
            sample = self._nearest(joints[arm], stamp_s)
            if sample is None or abs(sample['stamp_s'] - stamp_s) > tol:
                return f'no {joint} sample within {tol} s of {stamp_s:.3f}', None
            t, r = tf.transform.translation, tf.transform.rotation
            arms[arm] = {
                'ee': {'frame': base, 'x': t.x, 'y': t.y, 'z': t.z,
                       'quat': [r.x, r.y, r.z, r.w], 'stamp_s': stamp_s},
                'gripper': {'joint': joint, 'position_rad': sample['position'],
                            'stamp_s': sample['stamp_s']},
            }
        return None, arms

    def _try_snapshot(self, cameras, after_stamp_s):
        """One attempt at matching frames and state. Returns (problem, stamp_s, frames, arms)."""
        tol = self.snapshot_tolerance_s
        with self._cache_lock:
            frames = {c: list(self._image_cache.get(c, ())) for c in cameras}
            joints = {arm: list(self._joint_cache.get(joint, ()))
                      for arm, joint in self.cfg['cyclo']['gripper_joint'].items()}
            body_histories = {
                joint: list(self._joint_cache.get(joint, ()))
                for joint in self.body_units
            }

        waiting = [c for c in cameras if not frames[c]]
        if waiting:
            return f'no frame yet from {", ".join(waiting)}', None, None, None

        # Gazebo renders all cameras on the same simulation steps, so their
        # stamps match exactly or differ by a whole frame. Walk the times newest
        # first and return the first one with a frame from every camera whose TF
        # and joint samples have already arrived. If TF lags the cameras by more
        # than a frame, the newest time is never complete when checked, so
        # waiting only for it would time out every call.
        newest_problem = None
        for stamp_s in sorted({f['stamp_s'] for f in frames[cameras[0]]}, reverse=True):
            if after_stamp_s is not None and stamp_s <= after_stamp_s:
                break
            chosen = {}
            for camera in cameras:
                frame = self._nearest(frames[camera], stamp_s)
                if abs(frame['stamp_s'] - stamp_s) > tol:
                    break
                chosen[camera] = frame
            else:
                problem, arms = self._state_at(stamp_s, joints)

                if problem is None:
                    problem, body = self._body_state_at(
                        stamp_s,
                        body_histories,
                    )

                if problem is None:
                    return None, stamp_s, chosen, {
                        'arms': arms,
                        'body': body,
                    }

                if newest_problem is None:
                    newest_problem = problem
                    
            return newest_problem, None, None, None
        after = '' if after_stamp_s is None else f' newer than {after_stamp_s:.3f}'
        return f'no time{after} with a frame from every camera', None, None, None

    def get_snapshot(self, cameras, after_stamp_s=None, timeout_s=None):
        """Camera frames and both arms' state that belong to one simulation time.

        The frames pick the time: the newest stamp every listed camera has a frame
        for (within snapshot_tolerance_s). Arm poses are TF at that stamp, gripper
        values the /joint_states samples nearest to it.
        after_stamp_s: accept only a time later than this, for example the moment
        a move settled, so the snapshot cannot show the robot before it finished.
        """
        cameras = list(cameras)
        unknown = [c for c in cameras if c not in self.cameras]
        if not cameras or unknown:
            raise KeyError(f'cameras must be some of {sorted(self.cameras)}, got {cameras}')
        if timeout_s is None:
            timeout_s = self.image_timeout_s
        deadline = time.monotonic() + timeout_s
        while True:
            self._check_running()
            problem, stamp_s, frames, state = self._try_snapshot(
                cameras,
                after_stamp_s,
            )
            if problem is None:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(f'no consistent snapshot within {timeout_s:.1f} s: {problem}')
            time.sleep(0.02)

        arms = state['arms']
        body = state['body']

        # JPEG encoding happens after matching, outside the lock and off the spin thread.
        now = time.monotonic()
        images = {camera: self._encode_frame(camera, frame, now) for camera, frame in frames.items()}
        offsets = [
            abs(image['stamp_s'] - stamp_s)
            for image in images.values()
        ]
        offsets += [
            abs(arm['gripper']['stamp_s'] - stamp_s)
            for arm in arms.values()
        ]
        offsets += [
            abs(joint['stamp_s'] - stamp_s)
            for joint in body.values()
        ]

        return {'stamp_s': stamp_s, 'frame': self.cfg['frames']['base'],
                'arms': arms, 'body': body, 'images': images,
                'max_offset_s': max(offsets), 'clock': self.get_sim_clock()}

    def _encode_frame(self, camera, frame, now):
        msg = frame['msg']
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            raise RuntimeError(f'{camera}: JPEG encoding failed')
        return {'camera': camera, 'jpeg': buffer.tobytes(),
                'topic': self.cameras[camera], 'frame_id': msg.header.frame_id,
                'stamp_s': frame['stamp_s'],
                'receive_age_s': now - frame['received'],
                'width': msg.width, 'height': msg.height, 'encoding': msg.encoding}

    # ---------------------------------------------------------------
    # Cyclo command publication
    # ---------------------------------------------------------------

    def _wait_for_subscriber(self, publisher, timeout_s=5.0):
        """Wait in wall time until Cyclo subscribes to the publisher"""
        deadline = time.monotonic() + timeout_s

        while True:
            self._check_running()

            if publisher.get_subscription_count() > 0:
                return True
            if time.monotonic() >= deadline:
                return False

            time.sleep(0.05)

    @staticmethod
    def _duration(seconds):
        """Convert non-negative seconds to builtin_interfaces/Duration"""
        seconds = float(seconds)

        if not math.isfinite(seconds) or seconds < 0.0:
            raise ValueError(
                f'duration must be finite and non-negative, got {seconds!r}'
            )

        sec = int(seconds)
        nanosec = int(round((seconds - sec) * 1e9))

        # Protect against rounding 0.999999999... to one full second.
        if nanosec == 1_000_000_000:
            sec += 1
            nanosec = 0

        return Duration(sec=sec, nanosec=nanosec)

    def send_pose(self, arm, target, move_time_s=None):
        """Publish a MoveL goal expressed in real SG2 base_link coordinates"""
        if move_time_s is None:
            move_time_s = self.cfg['cyclo']['move_time_s']

        publisher = self.pose_pub[arm]

        if not self._wait_for_subscriber(publisher):
            raise RuntimeError(
                f'nothing subscribes to {publisher.topic_name}; '
                'is Cyclo running?'
            )

        msg = MoveL()
        msg.pose.header.frame_id = self.cfg['frames']['base']
        msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Apply the SG2 solver correction here and only here.
        msg.pose.pose.position.x = float(
            target['x'] + self.cfg['cyclo']['x_offset_m']
        )
        msg.pose.pose.position.y = float(target['y'])
        msg.pose.pose.position.z = float(target['z'])

        qx, qy, qz, qw = rpy_to_quat(
            target['roll'],
            target['pitch'],
            target['yaw'],
        )
        msg.pose.pose.orientation.x = float(qx)
        msg.pose.pose.orientation.y = float(qy)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)

        msg.time_from_start = self._duration(move_time_s)
        publisher.publish(msg)

        return {
            'quat': [qx, qy, qz, qw],
            'sent_x': msg.pose.pose.position.x,
        }

    def send_gripper(self, arm, value, move_time_s=1.0):
        """Publish a normalized 0-open/1-closed gripper command"""
        lo, hi = self.cfg['limits']['gripper_rad']
        target_rad = float(lo + value * (hi - lo))
        publisher = self.gripper_pub[arm]

        if not self._wait_for_subscriber(publisher):
            raise RuntimeError(
                f'nothing subscribes to {publisher.topic_name}; '
                'is Cyclo running?'
            )

        trajectory = JointTrajectory()
        trajectory.joint_names = [
            self.cfg['cyclo']['gripper_joint'][arm]
        ]

        point = JointTrajectoryPoint()
        point.positions = [target_rad]
        point.time_from_start = self._duration(move_time_s)
        trajectory.points = [point]

        publisher.publish(trajectory)
        return target_rad

    def send_body(self, group, targets):
        """Publish one absolute trajectory for the head or lift."""
        names = self.body_groups[group]

        if set(targets) != set(names):
            raise ValueError(
                f'{group} requires exactly these joints: {names}'
            )

        positions = []
        for joint in names:
            raw = targets[joint]
            if isinstance(raw, bool):
                raise ValueError(f'{joint} must be numeric')

            value = float(raw)
            lower, upper = self.body_limits[joint]

            if not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(
                    f'{joint}={value} is outside [{lower}, {upper}]'
                )

            positions.append(value)

        publisher = self.body_pub[group]
        if not self._wait_for_subscriber(publisher):
            raise RuntimeError(
                f'nothing subscribes to {publisher.topic_name}'
            )

        trajectory = JointTrajectory()
        trajectory.joint_names = list(names)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = self._duration(self.body_move_time_s)
        trajectory.points = [point]

        # Leave header.stamp at zero: start upon receipt by the controller.
        sent_at = time.monotonic()
        publisher.publish(trajectory)
        return sent_at

    def wait_until_arrived(self, arm, target, target_quat, timeout_s=None):
        """Wait until the measured hand pose comes to rest.

        At rest inside tolerance is arrived; at rest outside it is stopped
        (blocked, e.g. pressing on an object). Rest is judged only after the
        MoveL trajectory time, so a short move cannot count as arrived before
        it has moved, and a pose still converging is not reported early.
        """
        tolerance = self.cfg['tolerance']
        if timeout_s is None:
            timeout_s = float(tolerance['timeout_s'])
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError(
                f'timeout_s must be finite and non-negative, got {timeout_s!r}'
            )

        started = time.monotonic()
        deadline = started + timeout_s
        stall_s = float(tolerance['stall_s'])
        rest = StallDetector(
            stall_s,
            float(self.cfg['cyclo']['move_time_s']) + stall_s,
            started,
            lambda old, new: (
                math.dist(old[0], new[0]) > float(tolerance['stall_position_m'])
                or quat_angle(old[1], new[1]) > float(tolerance['stall_orientation_deg'])
            ),
        )

        while True:
            remaining = max(0.0, deadline - time.monotonic())

            pose = self.get_ee_pose(
                arm, timeout_s=min(self.lookup_timeout_s, remaining)
            )
            position_error = math.dist(
                (pose['x'], pose['y'], pose['z']),
                (target['x'], target['y'], target['z']),
            )
            orientation_error = quat_angle(
                pose['quat'],
                target_quat,
            )
            inside = (
                position_error <= float(tolerance['position_m'])
                and orientation_error <= float(tolerance['orientation_deg'])
            )

            now = time.monotonic()
            at_rest = rest.update(
                now,
                pose['stamp_s'],
                ((pose['x'], pose['y'], pose['z']), pose['quat']),
            )

            if at_rest or now >= deadline:
                return {
                    'arrived': at_rest and inside,
                    'stopped': at_rest and not inside,
                    'pose': pose,
                    'position_error_m': position_error,
                    'orientation_error_deg': orientation_error,
                    'elapsed_s': now - started,
                }
            time.sleep(min(0.05, max(0.0, deadline - now)))

    def wait_until_gripper_arrived(self, arm, target_rad, timeout_s=None, move_time_s=1.0):
        """Wait until the measured gripper joint comes to rest.

        At rest inside tolerance is arrived; at rest outside it is stopped
        (fingers closed on an object). Rest is judged only after move_time_s,
        the trajectory time given to send_gripper.
        """
        tolerance = self.cfg['tolerance']
        if timeout_s is None:
            timeout_s = float(tolerance['timeout_s'])
        timeout_s = float(timeout_s)
        target_rad = float(target_rad)

        if not math.isfinite(target_rad):
            raise ValueError(
                f'target_rad must be finite, got {target_rad!r}'
            )
        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError(
                f'timeout_s must be finite and non-negative, got {timeout_s!r}'
            )

        started = time.monotonic()
        deadline = started + timeout_s
        stall_s = float(tolerance['stall_s'])
        rest = StallDetector(
            stall_s,
            float(move_time_s) + stall_s,
            started,
            lambda old, new: abs(old - new) > float(tolerance['stall_gripper_rad']),
        )

        while True:
            remaining = max(0.0, deadline - time.monotonic())

            measured = self.get_gripper(
                arm,
                timeout_s=min(self.lookup_timeout_s, remaining),
            )

            error_rad = abs(
                measured['position_rad'] - target_rad
            )
            inside = error_rad <= tolerance['gripper_rad']
            now = time.monotonic()
            at_rest = rest.update(now, measured['stamp_s'], measured['position_rad'])

            if at_rest or now >= deadline:
                return {
                    'arrived': at_rest and inside,
                    'stopped': at_rest and not inside,
                    'gripper': measured,
                    'error_rad': error_rad,
                    'elapsed_s': now - started,
                }

            time.sleep(min(0.05, max(0.0, deadline - now)))

    def wait_until_body_arrived(
        self,
        group,
        targets,
        after_received,
        timeout_s=None,
        settle_s=None,
    ):
        """Wait for fresh measured positions to settle inside tolerance"""
        tolerance = self.cfg['tolerance']

        if timeout_s is None:
            timeout_s = float(tolerance['timeout_s'])
        if settle_s is None:
            settle_s = float(tolerance['settle_s'])

        timeout_s = float(timeout_s)
        settle_s = float(settle_s)

        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError('timeout_s must be finite and positive')
        if not math.isfinite(settle_s) or settle_s < 0.0:
            raise ValueError('settle_s must be finite and non-negative')

        started = time.monotonic()
        deadline = started + timeout_s
        inside_since = None
        previous_stamps = None
        rewinds = self.get_sim_clock()['rewinds']

        while True:
            now = time.monotonic()
            remaining = max(0.0, deadline - now)

            measured = self.get_body_state(
                group=group,
                timeout_s=min(self.lookup_timeout_s, remaining),
                after_received=after_received,
            )

            if self.get_sim_clock()['rewinds'] != rewinds:
                raise RuntimeError('simulation clock reset during movement')

            errors = {
                joint: abs(measured[joint]['position'] - targets[joint])
                for joint in self.body_groups[group]
            }

            inside = all(
                errors[joint] <= self.body_tolerances[joint]
                for joint in errors
            )

            stamps = {
                joint: measured[joint]['stamp_s']
                for joint in errors
            }

            advanced = (
                previous_stamps is None
                or all(
                    stamps[joint] > previous_stamps[joint]
                    for joint in stamps
                )
            )

            now = time.monotonic()
            arrived = False

            if not inside:
                inside_since = None

            if advanced:
                previous_stamps = stamps

                if inside:
                    if inside_since is None:
                        inside_since = now
                    arrived = now - inside_since >= settle_s

            result = {
                'arrived': arrived,
                'joints': measured,
                'errors': errors,
                'elapsed_s': now - started,
            }

            if now >= deadline:
                result['arrived'] = False
                return result

            if arrived:
                return result

            time.sleep(min(0.02, max(0.0, deadline - now)))

    def _body_state_at(self, stamp_s, histories):
        """Match head/lift samples to the selected camera timestamp."""
        body = {}

        for joint, unit in self.body_units.items():
            sample = self._nearest(histories[joint], stamp_s)

            if (
                sample is None
                or not math.isfinite(sample['position'])
                or abs(sample['stamp_s'] - stamp_s)
                > self.snapshot_tolerance_s
            ):
                return (
                    f'no matching {joint} sample at {stamp_s:.3f}',
                    None,
                )

            body[joint] = {
                'position': float(sample['position']),
                'unit': unit,
                'stamp_s': float(sample['stamp_s']),
            }

        return None, body