"""
Validation and execution tools reached by the model through RobotRollout and ObservationGuard.
"""

import math

from transforms import quat_angle, rpy_to_quat, quat_to_rpy


POSE_KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
OBSERVATION_CAMERAS = ('head', 'wrist_left', 'wrist_right')

def _finite_float(args, key):
    """Read one required argument as a finite float

    Returns:
        (value, None) on success
        (None, reason) on failure
    """
    if key not in args:
        return None, f'missing required argument {key}'

    raw = args[key]

    # bool is technically convertible to float, but it is not a valid
    # coordinate or gripper command
    if isinstance(raw, bool):
        return None, f'{key} is not a number ({raw!r})'

    # Convert the required argument to float and catch invalid numeric input.
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None, f'{key} is not a number ({raw!r})'

    if not math.isfinite(value):
        return None, f'{key} is not finite'
    return value, None

def _arrival_status(arrival):
    """arrived, stopped (blocked short of the target and still) or timeout"""
    if arrival['arrived']:
        return 'arrived'
    return 'stopped' if arrival.get('stopped') else 'timeout'

def validate_move(args, cfg, current_pose, arm_base_z):
    """Validate one absolute E-E target against robot-only limits

    Returns:
        (True, 'ok', clean) when accepted
        (False, reason, None) when rejected
    """
    if not isinstance(args, dict):
        return False, f'args must be a dict, not {type(args).__name__}', None

    arm = args.get('arm')
    if arm not in cfg['frames']['ee']:
        return False, f'unknown arm {arm!r}', None

    clean = {'arm': arm}

    for key in POSE_KEYS:
        value, error = _finite_float(args, key)
        if error is not None:
            return False, error, None
        clean[key] = value

    # Reach is measured from the lift-carried arm base, so the bound moves
    # with the lift and says nothing about the environment.
    reach = cfg['reach']
    z_min = float(reach['z_min'])
    z_from_arm_base = clean['z'] - float(arm_base_z)

    if z_from_arm_base < z_min:
        return (
            False,
            (
                f'z {clean["z"]:.3f} is {-z_from_arm_base:.3f} m below '
                f'{reach["frame"]}; the arm reaches {-z_min:.3f} m below it'
            ),
            None,
        )

    clean['z_from_arm_base'] = z_from_arm_base

    step_m = math.dist(
        (current_pose['x'], current_pose['y'], current_pose['z']),
        (clean['x'], clean['y'], clean['z']),
    )
    max_step_m = float(cfg['limits']['max_step_m'])

    if step_m > max_step_m:
        return (False, f'step {step_m:.3f} m exceeds {max_step_m} m', None)

    clean['step_m'] = step_m

    # Rotation is limited against the current measured pose, not a stored
    # reference pose, so no prior orientation is handed to the model.
    target_quat = rpy_to_quat(clean['roll'], clean['pitch'], clean['yaw'])
    rotation_deg = quat_angle(target_quat, current_pose['quat'])
    max_rotation_deg = math.degrees(float(cfg['limits']['max_rotation_rad']))

    if rotation_deg > max_rotation_deg:
        return (
            False,
            (
                f'rotation {rotation_deg:.1f} deg from the current pose '
                f'exceeds {max_rotation_deg:.1f} deg'
            ),
            None,
        )

    clean['rotation_deg'] = rotation_deg

    return True, 'ok', clean


def validate_gripper(args, cfg):
    """Validate one normalized gripper command"""
    if not isinstance(args, dict):
        return False, f'arguments are not an object ({type(args).__name__})', None

    arm = args.get('arm')
    if arm not in cfg['frames']['ee']:
        return False, f'unknown arm {arm!r}', None

    value, error = _finite_float(args, 'value')
    if error is not None:
        return False, error, None

    if not 0.0 <= value <= 1.0:
        return False, f'value {value} outside [0, 1]', None

    return True, 'ok', {'arm': arm, 'value': value}


class RobotTools:
    """Model-facing tools built on top of RobotIO"""

    def __init__(self, io, cfg):
        self.io = io
        self.cfg = cfg

    def _model_arms(self, raw_arms):
        """Convert raw RobotIO arm state to model-facing units"""
        lower_rad, upper_rad = (
            float(value)
            for value in self.cfg['limits']['gripper_rad']
        )
        gripper_span = upper_rad - lower_rad

        if gripper_span <= 0.0:
            raise ValueError(
                'limits.gripper_rad upper bound must be greater '
                'than the lower bound'
            )

        arms = {}

        for arm, measured in raw_arms.items():
            pose = measured['ee']
            gripper = measured['gripper']

            roll, pitch, yaw = quat_to_rpy(pose['quat'])
            position_rad = float(gripper['position_rad'])
            normalized = (position_rad - lower_rad) / gripper_span

            gripper_data = {
                'joint': gripper['joint'],
                'value': normalized,
                'position_rad': position_rad,
                'stamp_s': float(gripper['stamp_s']),
            }

            # Latest-state observations provide age_s, while snapshots do not
            if 'age_s' in gripper:
                gripper_data['age_s'] = float(gripper['age_s'])

            arms[arm] = {
                'ee': {
                    'frame': pose['frame'],
                    'x': float(pose['x']),
                    'y': float(pose['y']),
                    'z': float(pose['z']),
                    'roll': float(roll),
                    'pitch': float(pitch),
                    'yaw': float(yaw),
                    'stamp_s': float(pose['stamp_s']),
                },
                'gripper': gripper_data,
            }
        return arms

    def get_state(self):
        """Return the latest robot state in model-facing units"""
        raw = self.io.get_state()

        return {
            'status': 'ok',
            'reason': 'state observed',
            'command_sent': False,
            'data': {
                'frame': raw['frame'],
                'arms': self._model_arms(raw['arms']),
                'clock': raw['clock'],
                'body': raw['body'],
            },
            'images': [],
        }

    @staticmethod
    def _image_metadata(image):
        """Return image metadata without JPEG bytes"""
        return {
            key: value
            for key, value in image.items()
            if key != 'jpeg'
        }

    def _snapshot_data(self, snapshot):
        """Return JSON-safe metadata and model-facing snapshot state"""
        return {
            'stamp_s': float(snapshot['stamp_s']),
            'frame': snapshot['frame'],
            'arms': self._model_arms(snapshot['arms']),
            'clock': snapshot['clock'],
            'body': snapshot['body'],
            'max_offset_s': float(snapshot['max_offset_s']),
            'cameras': {
                camera: self._image_metadata(image)
                for camera, image in snapshot['images'].items()
            },
        }

    def observe(self):
        """Return all cameras and both arms from one synchronized snapshot"""
        missing = [
            camera
            for camera in OBSERVATION_CAMERAS
            if camera not in self.cfg['cameras']
        ]

        if missing:
            raise KeyError(
                f'missing configured cameras: {", ".join(missing)}'
            )

        snapshot = self.io.get_snapshot(OBSERVATION_CAMERAS)
        images = [
            snapshot['images'][camera]
            for camera in OBSERVATION_CAMERAS
        ]

        return {
            'status': 'ok',
            'reason': 'synchronized observation captured',
            'command_sent': False,
            'data': self._snapshot_data(snapshot),
            'images': images,
        }

    def _capture_after(self, cameras, stamp_s):
        """Capture a synchronized observation after an action
        
        Returns:
            (snapshot, None) on success
            (None, reason) on failure
        """
        cameras = tuple(cameras)

        # Capture camera frames and robot state from one post-action moment
        try:
            snapshot = self.io.get_snapshot(
                cameras,
                after_stamp_s=stamp_s,
            )
        except Exception as exc:
            return None, f'{type(exc).__name__}: {exc}'

        return snapshot, None

    def move(self, arm, x, y, z, roll, pitch, yaw):
        """Validate, publish and wait for one absolute arm movement"""
        args = {
            'arm': arm,
            'x': x,
            'y': y,
            'z': z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
        }

        # Check the arm before using it to query RobotIO
        if arm not in self.cfg['frames']['ee']:
            return {
                'status': 'rejected',
                'reason': f'unknown arm {arm!r}',
                'command_sent': False,
                'data': {'requested': args},
                'images': [],
            }

        # Read the measured pose used for the maximum-step validation.
        try:
            current_pose = self.io.get_ee_pose(arm)
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed to read current pose: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {'requested': args},
                'images': [],
            }

        # Read the lift-carried arm base so the reach bound follows the lift
        try:
            arm_base = self.io.get_arm_base_z()
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed to read arm base height: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {'requested': args},
                'images': [],
            }

        # Validate the move and catch malformed configuration or state data.
        try:
            ok, reason, target = validate_move(args, self.cfg, current_pose, arm_base['z'])
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'move validation failed: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {'requested': args},
                'images': [],
            }

        if not ok:
            return {
                'status': 'rejected',
                'reason': reason,
                'command_sent': False,
                'data': {
                    'requested': args,
                    'current_pose': current_pose,
                },
                'images': [],
            }

        # Publish the MoveL target and catch subscriber or ROS publication failures.
        try:
            sent = self.io.send_pose(arm, target)
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed to publish move command: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {
                    'target': target,
                    'current_pose': current_pose,
                },
                'images': [],
            }

        # Wait for measured pose convergence and catch TF or observation failures.
        try:
            arrival = self.io.wait_until_arrived(arm, target, sent['quat'])
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed while waiting for movement: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': True,
                'data': {
                    'target': target,
                    'command': sent,
                },
                'images': [],
            }

        cameras = OBSERVATION_CAMERAS
        snapshot, observation_error = self._capture_after(
            cameras,
            arrival['pose']['stamp_s'],
        )

        status = _arrival_status(arrival)
        reason = {
            'arrived': 'move target reached',
            'stopped': 'move stopped before its target',
            'timeout': 'move did not settle before timeout',
        }[status]

        if observation_error is not None:
            reason += (
                '; post observation unavailable: '
                f'{observation_error}'
            )

        return {
            'status': status,
            'reason': reason,
            'command_sent': True,
            'data': {
                'target': target,
                'command': sent,
                'arrival': arrival,
                'post_observation': (
                    None
                    if snapshot is None
                    else self._snapshot_data(snapshot)
                ),
                'post_observation_error': observation_error,
            },
            'images': (
                []
                if snapshot is None
                else [
                    snapshot['images'][camera]
                    for camera in cameras
                ]
            ),
        }
    
    def set_gripper(self, arm, value):
        """Validate, publish and wait for one gripper movement"""
        args = {
            'arm': arm,
            'value': value,
        }

        ok, reason, clean = validate_gripper(args, self.cfg)

        if not ok:
            return {
                'status': 'rejected',
                'reason': reason,
                'command_sent': False,
                'data': {'requested': args},
                'images': [],
            }

        # Publish the gripper trajectory and catch subscriber or ROS publication failures.
        try:
            target_rad = self.io.send_gripper(
                clean['arm'],
                clean['value'],
            )
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed to publish gripper command: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {'target': clean},
                'images': [],
            }

        # Wait for measured gripper convergence and catch joint-state failures.
        try:
            arrival = self.io.wait_until_gripper_arrived(
                clean['arm'],
                target_rad,
            )
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed while waiting for gripper: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': True,
                'data': {
                    'target': clean,
                    'target_rad': target_rad,
                },
                'images': [],
            }

        cameras = OBSERVATION_CAMERAS
        snapshot, observation_error = self._capture_after(
            cameras,
            arrival['gripper']['stamp_s'],
        )

        status = _arrival_status(arrival)
        reason = {
            'arrived': 'gripper target reached',
            'stopped': 'gripper stopped before its target',
            'timeout': 'gripper did not settle before timeout',
        }[status]

        if observation_error is not None:
            reason += (
                '; post observation unavailable: '
                f'{observation_error}'
            )

        return {
            'status': status,
            'reason': reason,
            'command_sent': True,
            'data': {
                'target': clean,
                'target_rad': target_rad,
                'arrival': arrival,
                'post_observation': (
                    None
                    if snapshot is None
                    else self._snapshot_data(snapshot)
                ),
                'post_observation_error': observation_error,
            },
            'images': (
                []
                if snapshot is None
                else [
                    snapshot['images'][camera]
                    for camera in cameras
                ]
            ),
        }

    def move_head(self, head_joint1, head_joint2):
        return self._move_body(
            'head',
            {
                'head_joint1': head_joint1,
                'head_joint2': head_joint2,
            },
        )
    
    def move_lift(self, position_m):
        return self._move_body(
            'lift',
            {'lift_joint': position_m},
        )

    def _move_body(self, group, requested):
        """Validate, send, wait and capture one head/lift movement"""
        targets = {}

        # Validate before publishing and command
        for joint in self.io.body_groups[group]:
            value, error = _finite_float(requested, joint)

            if error is None:
                lower, upper = self.io.body_limits[joint]
                if not lower <= value <= upper:
                    error = (
                        f'{joint}={value} is outside '
                        f'[{lower}, {upper}]'
                    )

            if error is not None:
                return {
                    'status': 'rejected',
                    'reason': error,
                    'command_sent': False,
                    'data': {'requested': requested},
                    'images': [],
                }

            targets[joint] = value

        # Check that measured state is available before sending
        try:
            self.io.get_body_state(group=group)
            sent_at = self.io.send_body(group, targets)
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed before sending {group} command: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {'target': targets},
                'images': [],
            }

        try:
            arrival = self.io.wait_until_body_arrived(
                group,
                targets,
                after_received=sent_at,
            )
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed while waiting for {group}: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': True,
                'data': {'target': targets},
                'images': [],
            }

        # Require images newer than every measured joint in the result.
        after_stamp_s = max(
            state['stamp_s']
            for state in arrival['joints'].values()
        )

        snapshot, observation_error = self._capture_after(
            OBSERVATION_CAMERAS,
            after_stamp_s,
        )

        status = 'arrived' if arrival['arrived'] else 'timeout'
        reason = (
            f'{group} reached its joint targets'
            if arrival['arrived']
            else f'{group} did not settle before timeout'
        )

        if observation_error is not None:
            reason += f'; post-action observation failed: {observation_error}'

        return {
            'status': status,
            'reason': reason,
            'command_sent': True,
            'data': {
                'target': targets,
                'arrival': arrival,
                'post_observation': (
                    None
                    if snapshot is None
                    else self._snapshot_data(snapshot)
                ),
                'post_observation_error': observation_error,
            },
            'images': (
                []
                if snapshot is None
                else [
                    snapshot['images'][camera]
                    for camera in OBSERVATION_CAMERAS
                ]
            ),
        }

    
    def dispatch(self, name, args):
        """Validate and execute one model-requested tool call."""
        tool_specs = {
            'get_state': {
                'required': set(),
                'allowed': set(),
            },
            'observe': {
                'required': set(),
                'allowed': set(),
            },
            'move': {
                'required': {
                    'arm',
                    'x',
                    'y',
                    'z',
                    'roll',
                    'pitch',
                    'yaw',
                },
                'allowed': {
                    'arm',
                    'x',
                    'y',
                    'z',
                    'roll',
                    'pitch',
                    'yaw',
                },
            },
            'set_gripper': {
                'required': {'arm', 'value'},
                'allowed': {'arm', 'value'},
            },
            'move_head': {
                'required': {'head_joint1', 'head_joint2'},
                'allowed': {'head_joint1', 'head_joint2'},
            },
            'move_lift': {
                'required': {'position_m'},
                'allowed': {'position_m'},
            },
        }

        # only allowed tools are accepted
        if not isinstance(name, str) or name not in tool_specs:
            return {
                'status': 'rejected',
                'reason': f'unknown tool {name!r}',
                'command_sent': False,
                'data': {
                    'tool': repr(name),
                },
                'images': [],
            }

        if not isinstance(args, dict):
            return {
                'status': 'rejected',
                'reason': (
                    f'{name} arguments must be an object, '
                    f'not {type(args).__name__}'
                ),
                'command_sent': False,
                'data': {
                    'tool': name,
                },
                'images': [],
            }

        spec = tool_specs[name]
        missing = sorted(spec['required'] - set(args))
        unexpected = sorted(
            (
                key
                for key in args
                if key not in spec['allowed']
            ),
            key=repr,
        )

        if missing:
            return {
                'status': 'rejected',
                'reason': (
                    f'{name} missing required arguments: '
                    f'{", ".join(missing)}'
                ),
                'command_sent': False,
                'data': {
                    'tool': name,
                    'missing': missing,
                },
                'images': [],
            }

        if unexpected:
            return {
                'status': 'rejected',
                'reason': (
                    f'{name} received unexpected arguments: '
                    f'{", ".join(map(str, unexpected))}'
                ),
                'command_sent': False,
                'data': {
                    'tool': name,
                    'unexpected': unexpected,
                },
                'images': [],
            }

        handlers = {
            'get_state': self.get_state,
            'observe': self.observe,
            'move': self.move,
            'set_gripper': self.set_gripper,
            'move_head': self.move_head,
            'move_lift': self.move_lift,
        }

        # Execute the selected tool and convert unexpected failures to a result.
        try:
            result = handlers[name](**args)
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'{name} failed unexpectedly: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': (
                    False
                    if name in ('get_state', 'observe')
                    else None
                ),
                'data': {
                    'tool': name,
                },
                'images': [],
            }

        required_result_keys = {
            'status',
            'reason',
            'command_sent',
            'data',
            'images',
        }

        if not isinstance(result, dict):
            return {
                'status': 'error',
                'reason': (
                    f'{name} returned {type(result).__name__}, '
                    'expected an object'
                ),
                'command_sent': None,
                'data': {
                    'tool': name,
                },
                'images': [],
            }

        missing_result_keys = sorted(
            required_result_keys - set(result)
        )

        if missing_result_keys:
            return {
                'status': 'error',
                'reason': (
                    f'{name} result is missing fields: '
                    f'{", ".join(missing_result_keys)}'
                ),
                'command_sent': result.get('command_sent'),
                'data': {
                    'tool': name,
                    'missing': missing_result_keys,
                },
                'images': [],
            }

        return result
