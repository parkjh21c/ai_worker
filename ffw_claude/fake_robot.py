#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Jiho Park

"""A robot that is not there, for dry runs without ROS

FakeRobotTools answers the RobotTools dispatch interface. It sits under the
real ObservationGuard and rollout, so a dry run checks Claude Code, its login,
the sandbox and the tool flow while nothing moves. Every action arrives
exactly; the cameras show a labeled gray frame.
"""

import base64
import math
import time

import robot_layer  # noqa: F401
from robot_tools import validate_base, validate_gripper, validate_move
from transforms import rpy_to_quat


CAMERAS = {'head': (672, 376), 'wrist_left': (424, 240), 'wrist_right': (424, 240)}
# 32x24 gray JPEG for hosts without OpenCV
GRAY_JPEG = base64.b64decode(
    '/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDABALDA4MChAODQ4SERATGCgaGBYWGDEjJR0oOjM9PDkzODdASFxOQERX'
    'RTc4UG1RV19iZ2hnPk1xeXBkeFxlZ2P/2wBDARESEhgVGC8aGi9jQjhCY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2Nj'
    'Y2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2P/wAARCAAYACADASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAA'
    'AAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAk'
    'M2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKT'
    'lJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QA'
    'HwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdh'
    'cRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hp'
    'anN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk'
    '5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwAooooAKKKKACiiigAooooA/9k='
)
START_POSE = {
    'right': {'x': 0.30, 'y': -0.25, 'z': 0.95, 'roll': 0.0, 'pitch': -1.57, 'yaw': 0.0},
    'left': {'x': 0.30, 'y': 0.25, 'z': 0.95, 'roll': 0.0, 'pitch': -1.57, 'yaw': 0.0},
}
ARM_BASE_Z = 1.43  # Shoulder height at lift 0, from the robot contract


def render(camera, size, label):
    """Return a labeled JPEG, or the plain gray one without OpenCV"""
    try:
        import cv2
        import numpy as np
    except ImportError:
        return GRAY_JPEG, (32, 24)
    width, height = size
    image = np.full((height, width, 3), 96, np.uint8)
    for row, text in enumerate((f'DRY RUN - {camera}', label)):
        cv2.putText(image, text, (12, 32 + 28 * row), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)
    ok, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise RuntimeError('JPEG encoding failed')
    return buffer.tobytes(), size


class FakeRobotTools:
    """RobotTools stand-in whose actions always arrive"""

    def __init__(self, cfg):
        self.cfg = cfg
        self.frame = cfg['frames']['base']
        self.arms = {arm: dict(START_POSE[arm]) for arm in cfg['frames']['ee']}
        self.grippers = dict.fromkeys(cfg['frames']['ee'], 0.0)
        self.body = {'head_joint1': 0.0, 'head_joint2': 0.0, 'lift_joint': 0.0}
        self.base = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.actions = 0

    def _state(self):
        now = time.time()
        low, high = (float(value) for value in self.cfg['limits']['gripper_rad'])
        return {
            'frame': self.frame,
            'clock': {'kind': 'system', 'now_s': now},
            'arms': {
                arm: {
                    'ee': {'frame': self.frame, **pose, 'stamp_s': now},
                    'gripper': {
                        'joint': self.cfg['cyclo']['gripper_joint'][arm],
                        'value': self.grippers[arm],
                        'position_rad': low + self.grippers[arm] * (high - low),
                        'stamp_s': now,
                    },
                }
                for arm, pose in self.arms.items()
            },
            'body': {
                name: {'position': value, 'unit': 'm' if name == 'lift_joint' else 'rad',
                       'stamp_s': now}
                for name, value in self.body.items()
            },
            'base': {'frame': 'odom', 'child_frame': self.frame, **self.base,
                     'vx': 0.0, 'vy': 0.0, 'wz': 0.0, 'stamp_s': now},
        }

    def _snapshot(self):
        state = self._state()
        label = f'after {self.actions} actions'
        images = []
        for camera, size in CAMERAS.items():
            jpeg, (width, height) = render(camera, size, label)
            images.append({'camera': camera, 'jpeg': jpeg, 'width': width, 'height': height,
                           'stamp_s': state['clock']['now_s'], 'receive_age_s': 0.0})
        data = {
            **state,
            'stamp_s': state['clock']['now_s'],
            'max_offset_s': 0.0,
            'cameras': {
                image['camera']: {key: value for key, value in image.items() if key != 'jpeg'}
                for image in images
            },
        }
        return data, images

    def _done(self, target, arrival):
        self.actions += 1
        data, images = self._snapshot()
        return {
            'status': 'arrived',
            'reason': 'dry run: arrived without moving anything',
            'command_sent': True,
            'data': {'target': target, 'arrival': arrival, 'post_observation': data,
                     'post_observation_error': None},
            'images': images,
        }

    @staticmethod
    def _rejected(reason, args):
        return {'status': 'rejected', 'reason': reason, 'command_sent': False,
                'data': {'requested': args}, 'images': []}

    def dispatch(self, name, args):
        if name == 'get_state':
            return {'status': 'ok', 'reason': 'state observed', 'command_sent': False,
                    'data': self._state(), 'images': []}
        if name == 'observe':
            data, images = self._snapshot()
            return {'status': 'ok', 'reason': 'synchronized observation captured',
                    'command_sent': False, 'data': data, 'images': images}
        if name == 'move':
            pose = self.arms.get(args.get('arm'))
            if pose is None:
                return self._rejected(f'unknown arm {args.get("arm")!r}', args)
            current = {**pose, 'quat': rpy_to_quat(pose['roll'], pose['pitch'], pose['yaw'])}
            arm_base_z = ARM_BASE_Z + self.body['lift_joint']
            ok, reason, target = validate_move(args, self.cfg, current, arm_base_z)
            if not ok:
                result = self._rejected(reason, args)
                result['data']['current_pose'] = current
                return result
            pose.update({key: target[key] for key in pose})
            return self._done(target, {'arrived': True, 'stopped': False,
                                       'position_error_m': 0.0, 'orientation_error_deg': 0.0})
        if name == 'set_gripper':
            ok, reason, target = validate_gripper(args, self.cfg)
            if not ok:
                return self._rejected(reason, args)
            self.grippers[target['arm']] = target['value']
            return self._done(target, {'arrived': True, 'stopped': False, 'error_rad': 0.0})
        if name in ('move_head', 'move_lift'):
            key_map = {'position_m': 'lift_joint'}
            limits = self.cfg['limits']
            ranges = {'head_joint1': limits['head_joint1_rad'],
                      'head_joint2': limits['head_joint2_rad'],
                      'lift_joint': limits['lift_joint_m']}
            target = {key_map.get(key, key): float(value) for key, value in args.items()}
            for joint, value in target.items():
                low, high = ranges[joint]
                if not low <= value <= high:
                    return self._rejected(f'{joint} {value} outside [{low}, {high}]', args)
            self.body.update(target)
            return self._done(target, {'arrived': True, 'stopped': False})
        if name == 'move_base':
            ok, reason, command = validate_base(args, self.cfg['base'])
            if not ok:
                return self._rejected(reason, args)
            dx = command['vx'] * command['duration_s']
            dy = command['vy'] * command['duration_s']
            dyaw = command['wz'] * command['duration_s']
            yaw = self.base['yaw']
            self.base['x'] += dx * math.cos(yaw) - dy * math.sin(yaw)
            self.base['y'] += dx * math.sin(yaw) + dy * math.cos(yaw)
            self.base['yaw'] = math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw))
            result = self._done(command, {'arrived': True, 'stopped': False})
            result['data']['motion'] = {
                'commanded': command,
                'duration_s': command['duration_s'],
                'rest_confirmed': True,
                'odom_measured': {'frame': self.frame, 'dx': dx, 'dy': dy, 'dyaw': dyaw},
            }
            return result
        return self._rejected(f'unknown tool {name!r}', args)
