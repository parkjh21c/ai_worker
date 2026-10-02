"""
Check RobotTools.move_base and validate_base with a fake RobotIO.

No ROS, no Gazebo, no model:
    python3 poc_codex_base/check_base_tools.py
"""

import math
from pathlib import Path
import sys

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from robot_tools import RobotTools, validate_base  # noqa: E402


CFG = yaml.safe_load((HERE / 'config_gazebo.yaml').read_text(encoding='utf-8'))
BASE = CFG['base']

failures = []


def check(ok, label, detail=''):
    print(f'[{"ok" if ok else "FAIL"}] {label}{"  " + detail if detail else ""}')
    if not ok:
        failures.append(label)


def odom(x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0, stamp_s=10.0):
    return {'frame': 'odom', 'child_frame': 'base_link', 'x': x, 'y': y, 'yaw': yaw,
            'vx': vx, 'vy': vy, 'wz': wz, 'stamp_s': stamp_s, 'received': 1.0, 'age_s': 0.0}


class FakeIO:
    """Records calls; each attribute decides what the next base call returns."""

    def __init__(self):
        self.hand = {'right': (0.431, -0.2), 'left': (0.431, 0.2)}
        self.lift = -0.08
        self.base = odom()
        self.ownership = None
        self.run_outcome = 'completed'
        self.run_raises = None
        self.stop_error = None
        self.rested = True
        self.end = odom(x=0.2, stamp_s=12.0)
        self.snapshot_fails = False
        self.sent = []

    def get_ee_pose(self, arm):
        x, y = self.hand[arm]
        return {'frame': 'base_link', 'x': x, 'y': y, 'z': 1.1, 'quat': [0, 0, 0, 1]}

    def get_body_state(self, group=None):
        return {'lift_joint': {'position': self.lift, 'unit': 'm', 'stamp_s': 10.0}}

    def get_base_state(self):
        return dict(self.base)

    def base_command_problem(self):
        return self.ownership

    def send_base(self, vx, vy, wz, duration_s):
        if self.run_raises:
            raise self.run_raises
        self.sent.append((vx, vy, wz, duration_s))
        return {'command_sent': True, 'outcome': self.run_outcome,
                'detail': None if self.run_outcome == 'completed' else 'detail',
                'stop_error': self.stop_error, 'stopped_at': 5.0,
                'commanded': {'vx': vx, 'vy': vy, 'wz': wz}, 'duration_s': duration_s,
                'elapsed_s': duration_s, 'start': odom(), 'path_m': 0.19, 'path_rad': 0.0}

    def wait_until_base_stopped(self, after_received):
        return {'rested': self.rested, 'reason': 'base at rest' if self.rested else 'no rest',
                'state': self.end, 'elapsed_s': 0.5}

    def get_snapshot(self, cameras, after_stamp_s=None):
        if self.snapshot_fails:
            raise TimeoutError('no snapshot')
        images = {c: {'camera': c, 'jpeg': b'', 'stamp_s': 12.1} for c in cameras}
        return {'stamp_s': 12.1, 'frame': 'base_link', 'arms': {}, 'body': {},
                'base': odom(x=0.2, stamp_s=12.1), 'images': images,
                'max_offset_s': 0.0, 'clock': {}}


def move(io, tools=None, **overrides):
    args = {'vx': 0.1, 'vy': 0.0, 'wz': 0.0, 'duration_s': 2.0, **overrides}
    tools = tools or RobotTools(io, CFG)
    return tools, tools.dispatch('move_base', args)


def check_validate():
    def accepted(**args):
        full = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0, 'duration_s': 1.0, **args}
        return validate_base(full, BASE)[0]

    check(accepted(vx=0.1), 'forward at the deadband')
    check(accepted(vx=-0.15, duration_s=1.0), 'backward at max speed')
    check(accepted(wz=0.3, duration_s=1.0), 'pure rotation')
    check(not accepted(vx=0.05), 'below deadband rejected')
    check(not accepted(wz=0.05), 'rotation below deadband rejected')
    check(not accepted(vx=0.12, vy=0.12), 'diagonal over the resultant limit rejected',
          f'hypot {math.hypot(0.12, 0.12):.3f}')
    check(accepted(vx=0.1, vy=0.1, duration_s=1.0), 'diagonal at hypot 0.141 accepted')
    check(not accepted(vx=0.15, duration_s=2.0), 'step over max_step_m rejected', '0.3 m')
    check(not accepted(wz=0.3, duration_s=2.0), 'rotation over max_step_rad rejected', '0.6 rad')
    check(not accepted(), 'all zero rejected')
    check(not accepted(vx=float('nan')), 'NaN rejected')
    check(not accepted(vx=float('inf')), 'infinity rejected')
    check(not accepted(vx=True), 'bool rejected')
    check(not accepted(vx='0.1'), 'string rejected')
    check(not accepted(vx=b'0.1'), 'bytes rejected')
    check(not accepted(vx=0.1, duration_s=0.0), 'zero duration rejected')
    check(not accepted(vx=0.1, duration_s=-1.0), 'negative duration rejected')
    check(not accepted(vx=0.1, duration_s=2.5), 'duration over max rejected')


def check_move_base():
    io = FakeIO()
    _, result = move(io)
    motion = result['data'].get('motion') or {}
    check(result['status'] == 'arrived' and result['command_sent'] is True, 'normal run arrives')
    check(motion.get('rest_confirmed') is True, 'rest confirmed in motion')
    check(abs((motion.get('odom_measured') or {}).get('dx', 0) - 0.2) < 1e-9,
          'odom-measured dx reported', str(motion.get('odom_measured')))
    check(result['data']['post_observation']['base']['x'] == 0.2, 'post observation has base')
    check(len(result['images']) == 3, 'three images')

    io = FakeIO()
    _, result = move(io, vx=0.05)
    check(result['status'] == 'rejected' and not result['data']['invalidate_observation']
          and not io.sent, 'invalid input: rejected, observation kept, nothing sent')

    io = FakeIO()
    io.hand['right'] = (0.6, -0.2)
    _, result = move(io)
    check(result['status'] == 'rejected' and not result['data']['invalidate_observation']
          and not io.sent, 'arm extended: rejected by posture, observation kept')

    io = FakeIO()
    io.lift = -0.6
    _, result = move(io)
    check(result['status'] == 'rejected' and not io.sent, 'lift out of range: rejected')

    io = FakeIO()
    io.base = odom(vx=0.05)
    _, result = move(io)
    check(result['status'] == 'rejected' and result['data']['invalidate_observation']
          and not io.sent, 'base moving: rejected, observation invalidated')

    io = FakeIO()
    io.ownership = '1 other publisher(s) on /cmd_vel'
    _, result = move(io)
    check(result['status'] == 'rejected' and result['data']['invalidate_observation']
          and not io.sent, 'other cmd_vel publisher: rejected, observation invalidated')

    io = FakeIO()
    io.run_raises = TimeoutError('no fresh /odom sample')
    _, result = move(io)
    check(result['status'] == 'rejected' and result['command_sent'] is False
          and result['data']['invalidate_observation'], 'send_base raised before publish')

    for outcome in ('odom_lost', 'clock_stopped', 'clock_reset', 'ownership',
                    'path_limit', 'cancelled', 'error'):
        io = FakeIO()
        io.run_outcome = outcome
        _, result = move(io)
        check(result['status'] == 'error' and result['command_sent'] is True,
              f'{outcome}: error, command_sent True')

    io = FakeIO()
    io.stop_error = 'RuntimeError: publisher gone'
    _, result = move(io)
    check(result['status'] == 'error', 'zero publish failure: error')

    io = FakeIO()
    io.rested = False
    _, result = move(io)
    check(result['status'] == 'timeout', 'rest not confirmed: timeout')

    io = FakeIO()
    io.run_outcome = 'stalled'
    io.rested = False
    _, result = move(io)
    check(result['status'] == 'timeout', 'stall without rest: timeout, not stopped')

    io = FakeIO()
    io.run_outcome = 'stalled'
    io.end = odom(x=0.003, stamp_s=12.0)
    tools, result = move(io)
    check(result['status'] == 'stopped', 'stall with rest: stopped')
    _, retry = move(io, tools)
    check(retry['status'] == 'rejected' and len(io.sent) == 1,
          'same-direction retry after stall rejected')
    _, other = move(io, tools, vx=0.0, vy=0.1)
    check(other['status'] != 'rejected' or 'stalled' not in other['reason'],
          'perpendicular direction allowed after stall')

    io = FakeIO()
    io.run_outcome = 'stalled'
    io.end = odom(x=0.003, stamp_s=12.0)
    tools, _ = move(io)
    check(tools._base_stall is not None, 'stall recorded')
    io.run_outcome = 'completed'
    io.end = odom(x=0.2, stamp_s=12.0)
    tools.observe = lambda: {'status': 'ok', 'reason': '', 'command_sent': False,
                             'data': {}, 'images': []}
    tools.dispatch('observe', {})
    _, retry = move(io, tools)
    check(retry['status'] == 'arrived', 'observe lifts the stall block')

    io = FakeIO()
    io.end = odom(x=0.004, stamp_s=12.0)
    tools, result = move(io, duration_s=0.5)
    check(result['status'] == 'stopped', 'completed but barely moved: stopped')
    io.end = odom(x=0.05, stamp_s=12.0)
    _, retry = move(io, tools, duration_s=0.5)
    check(retry['status'] == 'arrived', 'one negligible move does not block the retry')

    io = FakeIO()
    io.end = odom(x=0.004, stamp_s=12.0)
    tools, first = move(io, duration_s=0.5)
    _, second = move(io, tools, duration_s=0.5)
    _, third = move(io, tools, duration_s=0.5)
    check(first['status'] == 'stopped' and second['status'] == 'stopped'
          and third['status'] == 'rejected' and len(io.sent) == 2,
          'two negligible moves in a row block the direction')
    check('observe' not in third['reason'] and 'head' not in third['reason'],
          'rejection does not suggest a way around the block', third['reason'])

    io = FakeIO()
    io.end = odom(yaw=0.25, stamp_s=12.0)
    _, result = move(io, vx=0.0, wz=0.2)
    check(result['status'] == 'arrived', 'pure rotation that turned arrives')

    io = FakeIO()
    io.snapshot_fails = True
    _, result = move(io)
    check(result['status'] == 'arrived' and result['data']['post_observation'] is None
          and result['data']['post_observation_error'], 'missing post observation reported')


def main():
    check_validate()
    check_move_base()
    print('PASS' if not failures else f'FAIL: {", ".join(failures)}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
