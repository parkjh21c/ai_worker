"""
Check robot_base through RobotRollout and ObservationGuard with a fake RobotIO.

The real RobotTools, guard and rollout run; only RobotIO is fake. No ROS,
no Gazebo, no model:
    python3 poc_codex_base/check_base_rollout.py
"""

from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from check_base_tools import FakeIO, odom  # noqa: E402
from codex_host import UncertainExecution  # noqa: E402
from observation_guard_gazebo import ObservationGuard  # noqa: E402
from robot_tools import RobotTools  # noqa: E402
from rollout import RobotRollout  # noqa: E402


CFG = yaml.safe_load((HERE / 'config_gazebo.yaml').read_text(encoding='utf-8'))

failures = []


def check(ok, label, detail=''):
    print(f'[{"ok" if ok else "FAIL"}] {label}{"  " + detail if detail else ""}')
    if not ok:
        failures.append(label)


class FakeRobotIO(FakeIO):
    """FakeIO plus the state and snapshot shapes the guard checks."""

    def __init__(self):
        super().__init__()
        self.sim_s = 12.1

    def _arms(self, stamp_s):
        return {arm: {'ee': {'frame': 'base_link', 'x': x, 'y': y, 'z': 1.1,
                             'quat': [0.0, 0.0, 0.0, 1.0], 'stamp_s': stamp_s},
                      'gripper': {'joint': f'gripper_{arm[0]}_joint1', 'position_rad': 0.0,
                                  'stamp_s': stamp_s}}
                for arm, (x, y) in self.hand.items()}

    def _body(self, stamp_s):
        return {joint: {'position': value, 'unit': unit, 'stamp_s': stamp_s}
                for joint, value, unit in (('head_joint1', 0.0, 'rad'),
                                           ('head_joint2', 0.0, 'rad'),
                                           ('lift_joint', self.lift, 'm'))}

    def _clock(self):
        return {'sim_s': self.sim_s, 'received_age_s': 0.0, 'advanced_age_s': 0.0, 'rewinds': 0}

    def get_state(self):
        return {'frame': 'base_link', 'arms': self._arms(self.sim_s),
                'body': self._body(self.sim_s),
                'base': dict(self.base, stamp_s=self.sim_s), 'clock': self._clock()}

    def get_snapshot(self, cameras, after_stamp_s=None):
        if self.snapshot_fails:
            raise TimeoutError('no snapshot')
        images = {c: {'camera': c, 'jpeg': b'jpg', 'stamp_s': self.sim_s,
                      'receive_age_s': 0.01, 'width': 4, 'height': 3} for c in cameras}
        return {'stamp_s': self.sim_s, 'frame': 'base_link', 'arms': self._arms(self.sim_s),
                'body': self._body(self.sim_s), 'base': dict(self.base, stamp_s=self.sim_s),
                'images': images, 'max_offset_s': 0.0, 'clock': self._clock()}

    def send_base(self, vx, vy, wz, duration_s):
        result = super().send_base(vx, vy, wz, duration_s)
        result['start'] = dict(self.base)
        self.base = dict(self.end)   # the robot is where it came to rest
        return result


def new_rollout(io, run_dir):
    guard = ObservationGuard(RobotTools(io, CFG), CFG)
    return RobotRollout(guard, CFG, run_dir, 'test task', max_actions=80, max_seconds=1800)


def packet(reply):
    return json.loads(reply['contentItems'][0]['text'])


def base_call(rollout, request_id, **overrides):
    args = {'request_id': request_id, 'reason': 'test', 'vx': 0.1, 'vy': 0.0,
            'wz': 0.0, 'duration_s': 2.0, **overrides}
    return packet(rollout.handle_tool_call('robot_base', args))


def main():
    run_dir = Path(tempfile.mkdtemp(prefix='check_base_rollout_'))

    io = FakeRobotIO()
    rollout = new_rollout(io, run_dir / 'a')
    names = [spec['name'] for spec in rollout.tool_specs()]
    check('robot_base' in names, 'robot_base is offered')

    start = packet(rollout.handle_tool_call('robot_start', {}))
    check(start['observation']['base'] == {'frame': 'odom', 'xy': [0.0, 0.0], 'yaw': 0.0},
          'observation has the base pose', str(start['observation'].get('base')))
    check('robot_base' in start['next_call']['tool'], 'next_call lists robot_base')

    io.end = odom(x=0.2, stamp_s=12.0)
    result = base_call(rollout, start['action_context']['request_id'])
    motion = (result.get('result') or {}).get('motion') or {}
    check(result['status'] == 'arrived' and result['action_executed'] is True, 'arrived',
          result['reason'])
    check(motion.get('odom_measured') == {'frame': 'start base_link', 'dx_m': 0.2,
                                          'dy_m': 0.0, 'dyaw_rad': 0.0},
          'packet reports odom-measured dx_m, dy_m, dyaw_rad', str(motion.get('odom_measured')))
    check(result['observation']['base']['xy'] == [0.2, 0.0], 'new observation after the move')
    check(result['action_context']['movement_allowed'] is True, 'movement allowed after arrival')

    request = result['action_context']['request_id']
    rejected = base_call(rollout, request, vx=0.05)
    check(rejected['rejection'] == 'invalid_action'
          and rejected['action_context']['movement_allowed'] is True,
          'below deadband: invalid_action, observation kept')

    io.ownership = '1 other publisher(s) on /cmd_vel'
    not_ready = base_call(rollout, rejected['action_context']['request_id'])
    check(not_ready['rejection'] == 'not_ready'
          and not_ready['action_context']['movement_allowed'] is False
          and not_ready['next_call']['tool'] == 'robot_observe',
          'not ready: observation invalidated, observe next')
    io.ownership = None

    observed = packet(rollout.handle_tool_call('robot_observe', {}))
    io.base = odom(x=0.2, vx=0.05)   # moving, though it has barely moved
    moving = base_call(rollout, observed['action_context']['request_id'])
    check(moving['rejection'] == 'observation_rejected' and 'moving' in moving['reason'],
          'base moving: any action rejected by the guard', moving['reason'])

    io.base = odom(x=0.2)
    observed = packet(rollout.handle_tool_call('robot_observe', {}))
    io.base = odom(x=0.25)            # shifted 5 cm since the observation
    shifted = packet(rollout.handle_tool_call('robot_head', {
        'request_id': observed['action_context']['request_id'], 'reason': 'test',
        'head_joint1': 0.0, 'head_joint2': 0.0}))
    check(shifted['rejection'] == 'observation_rejected' and 'base position' in shifted['reason'],
          'base shifted after the observation: other actions rejected', shifted['reason'])

    io = FakeRobotIO()
    io.run_outcome = 'odom_lost'
    rollout = new_rollout(io, run_dir / 'b')
    start = packet(rollout.handle_tool_call('robot_start', {}))
    try:
        base_call(rollout, start['action_context']['request_id'])
        raised = False
    except UncertainExecution:
        raised = True
    check(raised and rollout.outcome()['finish_reason'] == 'uncertain_execution',
          'odom lost mid-motion: uncertain_execution ends the run')
    halted = packet(rollout.handle_tool_call('robot_observe', {}))
    check(halted['rejection'] == 'episode_finished', 'nothing runs after that')

    io = FakeRobotIO()
    io.rested = False
    rollout = new_rollout(io, run_dir / 'c')
    start = packet(rollout.handle_tool_call('robot_start', {}))
    try:
        base_call(rollout, start['action_context']['request_id'])
        raised = False
    except UncertainExecution:
        raised = True
    check(raised, 'rest not confirmed: uncertain_execution')

    hardware = deepcopy(CFG)
    del hardware['base']
    rollout = RobotRollout(ObservationGuard(RobotTools(FakeRobotIO(), hardware), hardware),
                           hardware, run_dir / 'd', 'test task', 80, 1800)
    check('robot_base' not in [s['name'] for s in rollout.tool_specs()],
          'without a base: section, robot_base is not offered')

    print('PASS' if not failures else f'FAIL: {", ".join(failures)}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
