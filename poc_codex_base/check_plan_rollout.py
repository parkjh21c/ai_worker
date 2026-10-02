"""
Check robot_execute_plan through RobotRollout, ObservationGuard and RobotTools
with a fake RobotIO.

The tool call the model would make is built here and passed to
handle_tool_call, so no model tokens are used. No ROS, no Gazebo, no model:
    python3 poc_codex_base/check_plan_rollout.py
"""

import json
import math
from pathlib import Path
import sys
import tempfile

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from check_base_rollout import FakeRobotIO  # noqa: E402
from check_base_tools import odom  # noqa: E402
from codex_host import UncertainExecution  # noqa: E402
from observation_guard_gazebo import ObservationGuard  # noqa: E402
from plan_executor import ALLOWED, PLAN_TOOL  # noqa: E402
from robot_tools import RobotTools  # noqa: E402
from rollout import RobotRollout  # noqa: E402
from transforms import rpy_to_quat  # noqa: E402


CFG = yaml.safe_load((HERE / 'config_gazebo.yaml').read_text(encoding='utf-8'))
MAX_STEPS = CFG['plan']['max_steps']
POSE = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
START = {'right': (0.431, -0.2, 1.1), 'left': (0.431, 0.2, 1.1)}

failures = []


def check(ok, label, detail=''):
    print(f'[{"ok" if ok else "FAIL"}] {label}{"  " + detail if detail else ""}')
    if not ok:
        failures.append(label)


class PlanIO(FakeRobotIO):
    """FakeRobotIO whose arms, grippers, head and lift move when commanded.

    outcomes holds one entry per action, in order: arrived, stopped, timeout,
    no_observation (arrives, then the post-action snapshot fails) or raise
    (the wait raises). Once it is empty every action arrives.
    """

    def __init__(self, outcomes=()):
        super().__init__()
        self.pose = {arm: dict(zip(POSE, (*xyz, 0.0, 0.0, 0.0))) for arm, xyz in START.items()}
        self.grip = {arm: 0.0 for arm in START}
        self.head = {'head_joint1': 0.0, 'head_joint2': 0.0}
        self.outcomes = list(outcomes)
        self.drop_next_snapshot = False
        self.body_groups = {'head': ('head_joint1', 'head_joint2'), 'lift': ('lift_joint',)}
        self.body_limits = {
            'head_joint1': tuple(CFG['limits']['head_joint1_rad']),
            'head_joint2': tuple(CFG['limits']['head_joint2_rad']),
            'lift_joint': tuple(CFG['limits']['lift_joint_m']),
        }

    # State ---------------------------------------------------------------

    def _ee(self, arm):
        p = self.pose[arm]
        return {'frame': 'base_link', 'x': p['x'], 'y': p['y'], 'z': p['z'],
                'quat': list(rpy_to_quat(p['roll'], p['pitch'], p['yaw'])),
                'stamp_s': self.sim_s}

    def _arms(self, stamp_s):
        return {arm: {'ee': self._ee(arm),
                      'gripper': {'joint': f'gripper_{arm[0]}_joint1',
                                  'position_rad': self.grip[arm], 'stamp_s': stamp_s}}
                for arm in self.pose}

    def _body(self, stamp_s):
        joints = dict(self.head, lift_joint=self.lift)
        return {joint: {'position': value, 'unit': 'm' if joint == 'lift_joint' else 'rad',
                        'stamp_s': stamp_s}
                for joint, value in joints.items()}

    def get_ee_pose(self, arm):
        return self._ee(arm)

    def get_arm_base_z(self):
        return {'frame': 'base_link', 'z': 0.9 + self.lift, 'stamp_s': self.sim_s}

    def get_body_state(self, group=None):
        return self._body(self.sim_s)

    def get_snapshot(self, cameras, after_stamp_s=None):
        if self.drop_next_snapshot:
            self.drop_next_snapshot = False
            raise TimeoutError('no snapshot')
        return super().get_snapshot(cameras, after_stamp_s)

    # Commands ------------------------------------------------------------

    def _settle(self, apply):
        outcome = self.outcomes.pop(0) if self.outcomes else 'arrived'
        if outcome == 'raise':
            raise RuntimeError('controller lost')
        if outcome in ('arrived', 'no_observation'):
            apply()
        self.drop_next_snapshot = outcome == 'no_observation'
        self.sim_s += 1.0
        return {'arrived': outcome in ('arrived', 'no_observation'),
                'stopped': outcome == 'stopped', 'elapsed_s': 1.0}

    def send_pose(self, arm, target):
        self.sent.append(('robot_move', arm, {key: target[key] for key in POSE}))
        return {'quat': list(rpy_to_quat(target['roll'], target['pitch'], target['yaw'])),
                'sent_x': target['x']}

    def wait_until_arrived(self, arm, target, target_quat):
        flags = self._settle(lambda: self.pose[arm].update({key: target[key] for key in POSE}))
        return {**flags, 'pose': self._ee(arm),
                'position_error_m': 0.0, 'orientation_error_deg': 0.0}

    def send_gripper(self, arm, value):
        self.sent.append(('robot_gripper', arm, value))
        return float(value)   # gripper_rad is [0, 1] in config_gazebo.yaml

    def wait_until_gripper_arrived(self, arm, target_rad):
        flags = self._settle(lambda: self.grip.__setitem__(arm, target_rad))
        return {**flags, 'gripper': {'position_rad': self.grip[arm], 'stamp_s': self.sim_s},
                'error_rad': 0.0}

    def send_body(self, group, targets):
        self.sent.append((f'robot_{group}', dict(targets)))
        return self.sim_s

    def wait_until_body_arrived(self, group, targets, after_received):
        def apply():
            for joint, value in targets.items():
                if joint == 'lift_joint':
                    self.lift = value
                else:
                    self.head[joint] = value
        flags = self._settle(apply)
        body = self._body(self.sim_s)
        return {**flags, 'joints': {joint: body[joint] for joint in targets}}


# Steps the model would write -------------------------------------------

def move(dx=0.0, dy=0.0, dz=0.0, arm='right', reason='move step'):
    x, y, z = START[arm]
    return {'tool': 'robot_move',
            'args': {'arm': arm, 'x': x + dx, 'y': y + dy, 'z': z + dz,
                     'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'reason': reason}


def gripper(value, arm='right'):
    return {'tool': 'robot_gripper', 'args': {'arm': arm, 'value': value},
            'reason': 'gripper step'}


def head(j1=0.0, j2=0.0):
    return {'tool': 'robot_head', 'args': {'head_joint1': j1, 'head_joint2': j2},
            'reason': 'head step'}


def lift(position_m):
    return {'tool': 'robot_lift', 'args': {'position_m': position_m}, 'reason': 'lift step'}


def with_args(step, **changes):
    return {**step, 'args': {**step['args'], **changes}}


# Running ----------------------------------------------------------------

RUN_DIR = Path(tempfile.mkdtemp(prefix='check_plan_rollout_'))
_runs = []


def started(outcomes=(), max_actions=80):
    io = PlanIO(outcomes)
    guard = ObservationGuard(RobotTools(io, CFG), CFG)
    _runs.append(None)
    rollout = RobotRollout(guard, CFG, RUN_DIR / f'run_{len(_runs):02d}', 'test task',
                           max_actions=max_actions, max_seconds=1800)
    start = packet(rollout.handle_tool_call('robot_start', {}))
    return io, rollout, start['action_context']['request_id']


def packet(reply):
    return json.loads(reply['contentItems'][0]['text'])


def plan(rollout, request_id, steps, reason='test plan'):
    reply = rollout.handle_tool_call(PLAN_TOOL, {'request_id': request_id, 'reason': reason,
                                                 'steps': steps})
    return packet(reply), reply


def sent_tools(io):
    return [entry[0] for entry in io.sent]


def plan_state(rollout, result):
    index = result['progress']['call_index']
    return json.loads((rollout.private_dir / f'plan_{index:04d}_state.json').read_text())


# Checks ------------------------------------------------------------------

def strict_problems(schema, path='$'):
    """Objects whose properties are not all required or that allow extra keys."""
    problems = []
    if schema.get('type') == 'object':
        if schema.get('additionalProperties') is not False:
            problems.append(f'{path}: additionalProperties is not false')
        if set(schema.get('required', [])) != set(schema.get('properties', {})):
            problems.append(f'{path}: required differs from properties')
    for key, child in schema.get('properties', {}).items():
        problems += strict_problems(child, f'{path}.{key}')
    if 'items' in schema:
        problems += strict_problems(schema['items'], f'{path}[]')
    for i, variant in enumerate(schema.get('anyOf', [])):
        problems += strict_problems(variant, f'{path}|{i}')
    return problems


def check_spec():
    rollout = RobotRollout(ObservationGuard(RobotTools(PlanIO(), CFG), CFG), CFG,
                           RUN_DIR / 'spec', 'test task', 80, 1800)
    specs = {spec['name']: spec for spec in rollout.tool_specs()}
    check(PLAN_TOOL in specs, f'{PLAN_TOOL} is offered')
    schema = specs[PLAN_TOOL]['inputSchema']
    steps = schema['properties']['steps']
    check(steps['maxItems'] == MAX_STEPS, f'steps.maxItems follows plan.max_steps ({MAX_STEPS})',
          str(steps['maxItems']))
    variants = steps['items']['anyOf']
    tools = {variant['properties']['tool']['enum'][0] for variant in variants}
    check(tools == ALLOWED, 'one step variant per allowed tool', str(sorted(tools)))
    check(all('request_id' not in v['properties']['args']['properties'] for v in variants),
          'steps carry no request_id; the host fills it in')
    problems = strict_problems(schema)
    check(not problems, 'every object lists all properties as required, no extra keys',
          '; '.join(problems))
    try:
        json.dumps(specs[PLAN_TOOL])
        serializable = True
    except (TypeError, ValueError):
        serializable = False
    check(serializable, 'plan spec is JSON serializable')


def check_invalid_plans():
    """Every static problem is rejected before the first command."""
    io, rollout, request = started()
    base = {'tool': 'robot_base',
            'args': {'vx': 0.1, 'vy': 0.0, 'wz': 0.0, 'duration_s': 1.0}, 'reason': 'r'}
    cases = [
        ('steps missing', {'request_id': request, 'reason': 'r'}),
        ('extra top-level key', {'request_id': request, 'reason': 'r',
                                 'steps': [move(dx=0.01)], 'extra': 1}),
        ('blank plan reason', {'request_id': request, 'reason': '  ',
                               'steps': [move(dx=0.01)]}),
        ('steps is not a list', {'request_id': request, 'reason': 'r', 'steps': {}}),
        ('empty steps', []),
        (f'{MAX_STEPS + 1} steps', [move(dx=0.001 * i) for i in range(MAX_STEPS + 1)]),
        ('step is not an object', ['robot_move']),
        ('robot_base is not allowed in a plan', [base]),
        ('robot_start is not allowed in a plan',
         [{'tool': 'robot_start', 'args': {}, 'reason': 'r'}]),
        ('step without reason', [{'tool': 'robot_move', 'args': move(dx=0.01)['args']}]),
        ('blank step reason', [{**move(dx=0.01), 'reason': ' '}]),
        ('missing argument', [{**move(), 'args': {k: v for k, v in move()['args'].items()
                                                  if k != 'yaw'}}]),
        ('request_id inside step args', [with_args(move(dx=0.01), request_id=request)]),
        ('number as a string', [with_args(move(), x='0.4')]),
        ('bool as a number', [with_args(gripper(0.0), value=False)]),
        ('NaN', [with_args(move(), x=float('nan'))]),
        ('infinity', [with_args(move(), x=float('inf'))]),
        ('huge integer', [with_args(move(), x=10 ** 400)]),
        ('gripper above 1', [gripper(1.5)]),
        ('head out of range', [head(j2=1.0)]),
        ('lift above 0', [lift(0.1)]),
        ('unknown arm', [with_args(move(), arm='middle')]),
        ('head not the last step', [head(0.1), move(dx=0.01)]),
        ('lift not the last step', [lift(-0.1), move(dx=0.01)]),
        ('closing gripper not the last step', [gripper(1.0), move(dx=0.01)]),
        ('partly closing gripper not the last step', [gripper(0.3), move(dx=0.01)]),
    ]
    for label, steps_or_arguments in cases:
        if isinstance(steps_or_arguments, dict):
            arguments = steps_or_arguments
        else:
            arguments = {'request_id': request, 'reason': 'r', 'steps': steps_or_arguments}
        result = packet(rollout.handle_tool_call(PLAN_TOOL, arguments))
        check(result['rejection'] == 'invalid_plan' and not result['action_executed'],
              f'invalid plan: {label}', result['reason'] or '')
    check(io.sent == [], 'no command sent for any invalid plan', str(io.sent))
    check(packet(rollout.handle_tool_call('robot_observe', {}))['action_context']
          ['movement_allowed'], 'observation still usable afterwards')
    return io, rollout


def check_preconditions():
    io = PlanIO()
    rollout = RobotRollout(ObservationGuard(RobotTools(io, CFG), CFG), CFG,
                           RUN_DIR / 'not_started', 'test task', 80, 1800)
    result, _ = plan(rollout, 'req_x', [move(dx=0.01)])
    check(result['rejection'] == 'not_started', 'before robot_start: not_started')

    io, rollout, request = started()
    result, _ = plan(rollout, 'req_wrong', [move(dx=0.01)])
    check(result['rejection'] == 'stale_request_id' and io.sent == [],
          'wrong request_id: stale_request_id, nothing sent')

    io.base = odom(vx=0.05)     # base moving: the guard refuses the first step
    result, _ = plan(rollout, request, [move(dx=0.01), move(dx=0.02)])
    check(result['status'] == 'plan_interrupted' and io.sent == []
          and not result['action_executed']
          and result['result']['stop_reason'] == 'observation_rejected'
          and result['next_call']['tool'] == 'robot_observe',
          'guard rejects step 1: nothing sent, observe next',
          str(result['result']['stop_reason']))


def check_completed():
    io, rollout, request = started()
    steps = [move(dx=0.03), move(dx=0.03, dz=-0.03), gripper(1.0)]
    result, reply = plan(rollout, request, steps)
    summary = result['result']
    check(result['status'] == 'plan_completed' and result['action_executed'],
          'three steps: plan_completed', result['reason'])
    check(sent_tools(io) == ['robot_move', 'robot_move', 'robot_gripper'],
          'commands sent in plan order', str(sent_tools(io)))
    check(io.pose['right']['x'] == START['right'][0] + 0.03
          and math.isclose(io.pose['right']['z'], START['right'][2] - 0.03)
          and io.grip['right'] == 1.0, 'fake robot ended at the last targets')
    check([s['packet']['status'] for s in summary['steps']] == ['arrived'] * 3
          and summary['remaining_steps'] == [] and summary['stop_reason'] is None,
          'summary: three arrived steps, nothing remaining')
    used = [request] + [s['packet']['action_context']['request_id'] for s in summary['steps']]
    check(len(set(used)) == 4, 'each step ran on the request_id of the previous observation')
    check(summary['source_request_id'] == request, 'summary keeps the source request_id')
    check(result['observation'] is not None and len(reply['contentItems']) == 1 + 3,
          'reply has the final observation and three images')
    check(result['action_context']['movement_allowed']
          and result['action_context']['request_id'] == used[-1],
          'reply hands over the last step\'s request_id')
    check(result['progress']['actions_executed'] == 3, 'each step counts as one action')

    state = plan_state(rollout, result)
    index = result['progress']['call_index']
    step_files = sorted(p.name for p in rollout.private_dir.glob(f'plan_{index:04d}_step_*.json'))
    check(state['status'] == 'plan_completed', 'plan state journal: plan_completed')
    check(step_files == [f'plan_{index:04d}_step_{i:02d}.json' for i in (1, 2, 3)],
          'one private record per step', str(step_files))
    record = json.loads((rollout.private_dir / step_files[0]).read_text())
    check(record['arguments']['reason'] == 'move step', 'step record keeps the step reason')

    history = [json.loads(line) for line in rollout.history_path.read_text().splitlines()]
    check([h['tool'] for h in history] == ['robot_start', PLAN_TOOL],
          'history has the plan call, not each step', str([h['tool'] for h in history]))

    follow = packet(rollout.handle_tool_call('robot_move', {
        'request_id': result['action_context']['request_id'], 'reason': 'after plan',
        **move(dx=0.03)['args']}))
    check(follow['status'] == 'arrived', 'a single action works with the returned request_id',
          follow['reason'])


def check_open_gripper_then_lift():
    io, rollout, request = started()
    result, _ = plan(rollout, request, [gripper(0.0), move(dy=0.02), lift(-0.2)])
    check(result['status'] == 'plan_completed'
          and sent_tools(io) == ['robot_gripper', 'robot_move', 'robot_lift']
          and io.lift == -0.2,
          'opening gripper mid-plan and lift as the last step', result['reason'])


def check_stopped():
    io, rollout, request = started(['arrived', 'stopped'])
    result, _ = plan(rollout, request, [move(dx=0.02), move(dx=0.04), move(dx=0.04, dz=0.02)])
    summary = result['result']
    check(result['status'] == 'plan_interrupted' and summary['stop_reason'] == 'stopped',
          'step 2 stopped: plan_interrupted', str(summary['stop_reason']))
    check(len(io.sent) == 2 and len(summary['remaining_steps']) == 1,
          'step 3 not sent and reported as remaining')
    check(result['action_executed'] and result['observation'] is not None
          and result['action_context']['movement_allowed'],
          'reply has the observation after the stop')


def check_rejected_after_first_step():
    """Step 2 is valid alone but more than max_step_m from where step 1 ended."""
    io, rollout, request = started()
    result, reply = plan(rollout, request, [move(dx=0.04), move(dx=-0.04)])
    summary = result['result']
    check(summary['stop_reason'] == 'invalid_action' and len(io.sent) == 1,
          'step 2 out of reach from step 1: invalid_action, only step 1 sent',
          str(summary['stop_reason']))
    check(result['action_executed'], 'action_executed is true because step 1 moved')
    # The guard still calls step 1's observation current, so the model must see it.
    check(result['observation'] is not None and len(reply['contentItems']) == 1 + 3,
          'reply carries step 1\'s observation, which the guard still treats as current',
          f'observation={result["observation"] is not None}, '
          f'images={len(reply["contentItems"]) - 1}, '
          f'movement_allowed={result["action_context"]["movement_allowed"]}')


def check_without_observation():
    io, rollout, request = started(['no_observation'])
    result, _ = plan(rollout, request, [move(dx=0.02), move(dx=0.04)])
    check(result['result']['stop_reason'] == 'arrived_without_observation' and len(io.sent) == 1,
          'post-action snapshot fails: stop after step 1',
          str(result['result']['stop_reason']))
    check(result['observation'] is None and not result['action_context']['movement_allowed']
          and result['next_call']['tool'] == 'robot_observe',
          'no observation: movement blocked, observe next')


def check_uncertain():
    for outcome in ('timeout', 'raise'):
        io, rollout, request = started(['arrived', outcome])
        try:
            plan(rollout, request, [move(dx=0.02), move(dx=0.04), move(dx=0.04, dz=0.02)])
            raised = False
        except UncertainExecution:
            raised = True
        check(raised and rollout.outcome()['finish_reason'] == 'uncertain_execution',
              f'step 2 {outcome}: UncertainExecution ends the run',
              str(rollout.outcome()['finish_reason']))
        check(len(io.sent) == 2, f'step 2 {outcome}: step 3 not sent')
        state = json.loads(sorted(rollout.private_dir.glob('plan_*_state.json'))[-1].read_text())
        check(state['status'] == 'plan_uncertain' and state['running_step'] == 2,
              f'step 2 {outcome}: journal says plan_uncertain at step 2')
        after = packet(rollout.handle_tool_call('robot_observe', {}))
        check(after['rejection'] == 'episode_finished', f'step 2 {outcome}: nothing runs after')


def check_budget():
    io, rollout, request = started(max_actions=2)
    result, _ = plan(rollout, request, [move(dx=0.02), move(dx=0.04), move(dx=0.04, dz=0.02)])
    check(result['result']['stop_reason'] == 'budget_actions' and len(io.sent) == 2,
          'max_actions 2: plan stops after two steps', str(result['result']['stop_reason']))
    check(rollout.outcome()['finish_reason'] == 'budget_actions', 'run ends on budget_actions')


def main():
    check_spec()
    check_invalid_plans()
    check_preconditions()
    check_completed()
    check_open_gripper_then_lift()
    check_stopped()
    check_rejected_after_first_step()
    check_without_observation()
    check_uncertain()
    check_budget()
    print(f'run records: {RUN_DIR}')
    print('PASS' if not failures else f'FAIL: {", ".join(failures)}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
