"""Check that the robot contract's numeric promises match config and gripper code.

Run without ROS or a robot:
    python3 poc_codex/check_contract_config.py
"""

import ast
from pathlib import Path
import sys

import yaml


HERE = Path(__file__).resolve().parent
CONTRACT = HERE / 'agent_template' / 'context' / 'robot_contract.md'


def number(value):
    return f'{float(value):g}'


def method_default(path, method_name, argument_name):
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    robot_io = next(node for node in tree.body
                    if isinstance(node, ast.ClassDef) and node.name == 'RobotIO')
    method = next(node for node in robot_io.body
                  if isinstance(node, ast.FunctionDef) and node.name == method_name)
    arguments = method.args.args
    defaults = dict(zip((arg.arg for arg in arguments[-len(method.args.defaults):]),
                        method.args.defaults))
    return ast.literal_eval(defaults[argument_name])


def sections(text):
    result = {}
    section = None
    for line in text.splitlines():
        if line.startswith('## '):
            section = line[3:].split(':', 1)[0]
            result[section] = []
        elif section is not None:
            result[section].append(line)
    return {name: '\n'.join(lines) for name, lines in result.items()}


def main():
    cfg = yaml.safe_load((HERE / 'config.yaml').read_text(encoding='utf-8'))
    contract = sections(CONTRACT.read_text(encoding='utf-8'))
    limits, tolerance = cfg['limits'], cfg['tolerance']
    stall_s = tolerance['stall_s']
    arm_time_s = cfg['cyclo']['move_time_s']
    gripper_send_s = method_default(HERE / 'robot_io.py', 'send_gripper', 'move_time_s')
    gripper_wait_s = method_default(HERE / 'robot_io.py', 'wait_until_gripper_arrived',
                                    'move_time_s')

    checks = [
        ('Arms', 'maximum step', f'at most **{number(limits["max_step_m"])} m**'),
        ('Arms', 'maximum rotation', f'at most **{number(limits["max_rotation_rad"])} rad**'),
        ('Arms', 'lower reach', f'lower than {number(-cfg["reach"]["z_min"])} m below the arm base'),
        ('Arms', 'rest position and orientation',
         f'less than {number(tolerance["stall_position_m"])} m and '
         f'{number(tolerance["stall_orientation_deg"])}° for {number(stall_s)} s'),
        ('Arms', 'trajectory time', f'judged after its {number(arm_time_s)} s trajectory'),
        ('Arms', 'minimum duration', f'at least {number(arm_time_s + stall_s)} s'),
        ('Arms', 'arrival tolerance',
         f'within {number(tolerance["position_m"])} m and '
         f'{number(tolerance["orientation_deg"])}° of the target'),
        ('Arms', 'timeout',
         f'hand has come to rest within {number(tolerance["timeout_s"])} s'),
        ('Grippers', 'range',
         f'{number(limits["gripper_rad"][0])} to {number(limits["gripper_rad"][1])} rad'),
        ('Grippers', 'rest threshold',
         f'less than {number(tolerance["stall_gripper_rad"])} rad for {number(stall_s)} s'),
        ('Grippers', 'trajectory time',
         f'judged after its {number(gripper_send_s)} s trajectory'),
        ('Grippers', 'arrival tolerance',
         f'within {number(tolerance["gripper_rad"])} rad of the target'),
        ('Grippers', 'timeout',
         f'gripper has come to rest within {number(tolerance["timeout_s"])} s'),
        ('Head', 'head_joint1 range',
         'range {} to {} rad'.format(*map(number, limits['head_joint1_rad']))),
        ('Head', 'head_joint2 range',
         'range {} to {} rad'.format(*map(number, limits['head_joint2_rad']))),
        ('Lift', 'lift range',
         'range {} to {} m'.format(*map(number, limits['lift_joint_m']))),
        ('Observation and request_id', 'observation max age',
         f'seconds ({number(cfg["observation_guard"]["max_age_s"])})'),
    ]

    failures = []
    if gripper_send_s != gripper_wait_s:
        failures.append(
            f'robot_io.py gripper trajectory defaults differ: '
            f'send={gripper_send_s}, wait={gripper_wait_s}')
    for section, label, snippet in checks:
        if snippet not in contract.get(section, ''):
            failures.append(f'{section} / {label}: expected {snippet!r}')

    if failures:
        print('Robot contract mismatch:', file=sys.stderr)
        for failure in failures:
            print(f'- {failure}', file=sys.stderr)
        return 1
    print(f'Robot contract matches config.yaml and robot_io.py ({len(checks)} checks).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
