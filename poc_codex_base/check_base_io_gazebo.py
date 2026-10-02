"""
Check the mobile-base part of robot_io_gazebo against a running Gazebo.

Calls RobotIO directly: no guard, no Codex, no model. Without --move or --tools
nothing is published. --move runs one send_base profile. --tools runs
RobotTools.move_base (validation, posture, rest check, new observation),
--times in a row. --tools with --rollout goes through RobotRollout and
ObservationGuard like a model call (robot_start, then robot_base with the
latest request_id) and prints each packet; records go to runs/check_base_*.
Watch Gazebo.

    # In poc_codex_base/, with the ROS environment loaded and Gazebo running.
    python3 check_base_io_gazebo.py
    python3 check_base_io_gazebo.py --move 0.1 0 0 2.0     # forward ~0.2 m
    python3 check_base_io_gazebo.py --move 0 0 0.2 2.0     # turn left ~0.4 rad
    python3 check_base_io_gazebo.py --tools 0.1 0 0 2.0    # same through move_base
    python3 check_base_io_gazebo.py --tools 0.05 0 0 2.0   # rejected: below deadband
    python3 check_base_io_gazebo.py --tools 0.1 0 0 1.0 --rollout --times 2
"""

import argparse
import math
from pathlib import Path
import sys

import yaml


HERE = Path(__file__).resolve().parent
CAMERAS = ('head', 'wrist_left', 'wrist_right')

failures = []


def check(ok, label, detail=''):
    print(f'[{"ok" if ok else "FAIL"}] {label}{"  " + detail if detail else ""}', flush=True)
    if not ok:
        failures.append(label)


def pose(state):
    return (f'x {state["x"]:.4f} y {state["y"]:.4f} yaw {state["yaw"]:.4f}  '
            f'v {state["vx"]:.3f} {state["vy"]:.3f} {state["wz"]:.3f}')


def check_profile(RobotIO):
    """Ramp shape without ROS: floor at both ends, full speed in the middle."""
    scale = RobotIO._profile_scale
    check(abs(scale(0.0, 2.0, 0.3, 0.6) - 0.6) < 1e-9, 'profile starts at the floor')
    check(scale(1.0, 2.0, 0.3, 0.6) == 1.0, 'profile holds full speed')
    check(abs(scale(1.999, 2.0, 0.3, 0.6) - 0.6) < 0.01, 'profile ends near the floor')
    check(scale(0.25, 0.5, 0.3, 0.6) <= 1.0, 'short profile stays within [floor, 1]')


def check_static(io, cfg):
    state = io.get_base_state()
    check(state['frame'] == 'odom', 'odom frame', f'{state["frame"]} -> {state["child_frame"]}')
    check(state['age_s'] <= cfg['base']['odom_max_age_s'], 'odom is fresh', f'{state["age_s"]:.3f} s')
    print('  base', pose(state))

    check('base' in io.get_state(), 'get_state has base')

    tol = io.snapshot_tolerance_s
    matched = 0
    worst = 0.0
    for _ in range(5):
        try:
            snapshot = io.get_snapshot(CAMERAS)
        except TimeoutError as exc:
            print(f'  snapshot failed: {exc}')
            continue
        offset = abs(snapshot['base']['stamp_s'] - snapshot['stamp_s'])
        worst = max(worst, offset)
        matched += offset <= tol
    check(matched == 5, 'snapshot matches odom to the camera stamp',
          f'{matched}/5, worst offset {worst * 1000:.1f} ms (tolerance {tol * 1000:.1f} ms)')

    subscribers = io.cmd_vel_pub.get_subscription_count()
    publishers = io.count_publishers(io.cmd_vel_topic)
    print(f'  {io.cmd_vel_topic}: {subscribers} subscriber(s), {publishers} publisher(s) '
          '(1 expected: this node counts itself)')
    problem = io.base_command_problem()
    check(problem is None, 'cmd_vel ownership', problem or '')


def check_move(io, cfg, vx, vy, wz, duration_s):
    from transforms import base_delta

    base = cfg['base']
    for axis, value, deadband in (('vx', vx, base['deadband_mps']),
                                  ('vy', vy, base['deadband_mps']),
                                  ('wz', wz, base['deadband_rps'])):
        if value != 0.0 and abs(value) < deadband:
            print(f'  warning: {axis}={value} is below the controller deadband {deadband}; '
                  'the controller will zero it')

    result = io.send_base(vx, vy, wz, duration_s)
    print(f'  outcome {result["outcome"]}  {result["detail"] or ""}')
    print(f'  elapsed {result["elapsed_s"]:.2f} s, odom path {result["path_m"]:.4f} m, '
          f'{result["path_rad"]:.4f} rad, stop error {result["stop_error"]}')
    check(result['outcome'] == 'completed', 'profile completed')
    check(result['stop_error'] is None, 'zero published')

    rest = io.wait_until_base_stopped(result['stopped_at'])
    check(rest['rested'], 'rest confirmed', f'{rest["reason"]} after {rest["elapsed_s"]:.2f} s')
    if rest['state'] is None:
        return
    delta = base_delta(result['start'], rest['state'])
    print(f'  measured in start base_link: dx {delta["dx"]:.4f} m, dy {delta["dy"]:.4f} m, '
          f'dyaw {delta["dyaw"]:.4f} rad ({math.degrees(delta["dyaw"]):.1f} deg)')
    print(f'  commanded at full speed: {vx * duration_s:.3f} m, {vy * duration_s:.3f} m, '
          f'{wz * duration_s:.3f} rad (ramps make the measured value smaller)')


def check_tools(io, cfg, vx, vy, wz, duration_s, times):
    from robot_tools import RobotTools

    tools = RobotTools(io, cfg)
    for attempt in range(1, times + 1):
        before = io.get_base_state()
        result = tools.dispatch('move_base', {'vx': vx, 'vy': vy, 'wz': wz,
                                              'duration_s': duration_s})
        data = result['data']
        print(f'  #{attempt} {result["status"]} (command_sent {result["command_sent"]}): '
              f'{result["reason"]}')
        if result['status'] == 'rejected':
            print(f'  invalidate_observation {data.get("invalidate_observation")}')
            check(result['command_sent'] is False, f'#{attempt} rejected sent nothing')
            continue

        motion = data.get('motion') or {}
        print(f'  outcome {motion.get("outcome")}, elapsed {motion.get("elapsed_s", 0):.2f} s, '
              f'odom path {motion.get("path_m", 0):.4f} m, {motion.get("path_rad", 0):.4f} rad')
        measured = motion.get('odom_measured')
        if measured:
            print(f'  measured in start base_link: dx {measured["dx"]:.4f} m, '
                  f'dy {measured["dy"]:.4f} m, dyaw {measured["dyaw"]:.4f} rad '
                  f'({math.degrees(measured["dyaw"]):.1f} deg)')
        check(result['status'] in ('arrived', 'stopped'), f'#{attempt} arrived or stopped')
        check(motion.get('rest_confirmed') is True, f'#{attempt} rest confirmed')

        observation = data.get('post_observation')
        check(observation is not None and len(result['images']) == 3,
              f'#{attempt} new observation with three images',
              data.get('post_observation_error') or '')
        if observation is not None:
            base = observation['base']
            offset = abs(base['stamp_s'] - observation['stamp_s'])
            check(offset <= io.snapshot_tolerance_s, f'#{attempt} observation base matches its stamp',
                  f'{offset * 1000:.1f} ms')
            check(observation['stamp_s'] > before['stamp_s'], f'#{attempt} observation is after the motion')
            print('  observation base', pose(base))


def check_rollout(io, cfg, vx, vy, wz, duration_s, times):
    import json
    import time
    from codex_host import UncertainExecution
    from observation_guard_gazebo import ObservationGuard
    from robot_tools import RobotTools
    from rollout import RobotRollout

    run_dir = HERE / cfg['run']['output_root'] / time.strftime('check_base_%Y%m%d_%H%M%S')
    guard = ObservationGuard(RobotTools(io, cfg), cfg)
    rollout = RobotRollout(guard, cfg, run_dir, 'check_base_io_gazebo', 80, 1800)
    print(f'  records: {run_dir}')

    def call(tool, arguments):
        reply = rollout.handle_tool_call(tool, arguments)
        packet = json.loads(reply['contentItems'][0]['text'])
        images = sum(item['type'] == 'inputImage' for item in reply['contentItems'])
        shown = {key: packet.get(key) for key in
                 ('action_executed', 'status', 'rejection', 'reason', 'result',
                  'observation_note')}
        if packet.get('observation'):
            shown['observation.base'] = packet['observation'].get('base')
        shown['movement_allowed'] = packet['action_context']['movement_allowed']
        shown['next_call'] = packet['next_call']
        shown['images'] = images
        print(f'  {tool}: ' + json.dumps(shown, ensure_ascii=False))
        return packet

    packet = call('robot_start', {})
    check(packet['status'] == 'observed', 'robot_start observed')
    for attempt in range(1, times + 1):
        context = packet['action_context']
        if not context['movement_allowed']:
            packet = call('robot_observe', {})
            context = packet['action_context']
        try:
            packet = call('robot_base', {
                'request_id': context['request_id'], 'reason': f'check #{attempt}',
                'vx': vx, 'vy': vy, 'wz': wz, 'duration_s': duration_s})
        except UncertainExecution as exc:
            print(f'  #{attempt} UncertainExecution: {exc}')
            print(f'  outcome: {json.dumps(rollout.outcome(), ensure_ascii=False)}')
            check(False, f'#{attempt} ended the episode (expected only in failure scenarios)')
            return
        check(packet['status'] in ('arrived', 'stopped', 'rejected'),
              f'#{attempt} {packet["status"]}', packet.get('rejection') or '')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--config', default=str(HERE / 'config_gazebo.yaml'))
    parser.add_argument('--move', nargs=4, type=float, metavar=('VX', 'VY', 'WZ', 'DURATION_S'))
    parser.add_argument('--tools', nargs=4, type=float, metavar=('VX', 'VY', 'WZ', 'DURATION_S'))
    parser.add_argument('--times', type=int, default=1, help='--tools calls in a row')
    parser.add_argument('--rollout', action='store_true',
                        help='send --tools through RobotRollout and ObservationGuard')
    args = parser.parse_args()
    if args.move and args.tools:
        parser.error('use --move or --tools, not both')
    if args.rollout and not args.tools:
        parser.error('--rollout needs --tools')

    import rclpy
    from robot_io_gazebo import RobotIO

    check_profile(RobotIO)

    with open(args.config, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    rclpy.init()
    io = RobotIO(cfg)
    try:
        io.start()
        io.wait_ready()
        check(True, 'wait_ready (includes /odom)')
        check_static(io, cfg)
        if args.move:
            check_move(io, cfg, *args.move)
        if args.tools and args.rollout:
            check_rollout(io, cfg, *args.tools, max(1, args.times))
        elif args.tools:
            check_tools(io, cfg, *args.tools, max(1, args.times))
    finally:
        io.close()
        rclpy.try_shutdown()

    print('PASS' if not failures else f'FAIL: {", ".join(failures)}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
