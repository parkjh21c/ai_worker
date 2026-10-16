"""
grasp probe: move one hand by hand and close its gripper on an object.

Calls RobotTools directly: no guard, no Codex, no model. A timeout does not
end anything here. Watch Gazebo while using it.

    # In poc_codex/, with the ROS environment loaded.
    python3 probe_grasp_gazebo.py [--arm right]

Commands:
    s               show hand pose and gripper
    m dx dy dz [dyaw]   move by dx dy dz meters in base_link; optionally add dyaw radians to yaw
    g value         gripper 0 open .. 1 closed; prints status and where it stopped
    w [seconds]     print the gripper angle every 0.5 s (default 5 s)
    q               quit
"""

import argparse
from pathlib import Path
import time

import yaml


HERE = Path(__file__).resolve().parent


def show(tools, arm):
    state = tools.get_state()['data']['arms'][arm]
    ee, gripper = state['ee'], state['gripper']
    print(f'xyz {ee["x"]:.4f} {ee["y"]:.4f} {ee["z"]:.4f}  '
          f'rpy {ee["roll"]:.3f} {ee["pitch"]:.3f} {ee["yaw"]:.3f}  '
          f'gripper {gripper["value"]:.3f} ({gripper["position_rad"]:.4f} rad)', flush=True)
    return ee


def move(tools, arm, dx, dy, dz, dyaw=0.0):
    ee = show(tools, arm)
    result = tools.move(arm, ee['x'] + dx, ee['y'] + dy, ee['z'] + dz,
                        ee['roll'], ee['pitch'], ee['yaw'] + dyaw)
    arrival = result['data'].get('arrival') or {}
    print(f'{result["status"]}: {result["reason"]}  '
          f'error {arrival.get("position_error_m")} m, '
          f'{arrival.get("orientation_error_deg")} deg', flush=True)
    show(tools, arm)


def gripper(tools, arm, value):
    started = time.monotonic()
    result = tools.set_gripper(arm, value)
    data = result['data']
    arrival = data.get('arrival') or {}
    stopped = (arrival.get('gripper') or {}).get('position_rad')
    print(f'{result["status"]}: {result["reason"]}  target {data.get("target_rad")} rad, '
          f'stopped at {stopped} rad, error {arrival.get("error_rad")} rad, '
          f'{time.monotonic() - started:.1f} s', flush=True)


def watch(tools, arm, seconds):
    for _ in range(max(1, int(seconds / 0.5))):
        show(tools, arm)
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--config', default=str(HERE / 'config_gazebo.yaml'))
    parser.add_argument('--arm', default='right')
    args = parser.parse_args()

    import rclpy
    from robot_io_gazebo import RobotIO
    from robot_tools import RobotTools

    with open(args.config, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    rclpy.init()
    io = RobotIO(cfg)
    try:
        io.start()
        io.wait_ready()
        tools = RobotTools(io, cfg)
        show(tools, args.arm)
        while True:
            parts = input('> ').split()
            if not parts:
                continue
            try:
                command, values = parts[0], [float(value) for value in parts[1:]]
            except ValueError:
                print('commands: s | m dx dy dz [dyaw] | g value | w [seconds] | q')
                continue
            if command == 'q':
                break
            elif command == 's':
                show(tools, args.arm)
            elif command == 'm' and len(values) in (3, 4):
                move(tools, args.arm, *values)
            elif command == 'g' and len(values) == 1:
                gripper(tools, args.arm, values[0])
            elif command == 'w':
                watch(tools, args.arm, values[0] if values else 5.0)
            else:
                print('commands: s | m dx dy dz [dyaw] | g value | w [seconds] | q')
    finally:
        io.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
