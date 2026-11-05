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

"""Run one robot episode with Claude Code

Requires ROS, SG2 bringup and Cyclo; --dry-run needs neither.
Example: python3 run_claude.py --task "your task" --run-id first_try
"""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys

import yaml

from claude_host import ClaudeHost, HostConfig, write_json
from claude_workspace import build_instructions, prepare_workspace, write_run_record
import robot_layer  # noqa: F401
from rollout import RobotRollout


HERE = Path(__file__).resolve().parent
DEFAULT_GAZEBO_TASK = 'pick up the blue block'
# Host-level failure reasons
HOST_FAILURES = frozenset({'host_error', 'transport_closed'})


def host_config(cfg, system, prompt, **overrides):
    """Build HostConfig from settings and optional overrides"""
    claude = cfg['claude']
    sandbox = claude.get('sandbox') or {}
    return HostConfig(
        system_prompt_append=system,
        initial_prompt=prompt,
        model=cfg['model']['id'],
        effort=cfg['model']['effort'],
        cli=claude.get('bin'),
        config_dir=Path(claude['config_dir']),
        home=claude.get('home', '/root'),
        pass_env=tuple(claude.get('pass_env') or ()),
        bash=bool(claude.get('bash', True)),
        sandbox_weaker_nested=bool(sandbox.get('weaker_nested', False)),
        sandbox_deny_read=tuple(sandbox.get('deny_read') or ()),
        token_limit=cfg['episode'].get('max_total_tokens'),
        budget_usd=cfg['episode'].get('max_budget_usd'),
        **overrides,
    )


def estimate_cost(tokens, prices):
    """Estimate token cost in USD when Claude Code reports none"""
    if not tokens or not prices:
        return None
    dollars = (
        tokens['input'] * prices['input']
        + tokens['cache_read'] * prices['cache_read']
        + tokens['cache_write'] * prices['cache_write']
        + tokens['output'] * prices['output']
    ) / 1e6
    return round(dollars, 3)


def summarize(record, outcome, cfg):
    """Build the episode summary for result.json"""
    rollout = outcome.get('rollout') or {}
    tokens = outcome.get('tokens') or {}
    cost = outcome.get('cost_usd')
    return {
        'run_id': record['run_id'],
        'task': record['task'],
        'agent': record['agent'],
        'model': record['model'],
        'effort': record['effort'],
        'end_reason': outcome.get('end_reason'),
        'end_detail': outcome.get('end_detail'),
        'finish_reason': rollout.get('finish_reason'),
        'claim': rollout.get('claim'),
        'actions_executed': rollout.get('actions_executed'),
        'observations': rollout.get('observations'),
        'robot_tool_calls': rollout.get('tool_calls'),
        'rejections': rollout.get('rejections'),
        'policy_denials': outcome.get('policy_denials'),
        'turns': outcome.get('turns'),
        'continuations': outcome.get('continuations'),
        'elapsed_s': outcome.get('elapsed_s'),
        'tokens': tokens,
        'cost_usd': round(cost, 3) if cost is not None else None,
        'cost_usd_estimate': estimate_cost(tokens, cfg['model'].get('price_per_million')),
        'verdict': None,        # Human verdict: "success" or "failure"
        'verdict_note': None,
    }


def run_episode(cfg, config_path, task, run_dir, guard, template_dir,
                client_factory=None, host_overrides=None):
    """Run one episode and save result.json"""
    run_dir = Path(run_dir).resolve()
    agent_dir = prepare_workspace(run_dir, cfg['claude']['config_dir'], template_dir)
    system, prompt = build_instructions(agent_dir, task)
    record = write_run_record(run_dir, run_dir.name, task, cfg, config_path, system, prompt)
    rollout = RobotRollout(
        guard,
        cfg,
        run_dir,
        task,
        max_actions=cfg['episode']['max_actions'],
        max_seconds=cfg['episode']['max_seconds'],
    )
    kwargs = {} if client_factory is None else {'client_factory': client_factory}
    host = ClaudeHost(
        host_config(cfg, system, prompt, **(host_overrides or {})),
        rollout,
        run_dir,
        **kwargs,
    )
    result = summarize(record, host.run(), cfg)
    write_json(run_dir / 'result.json', result)
    return result


def main(gazebo=False):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument(
        '--config',
        default=str(HERE / ('config_gazebo.yaml' if gazebo else 'config.yaml')),
    )
    parser.add_argument(
        '--task',
        required=not gazebo,
        default=DEFAULT_GAZEBO_TASK if gazebo else None,
    )
    parser.add_argument(
        '--run-id',
        default=None,
        help='folder name under runs/ (default: UTC time)',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='use a fake robot: checks Claude Code without ROS; nothing moves',
    )
    args = parser.parse_args()
    if args.dry_run and gazebo:
        parser.error('--dry-run uses the physical-robot guard; run it with run_claude.py')

    with open(args.config, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    run_id = args.run_id or datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    run_dir = HERE / cfg['run']['output_root'] / run_id
    template_dir = HERE / ('agent_template_gazebo' if gazebo else 'agent_template')

    if args.dry_run:
        from fake_robot import FakeRobotTools
        from observation_guard import ObservationGuard
        guard = ObservationGuard(FakeRobotTools(cfg), cfg)
        result = run_episode(cfg, args.config, args.task, run_dir, guard, template_dir)
    else:
        import rclpy
        from robot_tools import RobotTools
        if gazebo:
            from observation_guard_gazebo import ObservationGuard
            from robot_io_gazebo import RobotIO
        else:
            from observation_guard import ObservationGuard
            from robot_io import RobotIO

        rclpy.init()
        io = None
        try:
            io = RobotIO(cfg)
            io.start()
            io.wait_ready()
            guard = ObservationGuard(RobotTools(io, cfg), cfg)
            result = run_episode(cfg, args.config, args.task, run_dir, guard, template_dir)
        finally:
            if io is not None:
                io.close()
            rclpy.try_shutdown()

    claim = result['claim']
    cost = result['cost_usd'] if result['cost_usd'] is not None else result['cost_usd_estimate']
    print(
        f'\nend: {result["end_reason"]} / {result["finish_reason"]}, '
        f'actions {result["actions_executed"]}, {result["elapsed_s"]} s, '
        f'tokens {result["tokens"].get("total")}, ~${cost}'
    )
    print(f'claim: {claim["reason"] if claim else None}')
    print(f'record: {run_dir}  (fill in verdict in result.json)')
    return 1 if result['end_reason'] in HOST_FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
