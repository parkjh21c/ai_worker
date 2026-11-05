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

"""Start Claude Code with an allowlisted environment

The Agent SDK merges the whole host environment (ROS variables, API keys)
into the CLI environment, and Bash commands inherit it from there. The host
points ClaudeAgentOptions.cli_path at this script instead; it replaces the
environment with ROBOT_CLAUDE_ENV plus the variables the SDK itself sets,
then execs the real CLI named by ROBOT_CLAUDE_CLI.
"""

import json
import os
import sys


# Variables the SDK sets for its own protocol
SDK_KEYS = (
    'CLAUDE_CODE_ENTRYPOINT',
    'CLAUDE_AGENT_SDK_VERSION',
    'CLAUDE_AGENT_SDK_CLIENT_APP',
    'CLAUDE_CODE_SDK_READS_SESSION_STATE',
    'CLAUDE_CODE_ENABLE_SDK_FILE_CHECKPOINTING',
    'PWD',
)


def child_environment(environ):
    """Return the environment for Claude Code"""
    env = json.loads(environ['ROBOT_CLAUDE_ENV'])
    if not isinstance(env, dict) or not all(
        isinstance(key, str) and isinstance(value, str) for key, value in env.items()
    ):
        raise ValueError('ROBOT_CLAUDE_ENV must be a JSON object of strings')
    for key in SDK_KEYS:
        if key in environ:
            env[key] = environ[key]
    return env


def main():
    cli = os.environ.get('ROBOT_CLAUDE_CLI')
    if not cli or 'ROBOT_CLAUDE_ENV' not in os.environ:
        sys.exit('claude_cli_env.py: ROBOT_CLAUDE_CLI and ROBOT_CLAUDE_ENV must be set')
    os.execve(cli, [cli, *sys.argv[1:]], child_environment(os.environ))


if __name__ == '__main__':
    main()
