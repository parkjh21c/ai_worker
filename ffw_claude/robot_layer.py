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

"""Share the robot side with ffw_codex

ROS I/O, the observation guard, the rollout and the plan executor do not
depend on the model. Importing this module makes them importable from the
sibling ffw_codex folder, so Codex and Claude drive the robot through the
same checked code. Set FFW_ROBOT_LAYER_DIR to use another copy.
"""

import os
from pathlib import Path
import sys


ROBOT_LAYER_DIR = Path(
    os.environ.get('FFW_ROBOT_LAYER_DIR')
    or Path(__file__).resolve().parent.parent / 'ffw_codex'
).resolve()
REQUIRED = (
    'rollout.py',
    'plan_executor.py',
    'codex_host.py',  # rollout imports UncertainExecution, utc_now, write_json
    'transport.py',
    'robot_tools.py',
    'observation_guard.py',
    'transforms.py',
)


def use_robot_layer():
    """Add the robot layer to sys.path after this folder"""
    missing = [name for name in REQUIRED if not (ROBOT_LAYER_DIR / name).is_file()]
    if missing:
        raise ImportError(f'robot layer {ROBOT_LAYER_DIR} is missing {missing}')
    path = str(ROBOT_LAYER_DIR)
    if path not in sys.path:
        # Append so modules in this folder keep priority
        sys.path.append(path)


use_robot_layer()
