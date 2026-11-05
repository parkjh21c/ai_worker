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

"""Build a run viewer page from the ffw_codex viewer template

Runs keep the ffw_codex layout, so the same viewer replays them.
Without --run the page is empty: follow runs/<id> through a local web server,
or open a run folder. With --run it embeds that run as a single-file replay.
"""

import argparse
import base64
import json
from pathlib import Path
import re

from robot_layer import ROBOT_LAYER_DIR


HERE = Path(__file__).resolve().parent
VIEWER_DIR = ROBOT_LAYER_DIR / 'viewer'
# The files the viewer's folder loader reads
TEXT_FILES = re.compile(
    r'^(run|host_result|result)\.json$|^public/history\.jsonl$|^rollout/[^/]+\.json$'
    r'|^calls/[^/]+_request\.json$|^public/observations/[^/]+/observation\.json$')
IMAGE_FILES = re.compile(r'^public/observations/[^/]+/[^/]+\.jpg$')


def run_files(run_dir):
    """Collect a run's viewer files: text as strings, JPEG frames as data URLs"""
    run_dir = Path(run_dir)
    if not (run_dir / 'run.json').is_file():
        raise FileNotFoundError(f'{run_dir} has no run.json')
    files = {}
    for path in sorted(run_dir.rglob('*')):
        key = path.relative_to(run_dir).as_posix()
        if TEXT_FILES.match(key):
            files[key] = path.read_text(encoding='utf-8')
        elif IMAGE_FILES.match(key):
            encoded = base64.b64encode(path.read_bytes()).decode('ascii')
            files[key] = 'data:image/jpeg;base64,' + encoded
    return files


def build(files):
    """Fill the template; </ is escaped so data cannot close the script element"""
    page = (VIEWER_DIR / 'template.html').read_text(encoding='utf-8')
    for marker, name in (('__LIVE_SCRIPT__', 'live.js'), ('__FOLLOW_SCRIPT__', 'follow.js')):
        page = page.replace(marker, (VIEWER_DIR / name).read_text(encoding='utf-8'), 1)
    page = page.replace('ffw_codex/runs', f'{HERE.name}/runs')  # follow-bar hint
    data = json.dumps(files, ensure_ascii=False).replace('</', '<\\/')
    return page.replace('__RUN_FILES__', data, 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--run', help='run folder to embed, e.g. runs/first_try')
    parser.add_argument('--out', default=None, help='output page (default: run_viewer.html)')
    args = parser.parse_args()
    files = run_files(args.run) if args.run else {}
    out = Path(args.out) if args.out else HERE / 'run_viewer.html'
    out.write_text(build(files), encoding='utf-8')
    print(f'{out}  ({len(files)} run files)')


if __name__ == '__main__':
    main()
