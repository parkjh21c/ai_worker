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

"""Serve the rollout's robot tools to Claude Code as an in-process MCP server

The rollout builds Codex dynamic-tool specs and replies. This module rewords
the specs for Claude and turns each reply into MCP content: the JSON packet as
a text block and each camera frame as an image block.
"""

import re

from claude_agent_sdk import create_sdk_mcp_server, tool, ToolAnnotations

import robot_layer  # noqa: F401
from rollout import RETURN_FORMAT


SERVER = 'robot'
PREFIX = f'mcp__{SERVER}__'
# Keep observation results inline instead of a file preview
MAX_RESULT_CHARS = 5_000_000
CLAUDE_RETURN_FORMAT = (
    ' Returns a text block with a JSON packet; action_executed says whether a '
    'robot command ran. If packet.observation is not null, the camera images '
    'follow as image blocks in the order head, wrist_left, wrist_right. The '
    'same files are listed in observation.images and can be opened with Read.'
)
# The plan tool states its return format in its own words
PLAN_RETURN_FORMAT = re.compile(r' The first response line is a JSON packet\b.*$', re.S)
CODEX_ONLY = ('image()', 'data:', 'view_image')


def claude_specs(specs):
    """Reword rollout tool specs for Claude"""
    result = []
    for spec in specs:
        text = spec['description']
        if text.endswith(RETURN_FORMAT):
            text = text[:-len(RETURN_FORMAT)] + CLAUDE_RETURN_FORMAT
        else:
            text, count = PLAN_RETURN_FORMAT.subn(CLAUDE_RETURN_FORMAT, text)
            if count != 1:
                raise ValueError(f'{spec["name"]}: return format not found in the description')
        leftover = [word for word in CODEX_ONLY if word in text]
        if leftover:
            raise ValueError(
                f'{spec["name"]}: description still mentions {leftover}; update robot_server.py')
        result.append({
            'name': spec['name'],
            'description': text,
            'input_schema': spec['inputSchema'],
        })
    return result


def mcp_result(reply):
    """Convert a rollout reply into an MCP tool result"""
    content = []
    for item in reply['contentItems']:
        kind = item.get('type')
        if kind == 'inputText':
            content.append({'type': 'text', 'text': item['text']})
        elif kind == 'inputImage':
            header, _, data = item['imageUrl'].partition(',')
            if not header.startswith('data:') or not header.endswith(';base64'):
                raise ValueError('image must be a base64 data URL')
            content.append({
                'type': 'image',
                'data': data,
                'mimeType': header[len('data:'):-len(';base64')],
            })
        else:
            raise ValueError(f'unsupported content item {kind!r}')
    return {'content': content, 'is_error': not reply['success']}


def build_server(specs, call):
    """Build the MCP server; call(name, arguments) returns an MCP result"""
    tools = []
    for spec in specs:
        async def handler(arguments, name=spec['name']):
            return await call(name, arguments)

        tools.append(tool(
            spec['name'],
            spec['description'],
            spec['input_schema'],
            annotations=ToolAnnotations(maxResultSizeChars=MAX_RESULT_CHARS),
        )(handler))
    return create_sdk_mcp_server(SERVER, tools=tools)
