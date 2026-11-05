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

"""Run one robot episode through Claude Code (Claude Agent SDK)

The SDK starts the Claude Code CLI as a child process. The robot tools run in
this process as an SDK MCP server; each call executes in one worker thread, so
robot actions never overlap. Built-in tools are limited to files of this run:
permission rules, a PreToolUse hook and the Bash sandbox all apply.
"""

import asyncio
from collections import defaultdict, deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass, field, is_dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import sys
import threading
import time
import traceback

from claude_agent_sdk import (
    AssistantMessage,
    ClaudeAgentOptions,
    ClaudeSDKClient,
    HookMatcher,
    ResultMessage,
    SystemMessage,
    TextBlock,
    ToolResultBlock,
    ToolUseBlock,
    UserMessage,
)

from robot_server import build_server, claude_specs, mcp_result, PREFIX, SERVER


HERE = Path(__file__).resolve().parent
CLI_ENV_WRAPPER = HERE / 'claude_cli_env.py'
MAX_MESSAGE_BYTES = 64 * 1024 * 1024
READ_TOOLS = ('Read', 'Glob', 'Grep')
WRITE_TOOLS = ('Write', 'Edit')
# Credentials that sandboxed commands must never see
CREDENTIAL_ENV = (
    'ANTHROPIC_API_KEY',
    'ANTHROPIC_AUTH_TOKEN',
    'CLAUDE_CODE_OAUTH_TOKEN',
    'AWS_ACCESS_KEY_ID',
    'AWS_SECRET_ACCESS_KEY',
    'AWS_SESSION_TOKEN',
)
# API failures after which the same session can continue
RETRYABLE_STATUS = frozenset({408, 429, 500, 502, 503, 504, 529})
CONTINUE_TEXT = 'The episode is not finished. Continue from the latest observation. Next call: '
_CLOSED = object()


class TransportClosed(RuntimeError):
    """Claude Code exited or closed its message stream."""


class UncertainExecution(RuntimeError):
    """Robot command execution could not be confirmed."""


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, data):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2, default=str) + '\n', encoding='utf-8')
    temporary.replace(path)


def append_jsonl(path, record):
    with Path(path).open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(record, ensure_ascii=False, default=str) + '\n')


def digest(value):
    text = json.dumps(value, sort_keys=True, separators=(',', ':'), default=str)
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


TOKEN_KINDS = ('input', 'cache_read', 'cache_write', 'output')


def add_usage(totals, usage):
    """Add one Messages API usage object to token totals"""
    totals['input'] += usage.get('input_tokens') or 0
    totals['cache_read'] += usage.get('cache_read_input_tokens') or 0
    totals['cache_write'] += usage.get('cache_creation_input_tokens') or 0
    totals['output'] += usage.get('output_tokens') or 0


def failure(text):
    """Build a failed rollout-style reply for the model."""
    return {'success': False, 'contentItems': [{'type': 'inputText', 'text': text}]}


def redact(value):
    """Summarize image data for logging"""
    if is_dataclass(value) and not isinstance(value, type):
        value = {'type': type(value).__name__, **asdict(value)}
    if isinstance(value, str):
        if value.startswith('data:') and len(value) > 256:
            header, _, body = value.partition(',')
            return f'<{header} {len(body)} chars sha256:{digest(body)[:16]}>'
        return value
    if isinstance(value, dict):
        if value.get('type') == 'image':
            source = value.get('source') if isinstance(value.get('source'), dict) else value
            data = source.get('data')
            if isinstance(data, str) and len(data) > 256:
                kind = source.get('mimeType') or source.get('media_type')
                summary = f'<{kind} {len(data)} chars sha256:{digest(data)[:16]}>'
                return {'type': 'image', 'data': summary}
        return {key: redact(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [redact(item) for item in value]
    return value


def inside(path, roots):
    """Whether path, with symlinks resolved, is one of roots or below one"""
    resolved = Path(path).resolve()
    return any(resolved == root or root in resolved.parents for root in roots)


def static_prefix(pattern):
    """Return the part of a glob pattern before its first wildcard"""
    parts = []
    for part in Path(pattern).parts:
        if any(char in part for char in '*?[{'):
            break
        parts.append(part)
    return Path(*parts) if parts else Path('.')


def default_cli():
    """Return the CLI bundled with claude-agent-sdk, or claude on PATH"""
    import claude_agent_sdk
    bundled = Path(claude_agent_sdk.__file__).resolve().parent / '_bundled' / 'claude'
    if bundled.is_file():
        return str(bundled)
    found = shutil.which('claude')
    if found:
        return found
    raise FileNotFoundError(
        'Claude Code CLI not found; install claude-agent-sdk or set claude.bin')


def render_settings(run_dir, cfg):
    """Render the --settings layer: permission rules and the Bash sandbox"""
    run_dir = Path(run_dir).resolve()
    agent, public = run_dir / 'agent', run_dir / 'public'
    notes, scratch = agent / 'notes', agent / 'scratch'

    def rule(tool, directory):
        return f'{tool}(/{directory}/**)'  # //absolute/path in permission rules

    settings = {
        'autoMemoryEnabled': False,
        'disableBundledSkills': True,
        'permissions': {
            'defaultMode': 'dontAsk',
            'allow': [
                rule('Read', agent),
                rule('Read', public),
                rule('Edit', notes),
                rule('Edit', scratch),
            ],
            'deny': [
                rule('Edit', agent / 'context'),
                f'Edit(/{agent / "CLAUDE.md"})',
                f'Edit(/{agent / "workspace.json"})',
            ],
        },
    }
    if cfg.bash:
        deny_read = [*cfg.sandbox_deny_read, str(cfg.config_dir), str(run_dir.parent)]
        settings['sandbox'] = {
            'enabled': True,
            'failIfUnavailable': True,
            'autoAllowBashIfSandboxed': True,
            'allowUnsandboxedCommands': False,
            'enableWeakerNestedSandbox': cfg.sandbox_weaker_nested,
            'network': {'allowedDomains': []},
            'filesystem': {
                'denyRead': deny_read,
                'allowRead': [str(agent), str(public)],
                'allowWrite': [str(notes), str(scratch)],
                'denyWrite': [
                    str(agent / 'CLAUDE.md'),
                    str(agent / 'context'),
                    str(agent / 'workspace.json'),
                    str(agent / '.claude'),
                ],
            },
            'credentials': {
                'envVars': [{'name': name, 'mode': 'deny'} for name in CREDENTIAL_ENV],
            },
        }
    return settings


@dataclass
class HostConfig:
    system_prompt_append: str
    initial_prompt: str
    model: str = 'claude-opus-5-5'
    effort: str = 'medium'
    cli: str | None = None  # None: the CLI bundled with claude-agent-sdk
    config_dir: Path = Path('/root/.claude_robot')
    home: str = '/root'
    pass_env: tuple = ()
    bash: bool = True
    sandbox_weaker_nested: bool = False
    sandbox_deny_read: tuple = ('~/', '/root', '/home')
    request_timeout_s: float = 60.0
    poll_s: float = 1.0
    idle_timeout_s: float = 900.0
    finish_grace_s: float = 120.0
    interrupt_wait_s: float = 30.0
    max_continuations: int = 2
    network_retry_delays_s: tuple = (10.0,) * 20
    token_limit: int | None = None
    budget_usd: float | None = None
    extra_env: dict = field(default_factory=dict)


@dataclass
class ToolCall:
    index: int
    call_id: str | None
    tool: str
    arguments: object


class CallLedger:
    """Record each robot tool call before and after it runs"""

    def __init__(self, calls_dir):
        self.dir = Path(calls_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._count = 0

    def count(self):
        with self._lock:
            return self._count

    def register(self, call_id, tool, arguments):
        with self._lock:
            call = ToolCall(self._count, call_id, tool, arguments)
            self._count += 1
        return call

    def begin(self, call):
        write_json(self.dir / f'{call.index:04d}_request.json', {
            'index': call.index,
            'call_id': call.call_id,
            'tool': call.tool,
            'arguments': call.arguments,
            'digest': digest([call.tool, call.arguments]),
            'status': 'running',
            'started_at': utc_now(),
        })

    def complete(self, call, reply, status):
        write_json(self.dir / f'{call.index:04d}_result.json', {
            'index': call.index,
            'call_id': call.call_id,
            'status': status,
            'finished_at': utc_now(),
            'reply': redact(reply),
        })


class ClaudeHost:
    """Run one episode through Claude Code"""

    def __init__(self, cfg, rollout, run_dir, client_factory=ClaudeSDKClient):
        self.cfg = cfg
        self.rollout = rollout
        self.run_dir = Path(run_dir).resolve()
        self.agent_dir = self.run_dir / 'agent'
        self.public_dir = self.run_dir / 'public'
        self.client_factory = client_factory
        self.client = None
        self.reader = None
        self.loop = None
        self.inbox = None
        self.ledger = CallLedger(self.run_dir / 'calls')
        self.settings = render_settings(self.run_dir, cfg)
        self.specs = claude_specs(rollout.tool_specs())
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='robot-tool')
        self.active = set()  # robot call futures not yet finished
        self.tool_use_ids = defaultdict(deque)
        self.init = None
        self.session_id = None
        self.turn_active = False
        self.turns = 0
        self.continuations = 0
        self.network_continues = 0
        self.retry_at = None
        self.last_activity = time.monotonic()
        self.finished_at = None
        self.interrupt_deadline = None
        self.end_reason = None
        self.end_detail = None
        self.turn_tokens = dict.fromkeys(TOKEN_KINDS, 0)  # completed turns
        self.message_usage = {}  # current turn: message_id -> usage of that API response
        self.model_usage = None
        self.cost_usd = None
        self.denials = 0
        self.errors = []
        self.started = time.monotonic()

    # Lifecycle

    def run(self):
        """Run the episode and return its outcome."""
        return asyncio.run(self._main())

    async def _main(self):
        self.loop = asyncio.get_running_loop()
        self.inbox = asyncio.Queue()
        sigint = self._install_sigint()
        try:
            await self._start()
            await self._start_turn(self.cfg.initial_prompt)
            await self._loop()
        except TransportClosed as exc:
            self.turn_active = False
            self._end('transport_closed', str(exc))
        except Exception as exc:
            self.errors.append({'at': utc_now(), 'traceback': traceback.format_exc()})
            self._end('host_error', f'{type(exc).__name__}: {exc}')
        finally:
            outcome = await self._shutdown()
            self._restore_sigint(sigint)
        return outcome

    def _install_sigint(self):
        """Ctrl-C ends the episode after the active robot tool"""
        previous = signal.getsignal(signal.SIGINT)
        try:
            self.loop.add_signal_handler(signal.SIGINT, self._on_sigint)
        except (NotImplementedError, RuntimeError, ValueError):
            return False, None
        return True, previous

    def _restore_sigint(self, installed):
        done, previous = installed
        if not done:
            return
        self.loop.remove_signal_handler(signal.SIGINT)
        if callable(previous) or previous in (signal.SIG_DFL, signal.SIG_IGN):
            signal.signal(signal.SIGINT, previous)

    def _on_sigint(self):
        print('\nStopping: the active robot tool finishes, then the episode ends.',
              file=sys.stderr)
        self._end('interrupted_by_user')

    def _child_env(self):
        """Environment for Claude Code; Bash commands inherit it"""
        env = {
            'PATH': '/usr/local/bin:/usr/bin:/bin',
            'HOME': self.cfg.home,
            'LANG': 'C.UTF-8',
            'CLAUDE_CONFIG_DIR': str(self.cfg.config_dir),
            # No telemetry, error reports or auto-update
            'CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC': '1',
            'DISABLE_TELEMETRY': '1',
            'DISABLE_AUTOUPDATER': '1',
            # Each episode starts from its workspace only: no memory or bundled skills
            'CLAUDE_CODE_DISABLE_AUTO_MEMORY': '1',
            'CLAUDE_CODE_DISABLE_BUNDLED_SKILLS': '1',
            'CLAUDE_CODE_DISABLE_POLICY_SKILLS': '1',
            'CLAUDE_CODE_DISABLE_WORKFLOWS': '1',
            'CLAUDE_CODE_DISABLE_OFFICIAL_MARKETPLACE_AUTOINSTALL': '1',
            # Images count against this limit; three frames must fit
            'MAX_MCP_OUTPUT_TOKENS': '200000',
            **self.cfg.extra_env,
        }
        for key in self.cfg.pass_env:
            if os.environ.get(key):
                env[key] = os.environ[key]
        return env

    def _options(self):
        builtin = [*READ_TOOLS, *WRITE_TOOLS] + (['Bash'] if self.cfg.bash else [])
        server = build_server(self.specs, self._robot_call)
        return ClaudeAgentOptions(
            tools=builtin,
            allowed_tools=[PREFIX + spec['name'] for spec in self.specs],
            mcp_servers={SERVER: server},
            strict_mcp_config=True,
            system_prompt={
                'type': 'preset',
                'preset': 'claude_code',
                'append': self.cfg.system_prompt_append,
            },
            setting_sources=['project'],  # loads agent/CLAUDE.md
            settings=json.dumps(self.settings),
            permission_mode='dontAsk',
            model=self.cfg.model,
            effort=self.cfg.effort,
            cwd=str(self.agent_dir),
            cli_path=str(CLI_ENV_WRAPPER),
            env={
                'ROBOT_CLAUDE_CLI': self.cfg.cli or default_cli(),
                'ROBOT_CLAUDE_ENV': json.dumps(self._child_env()),
            },
            hooks={'PreToolUse': [
                HookMatcher(matcher=None, hooks=[self._pre_tool_use], on_failure='block'),
            ]},
            stderr=self._on_stderr,
            max_budget_usd=self.cfg.budget_usd,
            # Tool results come back with their camera frames; the SDK default is 1 MB
            max_buffer_size=MAX_MESSAGE_BYTES,
        )

    async def _start(self):
        if not self.agent_dir.is_dir():
            raise FileNotFoundError(f'agent workspace missing: {self.agent_dir}')
        if not os.access(CLI_ENV_WRAPPER, os.X_OK):
            raise PermissionError(f'{CLI_ENV_WRAPPER} must be executable: chmod +x it')
        if self.cfg.bash:
            missing = [name for name in ('bwrap', 'socat') if shutil.which(name) is None]
            if missing:
                raise RuntimeError(
                    f'the Bash sandbox needs {missing}: apt-get install bubblewrap socat, '
                    'or set claude.bash to false')
        options = self._options()
        self.client = self.client_factory(options)
        await asyncio.wait_for(self.client.connect(), self.cfg.request_timeout_s)
        self.reader = asyncio.create_task(self._read())

        def sha(text):
            return hashlib.sha256(text.encode('utf-8')).hexdigest()

        write_json(self.run_dir / 'host.json', {
            'started_at': utc_now(),
            'cli': options.env['ROBOT_CLAUDE_CLI'],
            'model': self.cfg.model,
            'effort': self.cfg.effort,
            'permission_mode': options.permission_mode,
            'builtin_tools': options.tools,
            'robot_tools': [spec['name'] for spec in self.specs],
            'settings': self.settings,
            'child_env_keys': sorted(self._child_env()),
            'system_prompt_append_sha256': sha(self.cfg.system_prompt_append),
            'initial_prompt_sha256': sha(self.cfg.initial_prompt),
        })

    async def _read(self):
        """Move SDK messages into the inbox"""
        try:
            async for message in self.client.receive_messages():
                await self.inbox.put(message)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            await self.inbox.put(exc)
        else:
            await self.inbox.put(_CLOSED)

    async def _start_turn(self, text):
        await asyncio.wait_for(self.client.query(text), self.cfg.request_timeout_s)
        self.turn_active = True
        self.turns += 1
        self.interrupt_deadline = None
        self.last_activity = time.monotonic()
        append_jsonl(self.run_dir / 'turns.jsonl', {
            'event': 'started',
            'index': self.turns,
            'at': utc_now(),
            'text': text,
        })

    def _finished(self):
        return self.end_reason is not None and not self.active and not self.turn_active

    async def _loop(self):
        while not self._finished():
            self.rollout.tick()
            await self._check_limits()
            if self._finished():
                break
            wait_s = 0.1 if self.active else self.cfg.poll_s
            try:
                message = await asyncio.wait_for(self.inbox.get(), wait_s)
            except asyncio.TimeoutError:
                continue
            await self._dispatch(message)

    def _end(self, reason, detail=None):
        """End the episode; later robot calls are rejected"""
        if self.end_reason is not None:
            return
        self.end_reason, self.end_detail = reason, detail
        self.retry_at = None
        if reason != 'rollout_finished':
            self.rollout.halt(reason)

    def _tokens(self):
        """Token totals: completed turns plus the live estimate of the current turn"""
        totals = dict(self.turn_tokens)
        for usage in self.message_usage.values():
            add_usage(totals, usage)
        return {**totals, 'total': sum(totals.values())}

    def _final_tokens(self):
        """Session totals from Claude Code's cumulative per-model usage"""
        if not self.model_usage:
            return self._tokens()
        totals = dict.fromkeys(TOKEN_KINDS, 0)
        for usage in self.model_usage.values():
            totals['input'] += usage.get('inputTokens') or 0
            totals['cache_read'] += usage.get('cacheReadInputTokens') or 0
            totals['cache_write'] += usage.get('cacheCreationInputTokens') or 0
            totals['output'] += usage.get('outputTokens') or 0
        return {**totals, 'total': sum(totals.values())}

    async def _check_limits(self):
        now = time.monotonic()
        if self.finished_at is None and self.rollout.finished():
            self.finished_at = now
        if self.end_reason is None:
            total = self._tokens()['total']
            if (
                self.finished_at is not None
                and (not self.turn_active or now - self.finished_at >= self.cfg.finish_grace_s)
            ):
                # Allow time to read the final result.
                self._end('rollout_finished')
            elif (
                self.turn_active
                and not self.active
                and now - self.last_activity >= self.cfg.idle_timeout_s
            ):
                self._end('idle_timeout')
            elif self.cfg.token_limit and total >= self.cfg.token_limit:
                self._end('token_limit', total)
        if (
            self.end_reason is None
            and not self.turn_active
            and self.retry_at is not None
            and now >= self.retry_at
        ):
            self.retry_at = None
            await self._start_turn(self._continue_text())
        if self.end_reason is not None and self.turn_active and not self.active:
            await self._interrupt(now)

    async def _interrupt(self, now):
        if self.interrupt_deadline is None:
            self.interrupt_deadline = now + self.cfg.interrupt_wait_s
            try:
                await asyncio.wait_for(self.client.interrupt(), self.cfg.request_timeout_s)
            except Exception as exc:
                self.errors.append({'at': utc_now(), 'interrupt': repr(exc)})
        elif now >= self.interrupt_deadline:
            self.errors.append({'at': utc_now(), 'interrupt': 'no result after interrupt'})
            self.turn_active = False

    def _continue_text(self):
        return CONTINUE_TEXT + json.dumps(self.rollout.next_call(), ensure_ascii=False)

    # Messages

    async def _dispatch(self, message):
        if message is _CLOSED:
            raise TransportClosed('Claude Code closed its output; see cli_stderr.log')
        if isinstance(message, BaseException):
            raise TransportClosed(f'{type(message).__name__}: {message}')
        append_jsonl(self.run_dir / 'sdk_messages.jsonl', {'at': utc_now(), **redact(message)})
        if isinstance(message, SystemMessage):
            if message.subtype == 'init' and self.init is None:
                self._on_init(message.data)
        elif isinstance(message, AssistantMessage):
            self._on_assistant(message)
        elif isinstance(message, UserMessage):
            self._on_user(message)
        elif isinstance(message, ResultMessage):
            await self._on_result(message)

    def _on_init(self, data):
        """Check the session Claude Code started"""
        self.init = data
        self.session_id = data.get('session_id')
        servers = {
            server.get('name'): server.get('status') for server in data.get('mcp_servers', [])
        }
        record = {
            'session_id': self.session_id,
            'claude_code_version': data.get('claude_code_version'),
            'model': data.get('model'),
            'permission_mode': data.get('permissionMode'),
            'cwd': data.get('cwd'),
            'tools': data.get('tools'),
            'mcp_servers': servers,
        }
        path = self.run_dir / 'host.json'
        write_json(path, {**json.loads(path.read_text(encoding='utf-8')), 'session': record})
        if servers.get(SERVER) != 'connected':
            raise RuntimeError(f'robot MCP server is {servers.get(SERVER)!r}, not connected')
        if data.get('model') != self.cfg.model:
            raise RuntimeError(f'session uses {data.get("model")}, expected {self.cfg.model}')
        if data.get('permissionMode') != 'dontAsk':
            raise RuntimeError(f'permission mode is {data.get("permissionMode")}, not dontAsk')

    def _on_assistant(self, message):
        if message.message_id and message.usage:
            self.message_usage[message.message_id] = message.usage
            write_json(self.run_dir / 'token_usage.json',
                       {'at': utc_now(), 'session_id': self.session_id, 'tokens': self._tokens()})
        if message.error:
            self.errors.append({'at': utc_now(), 'assistant_error': message.error})
        for block in message.content:
            if isinstance(block, TextBlock):
                event = {'kind': 'text', 'text': block.text}
            elif isinstance(block, ToolUseBlock):
                self.last_activity = time.monotonic()
                event = {'kind': 'tool_use', 'id': block.id, 'name': block.name,
                         'input': redact(block.input)}
            else:
                continue
            append_jsonl(self.run_dir / 'agent_events.jsonl', {'at': utc_now(), **event})

    def _on_user(self, message):
        if not isinstance(message.content, list):
            return
        for block in message.content:
            if isinstance(block, ToolResultBlock):
                self.last_activity = time.monotonic()
                append_jsonl(self.run_dir / 'agent_events.jsonl', {
                    'at': utc_now(),
                    'kind': 'tool_result',
                    'id': block.tool_use_id,
                    'is_error': block.is_error,
                    'content': redact(block.content),
                })

    async def _on_result(self, message):
        self.turn_active = False
        self.cost_usd = message.total_cost_usd  # cumulative over the session
        self.model_usage = message.model_usage or self.model_usage
        if message.usage:
            # Per-turn usage replaces the live estimate of this turn
            add_usage(self.turn_tokens, message.usage)
            self.message_usage.clear()
        append_jsonl(self.run_dir / 'turns.jsonl', {
            'event': 'completed',
            'index': self.turns,
            'at': utc_now(),
            'subtype': message.subtype,
            'is_error': message.is_error,
            'stop_reason': message.stop_reason,
            'terminal_reason': message.terminal_reason,
            'api_error_status': message.api_error_status,
            'errors': message.errors,
            'num_turns': message.num_turns,
            'total_cost_usd': message.total_cost_usd,
            'usage': message.usage,
            'result': (message.result or '')[:2000],
        })
        if self.end_reason is not None:
            return
        if self.rollout.finished():
            self._end('rollout_finished')
        elif message.subtype == 'success' and not message.is_error:
            if self.continuations >= self.cfg.max_continuations:
                self._end('model_ended')
            else:
                self.continuations += 1
                await self._start_turn(self._continue_text())
        elif (
            message.api_error_status in RETRYABLE_STATUS
            and self.network_continues < len(self.cfg.network_retry_delays_s)
        ):
            # Continue the same session without replaying actions.
            delay = self.cfg.network_retry_delays_s[self.network_continues]
            self.network_continues += 1
            self.retry_at = time.monotonic() + delay
        elif message.subtype == 'error_max_budget_usd':
            self._end('budget_usd', message.total_cost_usd)
        else:
            reason = (f'api_error_{message.api_error_status}' if message.api_error_status
                      else f'turn_{message.subtype}')
            self._end(reason, {'errors': message.errors, 'result': message.result})

    def _on_stderr(self, line):
        with (self.run_dir / 'cli_stderr.log').open('a', encoding='utf-8') as stream:
            stream.write(line.rstrip('\n') + '\n')

    # Tool permission policy

    def _policy(self, tool, tool_input):
        """Return why a built-in tool call is not allowed, or None"""
        readable = (self.agent_dir, self.public_dir)
        writable = (self.agent_dir / 'notes', self.agent_dir / 'scratch')
        if tool == 'Bash':
            return None if self.cfg.bash else 'Bash is not available in this episode.'
        if tool in READ_TOOLS:
            roots, keys = readable, ('file_path', 'path', 'notebook_path')
        elif tool in WRITE_TOOLS:
            roots, keys = writable, ('file_path',)
        else:
            return f'{tool} is not available in this episode.'
        paths = [tool_input[key] for key in keys if tool_input.get(key) is not None]
        if tool in ('Glob', 'Grep'):
            base = Path(tool_input.get('path') or self.agent_dir)
            for key in ('pattern', 'glob') if tool == 'Glob' else ('glob',):
                pattern = tool_input.get(key)
                if not isinstance(pattern, str):
                    continue
                if '..' in Path(pattern).parts:
                    return f'{tool} patterns may not contain "..".'
                if tool == 'Glob' and key == 'pattern':
                    paths.append(str(base / static_prefix(pattern)))
            if not paths:
                paths.append(str(self.agent_dir))
        if not paths:
            return f'{tool} needs a path.'
        for path in paths:
            if not isinstance(path, str):
                return f'{tool}: paths must be strings.'
            target = Path(path) if Path(path).is_absolute() else self.agent_dir / path
            if not inside(target, roots):
                allowed = ', '.join(str(root) for root in roots)
                return f'{tool} may only access {allowed}; {path} is outside.'
        return None

    async def _pre_tool_use(self, hook_input, tool_use_id, context):
        tool = hook_input.get('tool_name', '')
        tool_input = hook_input.get('tool_input') or {}
        if tool.startswith(PREFIX):
            # Match the coming MCP call to its tool_use_id
            call_id = tool_use_id or hook_input.get('tool_use_id')
            self.tool_use_ids[(tool[len(PREFIX):], digest(tool_input))].append(call_id)
            return {}
        reason = self._policy(tool, tool_input)
        if reason is None:
            return {}
        self.denials += 1
        append_jsonl(self.run_dir / 'policy_denials.jsonl', {
            'at': utc_now(), 'tool': tool, 'input': redact(tool_input), 'reason': reason,
        })
        return {'hookSpecificOutput': {
            'hookEventName': 'PreToolUse',
            'permissionDecision': 'deny',
            'permissionDecisionReason': reason,
        }}

    # Robot tools

    async def _robot_call(self, tool, arguments):
        """Serve one robot tool call from the MCP server"""
        waiting = self.tool_use_ids.get((tool, digest(arguments)))
        call = self.ledger.register(waiting.popleft() if waiting else None, tool, arguments)
        self.last_activity = time.monotonic()
        if self.end_reason is not None:
            reply = failure(f'Not executed: the episode ended ({self.end_reason}).')
            self.ledger.complete(call, reply, 'rejected')
            return mcp_result(reply)
        future = self.executor.submit(self._execute, call)
        self.active.add(future)
        future.add_done_callback(
            lambda done: self.loop.call_soon_threadsafe(self._on_done, done))
        # Shielded: a cancelled MCP request must not cancel a queued robot action
        reply = await asyncio.shield(asyncio.wrap_future(future))
        return mcp_result(reply)

    def _on_done(self, future):
        self.active.discard(future)
        self.last_activity = time.monotonic()

    def _execute(self, call):
        """Run one call in the robot worker thread and record it"""
        self.ledger.begin(call)  # recorded as running before anything executes
        status = 'completed'
        try:
            reply = self.rollout.handle_tool_call(call.tool, call.arguments)
            if not (
                isinstance(reply, dict)
                and isinstance(reply.get('success'), bool)
                and isinstance(reply.get('contentItems'), list)
            ):
                raise UncertainExecution(f'rollout returned {type(reply).__name__}, not a reply')
            mcp_result(reply)
        except Exception as exc:
            # The command may already have been sent.
            status = 'uncertain'
            reply = failure(f'Execution state of {call.tool} is uncertain; movement is '
                            'blocked for this episode.')
            self.loop.call_soon_threadsafe(
                self._end, 'uncertain_execution', f'{type(exc).__name__}: {exc}')
        self.ledger.complete(call, reply, status)
        return reply

    # Shutdown

    async def _shutdown(self):
        if self.end_reason is None:
            self._end('host_error', 'shutdown without an end reason')
        # Wait for the active robot tool.
        pending = list(self.active)
        if pending:
            await asyncio.gather(*(asyncio.wrap_future(future) for future in pending),
                                 return_exceptions=True)
        if self.client is not None:
            if self.turn_active:
                try:
                    await asyncio.wait_for(self.client.interrupt(), 5.0)
                except Exception as exc:
                    self.errors.append({'at': utc_now(), 'shutdown_interrupt': repr(exc)})
            try:
                await asyncio.wait_for(self.client.disconnect(), 30.0)
            except Exception as exc:
                self.errors.append({'at': utc_now(), 'disconnect': repr(exc)})
        if self.reader is not None:
            self.reader.cancel()
            await asyncio.gather(self.reader, return_exceptions=True)
        self.executor.shutdown(wait=True)
        try:
            rollout_outcome = self.rollout.outcome()
        except Exception as exc:
            rollout_outcome = {'error': f'{type(exc).__name__}: {exc}'}
        outcome = {
            'finished_at': utc_now(),
            'elapsed_s': round(time.monotonic() - self.started, 1),
            'end_reason': self.end_reason,
            'end_detail': self.end_detail,
            'session_id': self.session_id,
            'claude_code_version': (self.init or {}).get('claude_code_version'),
            'turns': self.turns,
            'continuations': self.continuations,
            'network_continues': self.network_continues,
            'tool_calls': self.ledger.count(),
            'policy_denials': self.denials,
            'tokens': self._final_tokens(),
            'model_usage': self.model_usage,
            'cost_usd': self.cost_usd,
            'session_log': self._copy_session_log(),
            'errors': self.errors[-20:],
            'rollout': rollout_outcome,
        }
        write_json(self.run_dir / 'host_result.json', outcome)
        return outcome

    def _copy_session_log(self):
        """Copy the Claude Code transcript of this session"""
        projects = Path(self.cfg.config_dir) / 'projects'
        if not self.session_id or not projects.is_dir():
            return None
        matches = sorted(projects.rglob(f'{self.session_id}.jsonl'))
        if not matches:
            return None
        target = self.run_dir / 'claude_session.jsonl'
        shutil.copyfile(matches[-1], target)
        return str(target)
