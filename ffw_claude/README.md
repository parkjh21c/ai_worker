# ffw_claude

Claude Code drives the AI Worker, the way `ffw_codex` does with Codex.
One episode is one task: Claude sees the three cameras and the robot state,
acts only through checked `robot_*` tools, and every call is recorded.

The robot side is shared: ROS I/O, the observation guard, the rollout and the
plan executor are imported from `../ffw_codex` (`robot_layer.py`), so both
agents move the robot through the same safety checks. Only the agent host
differs.

| | ffw_codex | ffw_claude |
|---|---|---|
| Agent runtime | `codex app-server` over stdio JSON-RPC | Claude Code CLI through the Claude Agent SDK (`claude-agent-sdk`) |
| Robot tools | Codex dynamic tools | In-process SDK MCP server `robot` (`mcp__robot__robot_move`, ...) |
| Camera frames | `data:` URL lines passed to `image()` | Image blocks in the tool result |
| Workspace instructions | `agent/AGENTS.md` | `agent/CLAUDE.md` |
| Task rules | developer instructions | appended to the Claude Code system prompt |
| File access | permission profile | permission rules (`dontAsk`) + PreToolUse hook |
| Shell | Codex sandbox, no network | Bash in the Claude Code sandbox (bubblewrap), no network |
| Credentials | `CODEX_HOME` | `CLAUDE_CONFIG_DIR` (`claude.config_dir`) |
| Host | `codex_host.py` | `claude_host.py` |

## Setup

On the robot PC (or inside the AI Worker container):

```bash
pip install claude-agent-sdk
```

```bash
apt-get install bubblewrap socat
```

The SDK bundles the Claude Code CLI. Log in once into a config folder used
only for robot runs:

```bash
CLAUDE_BIN=$(python3 -c 'import claude_agent_sdk, pathlib; print(pathlib.Path(claude_agent_sdk.__file__).parent / "_bundled" / "claude")')
```

```bash
CLAUDE_CONFIG_DIR=/root/.claude_robot $CLAUDE_BIN auth login
```

```bash
CLAUDE_CONFIG_DIR=/root/.claude_robot $CLAUDE_BIN auth status
```

To use an API key instead, export `ANTHROPIC_API_KEY` and set
`claude.pass_env: [ANTHROPIC_API_KEY]` in the config.

## Run

Check the setup without ROS; a fake robot answers and nothing moves:

```bash
python3 run_claude.py --dry-run --task "look at the cameras, then call task_complete" --run-id dry
```

Physical robot (needs SG2 bringup and Cyclo, like `run_codex.py`):

```bash
python3 run_claude.py --task "pick up the black cube" --run-id first_try
```

Gazebo:

```bash
python3 run_claude_gazebo.py --task "pick up the blue block" --run-id sim_try
```

Ctrl-C ends the episode after the active robot tool finishes. After a run,
fill in `verdict` and `verdict_note` in `runs/<id>/result.json`.

## Config

`config.yaml` and `config_gazebo.yaml` keep the `ffw_codex` robot sections
unchanged. The agent sections:

| Key | Meaning |
|---|---|
| `model.id`, `model.effort` | Claude model and effort (`low` to `max`) |
| `claude.bin` | Claude Code CLI; `null` uses the one bundled with the SDK |
| `claude.config_dir` | `CLAUDE_CONFIG_DIR`: login and session transcripts |
| `claude.pass_env` | Host variables passed to Claude Code, e.g. `ANTHROPIC_API_KEY` |
| `claude.bash` | Sandboxed Bash for `python3` crops and math; `false` leaves Read/Glob/Grep/Write/Edit |
| `claude.sandbox.weaker_nested` | `true` in a container where bubblewrap cannot mount `/proc` |
| `claude.sandbox.deny_read` | Paths hidden from Bash; the run's `agent/` and `public/` stay readable |
| `episode.max_total_tokens`, `episode.max_budget_usd` | Token and cost limits |

## What Claude can touch

- Robot: only the `robot_*` tools. Every action needs the `request_id` of the
  latest observation, and the guard checks the measured state before sending.
- Files: read `agent/` and `public/` of this run; write `agent/notes/` and
  `agent/scratch/`. Permission rules, a host-side PreToolUse hook and the Bash
  sandbox all enforce this.
- Bash: sandboxed, no network, credentials removed. Unsandboxed retries are
  disabled, and Claude Code exits if the sandbox is unavailable.
- Nothing else: no web tools, subagents, skills, auto memory, or MCP servers
  other than `robot`. The run refuses to start when another `CLAUDE.md`,
  `AGENTS.md` or `.claude/` settings file would be loaded.

The CLI starts through `claude_cli_env.py`, which replaces the inherited
environment (ROS variables, keys) with an allowlist.

## Run folder

`runs/<id>/` has the `ffw_codex` layout, so the same viewer replays it.

| Path | Contents |
|---|---|
| `run.json`, `result.json` | Settings, input hashes, summary and verdict |
| `host.json`, `host_result.json` | Claude Code session, settings, end reason, tokens, cost |
| `calls/` | Each robot call before and after it ran |
| `rollout/`, `public/` | Rollout records, observations and `history.jsonl` |
| `agent/` | The workspace Claude worked in, with its notes |
| `agent_events.jsonl`, `turns.jsonl` | Claude's text, tool calls and results; turns |
| `policy_denials.jsonl` | Tool calls the host refused |
| `claude_session.jsonl` | The Claude Code transcript |

Build the viewer page and serve this folder with any local web server, then
follow a run by its id:

```bash
python3 build_viewer.py
```

To share one run as a single file:

```bash
python3 build_viewer.py --run runs/first_try --out first_try.html
```
