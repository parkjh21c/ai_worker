# poc_codex — Agent Control Loop

[HTML](control-loop-draft.html) · [PNG](control-loop-draft.png) · [Editable SVG](control-loop-draft.svg)

This English diagram follows the supplied reference: a task enters the model, the tool interface branches into observation and action tools, valid actions reach the robot, and measured observations return to the model. The dashed outer frame is labeled `AI Worker Container`. Observation, validation, and ROS control stay inside this boundary; the `Gazebo / Physical robot` block sits below it, outside the container. Command and feedback arrows cross the boundary.

The diagram is a native SVG with an embedded, standalone HTML copy. It uses a plain background, rounded boxes, grouped tools, and a feedback arrow to match the reference. The earlier module architecture remains in `architecture-draft.html` and is now in English.

## Mapping to the implementation

| Diagram element | Implementation |
|---|---|
| Model via Codex app-server | `codex_host.py`: app-server startup, thread configuration, dynamic tool registration; `transport.py`: JSON-RPC over stdio |
| Robot tool interface | `rollout.py`: `build_tool_specs`, `RobotRollout.handle_tool_call` |
| Observation tools | `robot_start`, `robot_observe` call the guarded observation path; `robot_tools.py`: `observe` |
| Action tools | `robot_move`, `robot_gripper`, `robot_head`, `robot_lift` map to `move`, `set_gripper`, `move_head`, `move_lift` |
| Validation | `rollout.py`: request context and action budget; `observation_guard.py`: observation freshness/state binding; `robot_tools.py`: argument and motion limits |
| ROS 2 control | `robot_io.py`: `send_pose`, `send_gripper`, `send_body`, and completion waiting; Gazebo uses `robot_io_gazebo.py` |
| Synchronized observation | `robot_io.py`: `get_snapshot`; `robot_tools.py`: `observe`, `_capture_after`; `rollout.py`: saved observations and replies |
| Completion | `rollout.py`: `_complete` validates the observation context and records the model's claim and final observation |
| Host stop | `codex_host.py` and `rollout.py`: configured limits, interruption, and execution errors |
| Sequential plan | `plan_executor.py`: `execute_plan`; `config.yaml`: `plan.max_steps = 5` |

The source is the current local working tree, including uncommitted changes. This is not a diagram of the repository's committed HEAD.

## Deliberate simplifications

- The Model → Result arrow abbreviates the `task_complete` tool call through the host and rollout. It is a model completion claim, not independently verified task success.
- The control block combines distinct command paths: Cyclo receives arm goals and gripper commands, while head/lift trajectories use their configured ROS topics. Intermediate external routing is abstracted.
- The observation block combines robot measurements with the host's synchronized snapshot and reply assembly. The host adds `request_id`; it is not published by the robot.
- The feedback arrow shows the normal observation response. Rejected calls return without motion; a failed observation can yield a response without new images/state.
- Actions execute one at a time. `robot_execute_plan` runs a bounded sequence, checking state and capturing observations between actions without asking the model to visually interpret intermediate images.
- `get_state` and `get_gripper` are internal reads rather than separately exposed model tools in this interface.

## Checks

- SVG XML parses successfully; diagram labels are English.
- All six observation/action tool names match the local tool definitions.
- PNG captured from the standalone HTML with Chromium at 1280 × 980.
- The rendered PNG was visually inspected: text and arrows are legible, with no clipped nodes or unintended crossings.
- No robot commands or application tests were run for this documentation-only change.

This plain SVG is independently authored; the Archify validation receipts belong to the earlier module architecture, not this diagram.
