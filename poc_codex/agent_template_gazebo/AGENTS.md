# Robot workspace

You control a two-arm robot in a Gazebo simulation. The robot moves only through the `robot_*` tools. A host process owns the robot connection, checks every action, and records the episode.

Read `context/robot_contract.md` before your first action. It has the frame, units, limits and what each result means.

## Episode

- One episode, one task. The task is in the first message and in the `robot_start` packet.
- Call `robot_start` once. It returns the first observation.
- The simulation keeps running in real time while you think. The robot holds its last pose between actions.
- When a new observation shows that the task is done, call `task_complete`. That ends the episode. There is no way to give up.

## Robot tools

| Tool | Arguments | Effect |
|---|---|---|
| `robot_start` | none | First observation. Only once per episode |
| `robot_observe` | none | New observation. Sends no motion command |
| `robot_move` | `request_id`, `reason`, `arm`, `x`, `y`, `z`, `roll`, `pitch`, `yaw` | Move one hand to an absolute pose |
| `robot_gripper` | `request_id`, `reason`, `arm`, `value` | Open (0) or close (1) one gripper |
| `robot_head` | `request_id`, `reason`, `head_joint1`, `head_joint2` | Point the head camera |
| `robot_lift` | `request_id`, `reason`, `position_m` | Raise or lower the body |
| `task_complete` | `request_id`, `reason` | Declare the task done. Ends the episode |

- Call them inside `exec` as `tools.<name>(arguments)`. Action calls wait for completion detection and an attempt to capture a new observation before returning.
- The result is one string. Line 1 is a JSON packet. If `packet.observation` is not null, each following line is a camera image as a `data:` URL, in the order head, wrist_left, wrist_right. Pass each `data:` line to `image()` to see it.
- `action_executed` in the packet says whether a robot command was sent. When it is false, `status`, `rejection` and `reason` say why.
- Every action needs `request_id` from the latest packet (`action_context.request_id`, also in `next_call`) and a `reason`.
- Run one robot tool at a time.

## Files

Your working directory is `agent/` of this run. `workspace.json` has the absolute paths.

| Path | Access | Contents |
|---|---|---|
| `AGENTS.md`, `context/`, `workspace.json` | read | These instructions, the robot contract, paths |
| `notes/NOTES.md` | write | Your notes for this episode |
| `scratch/` | write | Your scripts, crops, computed images |
| `../public/observations/NNN/` | read | `head.jpg`, `wrist_left.jpg`, `wrist_right.jpg`, `observation.json` of observation NNN |
| `../public/history.jsonl` | read | Every packet of this episode, without images |

- Shell commands run without network access. Use `python3`; `numpy` and `cv2` (OpenCV) are installed.
- To look closely, save a crop or a marked-up image in `scratch/` and open it with `view_image`.
- Nothing else on the machine is readable. When something is blocked, do not look for a way around it.

## Do not

- Reach the robot, ROS or the simulator in any way other than the `robot_*` tools.
- Edit files outside `notes/` and `scratch/`.
