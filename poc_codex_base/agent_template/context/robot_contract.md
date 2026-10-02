# Robot contract

## Frame and units

- All poses are in `base_link`: x forward, y left, z up. Meters and radians.
- A hand pose is `xyz` and `rpy`, the pose of that arm's end effector link. The rotation is R = Rz(yaw) · Ry(pitch) · Rx(roll).
- The gripper points along the -z axis of its end effector link.
- Packets round poses to 4 decimals.

## Arms: `robot_move`

- The target is an absolute pose for one arm (`right` or `left`). The other arm and both grippers stay as they are.
- Either arm can be used. The arms are controlled independently, one call at a time; nothing keeps the two hands in a fixed relation.
- The hand moves on a straight line to the target. The arm controller tracks it within joint position and velocity limits and a self-collision margin; when a target would break them, the hand leaves the line or stops short, with nothing touched.
- The controller avoids collisions only between parts of the robot, and not in every case. It does not know about the table or objects: a target inside them drives the hand into them.
- The target may be at most **0.05 m** away from the current measured position and at most **0.35 rad** of rotation from the current measured orientation. Larger steps are rejected without moving.
- A target lower than 0.8 m below the arm base is rejected. The arm base moves with the lift.
- A move ends when the hand comes to rest: it moved less than 0.002 m and 0.5° for 1 s, judged after its 3 s trajectory. A move takes at least 4 s, and the new observation shows the hand at rest.
- `arrived`: at rest within 0.01 m and 5° of the target. The packet gives the remaining error.
- `stopped`: at rest farther from the target, for example against an object or the table, at a joint limit, or where the self-collision margin blocks it, such as the two hands coming close. The action ran; the packet gives the remaining error and a new observation. The arm keeps pushing toward the target until the next move. A hand creeping very slowly, for example under a load, can also be reported as `stopped` and may still move a little afterwards.
- If the host cannot confirm that the hand has come to rest within 15 s, the episode ends with a timeout. This does not cancel the controller command or prove that the robot stopped.

## Grippers: `robot_gripper`

- `value` 0 is fully open, 1 is closed. It maps linearly to the gripper joint angle, 0 to 1 rad.
- A gripper command ends when the joint comes to rest: the angle moved less than 0.005 rad for 1 s, judged after its 1 s trajectory.
- `arrived`: at rest within 0.02 rad of the target.
- `stopped`: at rest farther from the target, for example with the fingers on an object. The gripper keeps closing toward the target until the next gripper command.
- Neither `arrived` nor `stopped` proves a grasp. Check the images.
- If the host cannot confirm that the gripper has come to rest within 15 s, the episode ends with a timeout. This does not cancel the controller command or prove that the gripper stopped.
- Observations report each gripper as `gripper` (0 to 1) and `gripper_rad`.

## Head: `robot_head`

- `head_joint1` tilts about the head's local Y axis, range -0.2317 to 0.6951 rad.
- `head_joint2` pans about the head's local Z axis, range -0.35 to 0.35 rad.
- Values outside the range are rejected without moving.

## Lift: `robot_lift`

- `position_m` is the lift joint displacement, range -0.5 to 0 m. 0 is the highest position.
- Raising or lowering moves the torso, which carries both arm bases and the head.
- The arm controller keeps each hand near its last commanded `base_link` pose by re-bending the arms. The measured poses may lag by about 1 cm while the lift moves, or shift farther near a reach limit.
- The region the arms can reach moves with the lift: lowering lets the hands reach lower (see the 0.8 m limit under Arms).
- The head camera moves with the torso, so use a new image after a lift move instead of pixel positions from an earlier head image.

## Base: `robot_base`

- Moves the mobile base by body velocity in `base_link`: `vx` forward and `vy` left in m/s, `wz` counterclockwise in rad/s, for `duration_s` seconds. `duration_s` includes speeding up and slowing down.
- Each non-zero axis must be at least 0.1 m/s (`vx`, `vy`) or 0.1 rad/s (`wz`); a smaller value would be ignored by the wheel controller, so it is rejected. Use 0 for an axis that should not move.
- Limits: hypot(`vx`, `vy`) at most 0.15 m/s, |`wz`| at most 0.3 rad/s, `duration_s` at most 2 s, hypot(`vx`, `vy`) × `duration_s` at most 0.2 m, and |`wz`| × `duration_s` at most 0.4 rad. Larger requests are rejected without moving.
- Both hands must be near the body: x at most 0.48 m and |y| at most 0.3 m in `base_link`. The lift must also be within -0.5 to 0 m. Otherwise the request is rejected without moving; bring the hands in and set the lift within range first.
- `arrived` means the velocity profile ran and the base came to rest. It does not mean a distance was reached. `result.motion.odom_measured` gives the measured net displacement in the `base_link` frame at the start of the command: `dx_m` and `dy_m` in meters, `dyaw_rad` in radians. Plan the next action from that value and the new images, never from velocity × `duration_s`.
- The measurement comes from wheel rotation (odometry). It is the net displacement, not the length of the path driven.
- Odometry cannot see the robot's surroundings. When the base is blocked, for example pushing against an object, the wheels can slip and odometry still reports normal motion with `arrived`. The host has no obstacle sensor; only the images show whether the base really moved and whether the path is clear. After each base command, compare the new images with the previous ones: the scene should shift by about the reported amount. If it did not, treat the base as blocked and do not repeat the command.
- The first command after a direction change may travel substantially less because of steering alignment and the velocity ramps.
- To cover a distance, repeat small commands in the same direction and check the images after each one. Avoid alternating directions (zig-zag). Do not make up a shortfall with one larger command.
- `stopped` means odometry saw a negligible amount or no progress. A blocked base can also be `arrived` (see above). Retry only after confirmed rest and a fresh observation show a clear path; stop retrying if progress remains negligible.
- When a direction is rejected after no progress, inspect the new observation and identify the cause before attempting that direction again.
- `rejection: not_ready`: nothing moved, but the base was not ready, for example still moving. Call `robot_observe`.
- If the host cannot confirm that the base has come to rest, or loses its measurements while the base moves, the episode ends.
- Observations report the base pose as `base`: `xy` in meters and `yaw` in radians in the odometry frame named by `base.frame`. Use it to follow how far the base has gone in total.
- Any action is rejected while the base is still moving, and after the base has moved more than 0.05 m or 3° since the observation. Call `robot_observe`.

- Call `robot_base` separately; it is not supported inside `robot_execute_plan`.
- The physical harness uses system timestamps and elapsed wall time; it does not require `/clock`.

## Cameras

| Camera | Mounted on |
|---|---|
| `head` | Head, moves with `robot_head` and the lift |
| `wrist_left` | Left wrist camera |
| `wrist_right` | Right wrist camera |

Image dimensions are provided in the observation metadata.
The cameras are matched approximately by their ROS timestamps.
They are not guaranteed to expose at exactly the same instant.

## Observation and request_id

- Every observation has a `request_id`. An action must use the `request_id` of the latest observation; any other value is rejected (`stale_request_id`).
- After an action completes and its post-observation succeeds, the same tool response contains a new observation and a new `request_id`; a separate `robot_observe` call is not needed.
- If post-observation fails, the response has `observation: null` and status `arrived_without_observation` or `stopped_without_observation`. Call `robot_observe` before another action.
- Submit the action target, `reason` (observed evidence and purpose), and latest `request_id` together. The response packet's `reason` describes the execution result. Robot tools run one at a time.
- When an action is rejected for its arguments (`invalid_action`), nothing moved and the previous observation stays current. The packet has a new `request_id` for the corrected action.
- An observation expires after `action_context.max_age_s` seconds (60). It is also rejected if the robot's state has changed since, for example a gripper holding an object that kept closing. Either way `movement_allowed` becomes false and you must call `robot_observe`.
- `next_call` in each packet names the tool to call next.

## Packet

| Field | Contents |
|---|---|
| `action_executed` | Whether a robot command was sent; this does not prove displacement or success |
| `status` | `observed`, `arrived`, `stopped`, `arrived_without_observation`, `stopped_without_observation`, `rejected`, `observation_failed`, `completed` (after `task_complete`) |
| `rejection`, `reason` | Why nothing ran, or details of the result |
| `result` | Target, arrival and error; for `robot_base`, also `motion` (`commanded`, `duration_s`, `rest_confirmed`, `odom_measured`); for a rejection, the requested values and the current position |
| `observation` | New observation or null: `observation_number`, `arms.{right,left}.{xyz, rpy, gripper, gripper_rad}`, `body` (head joints, lift), `base` (`frame`, `xy`, `yaw`), image paths, `path` of `observation.json` |
| `observation_note` | Why there is no new observation, and what to do |
| `action_context` | `movement_allowed`, `request_id`, `max_age_s` |
| `progress` | `actions_executed`, `actions_remaining`, `seconds_remaining`, `rollout_finished` (the episode has ended) |
| `next_call` | The tool to call next, with `request_id` when an action is allowed |
