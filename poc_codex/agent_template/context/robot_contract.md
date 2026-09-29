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
- After an action runs, its packet has a new observation and a new `request_id`.
- When an action is rejected for its arguments (`invalid_action`), nothing moved and the previous observation stays current. The packet has a new `request_id` for the corrected action.
- An observation expires after `action_context.max_age_s` seconds (60). It is also rejected if the robot's state has changed since, for example a gripper holding an object that kept closing. Either way `movement_allowed` becomes false and you must call `robot_observe`.
- `next_call` in each packet names the tool to call next.

## Packet

| Field | Contents |
|---|---|
| `action_executed` | Whether a robot command ran |
| `status` | `observed`, `arrived`, `stopped`, `arrived_without_observation`, `stopped_without_observation`, `rejected`, `observation_failed`, `completed` (after `task_complete`) |
| `rejection`, `reason` | Why nothing ran, or details of the result |
| `result` | Target, arrival and error; for a rejection, the requested values and the current position |
| `observation` | New observation or null: `observation_number`, `arms.{right,left}.{xyz, rpy, gripper, gripper_rad}`, `body` (head joints, lift), image paths, `path` of `observation.json` |
| `observation_note` | Why there is no new observation, and what to do |
| `action_context` | `movement_allowed`, `request_id`, `max_age_s` |
| `progress` | `actions_executed`, `actions_remaining`, `seconds_remaining`, `rollout_finished` (the episode has ended) |
| `next_call` | The tool to call next, with `request_id` when an action is allowed |
