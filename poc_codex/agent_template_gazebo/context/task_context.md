# Task rules

## Goal

Do the task in the first message as written. When a new observation shows that it is done, call `task_complete` with that observation's `request_id` and the visible evidence as `reason`. The host records the final state, and a person checks every claim against it: a claim the evidence does not support counts as a failure. Do not add conditions to the task.

## Evidence

- Base each conclusion on observations: the images and the measured state.
- `arrived` means the hand reached the commanded pose. It does not mean the hand touched anything. A closed gripper does not mean a grasp. Check contact, grasp and lift in the images of a later observation.
- If the evidence is unclear, keep the conclusion uncertain. When a later observation contradicts an earlier conclusion, drop the earlier conclusion.
- You are not given object positions, object sizes or the table height. Estimate them from the images and the measured poses, and write down how you estimated them.

## Acting

- Choose the method yourself.
- Prefer `robot_execute_plan` to group consecutive actions when all targets can be decided from the current observation without intermediate visual reasoning. Use the tool schema's step limit; each step contains `tool`, `args`, and `reason`, with the latest `request_id` at the plan level. Steps execute sequentially with host observations and state checks, but the model does not inspect intermediate images.
- Head, lift, and any nonzero gripper command must be the final plan step. When the next target depends on new visual evidence, use individual calls and inspect the result first. Inspect the final plan observation before proceeding or claiming success.
- Give every action a `reason`: the visible evidence and the purpose.
- Analyze as much as you need before acting: crop, compute, compare observations. An observation is valid for `action_context.max_age_s` seconds. When it has expired, the action is rejected; call `robot_observe`, check the scene again and decide again.
- A rejected action did not run, and nothing is retried for you.
- The host's pre-action check compares only the robot's own state. It cannot tell whether an object moved. Objects can move while you think, because the simulation keeps running.
- A command whose outcome cannot be confirmed, such as a hand still moving after the time limit, ends the episode. A hand or gripper that stopped short (`stopped`) did run; look at the new observation to see why.

## Keep going

- Continue until the task is done. A failed grasp, a rejection or low odds are not reasons to stop. Find the likely cause, change the approach and try again. There is no way to give up; if the task is not done, keep working until the budget runs out.
- Ending your turn does not end the episode. You will be asked to continue.
- `progress.actions_remaining` and `progress.seconds_remaining` show the budget. Rejections and observations do not use actions.

## Notes

Keep `notes/NOTES.md` current: your estimates and how you got them, what you tried, what happened, what you will change. `../public/history.jsonl` has every earlier packet if you need to look back.
