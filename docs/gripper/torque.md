# max_effort, control_torque, stall_torque, and safety_torque_limit

`command.max_effort` is the replacement for the older custom-message torque field.

At the gripper-wrapper level, these values have different jobs:

- `command.max_effort`: per-goal requested gripper force override in newtons.
- `max_effort`: default requested gripper force in newtons when torque mode is active and the goal does not provide one.
- `max_effort_to_torque_factor`: per-gripper conversion from gripper force in newtons to low-level torque units.
- `control_torque`: default low-level torque request for the servo action.
- `safety_torque_limit`: software safety stop threshold.
- `stall_torque`: hard physical ceiling from the actuator datasheet that should never be exceeded.

The requested effort is resolved in this order:

1. If `command.max_effort != 0.0`, use `command.max_effort` as the requested gripper force.
2. Else if torque mode is active and `max_effort != 0.0`, use the configured gripper-level `max_effort`.
3. Convert that gripper force into a torque request using `max_effort_to_torque_factor`.
4. Else use `0.0` and stay on the position-only path.

Torque mode is enabled when either of the following is true:

- the goal sends `command.max_effort != 0.0`
- the backend config already has `use_torque_mode: true`

During execution, the stop conditions are checked in this order:

1. If the position target is reached, the goal succeeds normally.
2. If torque mode is active and measured effort reaches the requested target effort, the node switches into current hold and reports success.
3. If measured effort reaches `safety_torque_limit`, the node switches into current hold and reports success at the safety limit.

Before the command is written to Dynamixel, the requested effort is clamped to the lowest configured ceiling among:

- `safety_torque_limit`
- `stall_torque`

This means `safety_torque_limit` is the active software guardrail, while `stall_torque` is the physical hard ceiling. If `safety_torque_limit` is lower than the requested effort target, the safety limit wins.

Example:

- config: `max_effort: 2.0`, `max_effort_to_torque_factor: 0.2`, `safety_torque_limit: 1.0`, `stall_torque: 4.1`, `use_torque_mode: true`
- goal: `command.max_effort: 1.5`

Result:

- the requested gripper force target becomes `1.5 N`
- the resulting torque request becomes `0.3` in the configured low-level torque units
- torque mode is active
- the gripper closes toward the requested position
- but motion will stop at `1.0` if the measured effort reaches `safety_torque_limit` before it reaches `1.5`

## Dynamixel current-unit note

For XM430 servos, ROBOTIS documents `Current Limit(38)`, `Goal Current(102)`, and `Present Current(126)` in raw current units with a resolution of about `2.69 mA` per count.

- The datasheet current-limit range is `0 .. 1193`.
- The goal-current command range is `-1193 .. 1193` when the current limit is left at its default maximum.

In this codebase:

- `min_current_unit` and `max_current_unit` configure the allowed raw current magnitude written by the node.
- `torque_per_current_unit` is not the same thing as `2.69 mA`.
- `torque_per_current_unit` converts one raw current count into your chosen torque unit, such as `Nm / raw_current_unit`.

The numeric unit is only physically meaningful after the active backend is calibrated:

- Dynamixel: if `torque_per_current_unit` and `max_effort_to_torque_factor` are calibrated, gripper-level `command.max_effort` can be treated as force at the fingers while the low-level servo action still uses torque units.
- Dynamixel position mode monitors measured torque and stops at `safety_torque_limit` without reopening the gripper.
- Dynamixel torque mode uses the resolved requested effort above and, once contact effort is reached, switches from current-based position mode into pure current mode so the gripper keeps holding torque until the next command.
- Feetech: if `torque_limit_per_torque_unit`, `torque_per_current_unit`, and the gripper geometry are calibrated, `command.max_effort` can also be treated in a physical unit.
- Feetech still uses an approximate torque mode implemented through a torque-limit register plus current monitoring, not true current-based position control.

If you want to command physical units such as Nm or newtons consistently, calibrate the active driver so one action-unit maps predictably to the driver's measured torque estimate and the gripper mechanism.
