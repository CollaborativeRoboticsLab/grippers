# max_effort, control_torque, default_torque, and safety_torque_limit

`command.max_effort` is the replacement for the older custom-message torque field.

At the gripper-wrapper level, these values have different jobs:

- `command.max_effort`: per-goal requested effort override.
- `control_torque`: default requested effort when the goal does not provide one.
- `default_torque`: final fallback if neither the goal nor `control_torque` provides a non-zero effort.
- `safety_torque_limit`: safety stop threshold that can stop motion before the requested effort is reached.

The requested effort is resolved in this order:

1. If `command.max_effort != 0.0`, use `command.max_effort`.
2. Else if `control_torque != 0.0`, use `control_torque`.
3. Else use `default_torque`.

Torque mode is enabled when either of the following is true:

- the goal sends `command.max_effort != 0.0`
- the backend config already has `use_torque_mode: true`

During execution, the stop conditions are checked in this order:

1. If the position target is reached, the goal succeeds normally.
2. If torque mode is active and measured effort reaches the requested target effort, the node holds position and reports success.
3. If measured effort reaches `safety_torque_limit`, the node holds position and reports success at the safety limit.

This means `safety_torque_limit` is a guardrail, not a default effort request. If `safety_torque_limit` is lower than the requested effort target, the safety limit wins.

Example:

- config: `control_torque: 2.0`, `default_torque: 0.0`, `safety_torque_limit: 1.0`, `use_torque_mode: true`
- goal: `command.max_effort: 1.5`

Result:

- the requested effort target becomes `1.5`
- torque mode is active
- the gripper closes toward the requested position
- but motion will stop at `1.0` if the measured effort reaches `safety_torque_limit` before it reaches `1.5`

The numeric unit is only physically meaningful after the active backend is calibrated:

- Dynamixel: if `torque_per_current_unit` and the gripper geometry are calibrated, `command.max_effort` can be treated as force at the fingers or torque in your chosen calibrated unit.
- Dynamixel position mode monitors measured torque and stops at `safety_torque_limit` without reopening the gripper.
- Dynamixel torque mode uses the resolved requested effort above and stops once that requested effort is reached while holding position.
- Feetech: if `torque_limit_per_torque_unit`, `torque_per_current_unit`, and the gripper geometry are calibrated, `command.max_effort` can also be treated in a physical unit.
- Feetech still uses an approximate torque mode implemented through a torque-limit register plus current monitoring, not true current-based position control.

If you want to command physical units such as Nm or newtons consistently, calibrate the active driver so one action-unit maps predictably to the driver's measured torque estimate and the gripper mechanism.
