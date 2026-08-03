# Gripper action interface

This document covers the **gripper-level** action exposed by gripper wrapper packages.

The intended ownership is:

- gripper-level wrapper packages expose `/gripper_command`
- low-level servo packages expose `/servo_control` separately

## Action types

### `control_msgs/action/GripperCommand`

Goal:

| Field | Type | Description |
| --- | --- | --- |
| `command.position` | `float64` | Target gripper jaw opening in meters. Use `gripper_open_width` for open and `gripper_closed_width` for closed. |
| `command.max_effort` | `float64` | Optional effort limit in newtons. A non-zero value enables the backend's force/torque/current limiting path where supported. |

Result:

| Field | Type | Description |
| --- | --- | --- |
| `position` | `float64` | Final measured gripper jaw opening in meters. |
| `effort` | `float64` | Final or requested effort estimate in newtons when calibrated. |
| `stalled` | `bool` | `true` when the motion stopped on requested/safety effort before reaching the target. |
| `reached_goal` | `bool` | `true` when the target position was reached. |

Feedback:

| Field | Type | Description |
| --- | --- | --- |
| `position` | `float64` | Current measured gripper jaw opening in meters. |
| `effort` | `float64` | Current or requested effort estimate in newtons when calibrated. |
| `stalled` | `bool` | `true` when the motion is stopped on effort. |
| `reached_goal` | `bool` | `true` when the target position has been reached. |

## CLI usage

Discover actions:

```bash
source install/setup.bash
ros2 action list
```

Inspect the interface:

```bash
source install/setup.bash
ros2 interface show control_msgs/action/GripperCommand
```

Send an open gripper:

```bash
source install/setup.bash
ros2 action send_goal /gripper_command control_msgs/action/GripperCommand "{command: {position: 0.09, max_effort: 0.0}}"
```

Send a close gripper:

```bash
source install/setup.bash
ros2 action send_goal /gripper_command control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"
```

Send an intermediate position:

```bash
source install/setup.bash
ros2 action send_goal /gripper_command control_msgs/action/GripperCommand "{command: {position: 0.045, max_effort: 0.0}}"
```

Close with torque/current limiting (if supported by the active driver):

```bash
source install/setup.bash
ros2 action send_goal /gripper_command control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 5.0}}"
```

## Notes on `max_effort`

The standard action field is specified in newtons. The wrappers convert the requested jaw opening in meters into each servo backend's configured open/close command units. Effort reporting and limiting are only physically meaningful in newtons after the active backend is calibrated:

- Dynamixel: if `torque_per_current_unit` and the gripper geometry are calibrated, then the action goal `max_effort` can be treated as newtons at the fingers.
- Dynamixel position mode monitors measured torque and stops at `safety_torque_limit` without reopening the gripper.
- Dynamixel torque mode uses `control_torque` unless the goal sends a non-zero `max_effort`, and it stops once the requested effort is reached while holding position.
- Feetech: if `torque_limit_per_torque_unit`, `torque_per_current_unit`, and the gripper geometry are calibrated, the action goal `max_effort` can also be treated as newtons at the fingers.
- Feetech still uses an approximate torque mode implemented through a torque-limit register plus current monitoring, not true current-based position control.

If you want to command physical units such as Nm, calibrate the active driver so one action-unit maps consistently to the driver's measured torque estimate.
