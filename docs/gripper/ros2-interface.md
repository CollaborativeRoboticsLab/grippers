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
| `command.position` | `float64` | Target total gripper jaw opening in meters. Use `gripper_open_width` for open and `gripper_closed_width` for closed. The node maps that public width into the configured left and right finger joint positions. |
| `command.max_effort` | `float64` | Optional gripper effort request in newtons. A non-zero value overrides the configured gripper default and is converted into the backend's low-level torque/current units. A value of `0.0` means "do not override the configured gripper default". |

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

Close with force limiting (if supported by the active driver and calibrated for the active gripper):

```bash
source install/setup.bash
ros2 action send_goal /gripper_command control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 5.0}}"
```

