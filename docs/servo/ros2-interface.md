# Servo action interface

This document covers the **low-level servo control** action exposed by the servo packages.

The intended ownership is:

- low-level servo packages expose `/servo_control`
- gripper-level wrapper packages should not expose low-level servo APIs unless explicitly needed for debugging

## Action types

### `gripper_msgs/action/ServoControl`

Goal:

- `position` (float): target servo position in the configured command units
- `torque` (float): optional torque request; interpretation depends on the active driver

Result:

- `success` (bool)
- `message` (string)

Feedback:

- `progress` (float32): $0.0 \rightarrow 1.0$ best-effort progress estimate

## CLI usage

Discover actions:

```bash
source install/setup.bash
ros2 action list
```

Inspect the interface:

```bash
source install/setup.bash
ros2 interface show gripper_msgs/action/ServoControl
```

Send a direct servo command:

```bash
source install/setup.bash
ros2 action send_goal /servo_control gripper_msgs/action/ServoControl "{position: 900.0, torque: 0.0}"
```

Servo command with torque/current limiting (if supported by the active driver):

```bash
source install/setup.bash
ros2 action send_goal /servo_control gripper_msgs/action/ServoControl "{position: 900.0, torque: 0.5}"
```

## Notes on `torque`

The action goal field is named `torque`, but its meaning is intentionally **driver-specific**:

- For `ServoControl`, a nonzero `torque` value typically implies a torque-limited or current-limited position command.
- A Dynamixel-based driver interprets it in the configured torque units and converts it to Goal Current internally using `torque_per_current_unit`. If that parameter is calibrated in Nm, then the action `torque` value is effectively in Nm as well.
- A Feetech STS/SCS-based driver can also map the action `torque` value into configured torque units when `torque_limit_per_torque_unit` and `torque_per_current_unit` are calibrated, but it remains an approximation built on a torque-limit register rather than true current-based position control.

See the driver-specific docs for how that value is used.
