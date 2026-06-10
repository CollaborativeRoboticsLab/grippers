# Servo action interface

This document covers the **low-level servo control** action exposed by the servo packages.

The intended ownership is:

- low-level servo packages expose `/servo_control`
- gripper-level wrapper packages should not expose low-level servo APIs unless explicitly needed for debugging

## Action types

### `gripper_msgs/action/ServoControl`

Goal:

- `position` (float): target servo position in the configured command units
- `torque` (float): optional implementation-defined torque/current limit

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
ros2 action send_goal /servo_control gripper_msgs/action/ServoControl "{position: 900.0, torque: 80.0}"
```

## Notes on `torque`

The action goal field is named `torque`, but its meaning is intentionally **driver-specific**:

- For `ServoControl`, a nonzero `torque` value typically implies a torque-limited or current-limited position command.
- A Dynamixel-based driver will typically treat it like **Goal Current** (raw value; model-specific units).
- A Feetech STS/SCS-based driver will typically treat it like a **torque limit register value** (raw value; model-specific units).

See the driver-specific docs for how that value is used.
