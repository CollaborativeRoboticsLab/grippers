# Gripper Servo Feetech

This package provides utility code and ROS 2 **action server** node for direct Feetech STS/SCS-style single-servo control, using the driver sources vendored in `gripper_servo_feetech/include/gripper_servo_feetech/Feetech-STSServo`.

The ros2 node follows the guidelines of [Servo-level ROS2 interface](../docs/servo/ros2-interface.md) and exposes the low-level `/servo_control` action.

## Quick start

Run the low level control node via launch file:

```bash
source install/setup.bash
ros2 launch gripper_ros feetech.launch.py
```

Direct servo command:

```bash
source install/setup.bash
ros2 action send_goal /servo_control control_msgs/action/GripperCommand "{command: {position: 900.0, max_effort: 0.0}}"
```

Servo command with torque/current limiting (if supported by the active driver):

```bash
source install/setup.bash
ros2 action send_goal /servo_control control_msgs/action/GripperCommand "{command: {position: 900.0, max_effort: 0.5}}"
```

## Position mode vs torque mode

The Feetech node now supports the same high-level torque-monitoring features as the Dynamixel path, but the control method is still approximate:

- Position mode writes the target position and monitors measured current-derived torque.
- If measured torque exceeds `safety_torque_limit`, the node rewrites the current position as a hold target so the gripper stops without reopening.
- Torque mode uses `control_torque` only when torque mode is enabled and the action goal leaves `command.max_effort` at `0.0`.
- The requested torque is clamped against `safety_torque_limit` and `stall_torque` when those are configured.
- The requested torque is then converted into the Feetech torque-limit register using `torque_limit_per_torque_unit`.
- The node then monitors measured torque and stops once the requested torque is reached.

Important: unlike Dynamixel current-based position control, the Feetech path still approximates torque mode through a torque-limit register plus current monitoring. It is not true current-based position control.

If `torque_limit_per_torque_unit` and `torque_per_current_unit` are calibrated in Nm, then the action goal `command.max_effort` is effectively interpreted in Nm.