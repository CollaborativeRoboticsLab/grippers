# Gripper Feetech Test

This package provides a small C++ (rclcpp) test node that simulates a Feetech gripper for integration and testing.

This node follows the guidelines of [Gripper-level ROS2 interface](../docs/gripper/ros2-interface.md) and exposes:

- `/gripper_command` using `control_msgs/action/GripperCommand`

This builds on top of [Gripper Servo Feetech](../gripper_servo_feetech/README.md)

## Quick start

Run the hardware/gripper node:

```bash
source install/setup.bash
ros2 launch gripper_ros gripper_feetech_test.launch.py
```

### Open the gripper:

```bash
source install/setup.bash
ros2 action send_goal /gripper_command control_msgs/action/GripperCommand "{command: {position: 0.09, max_effort: 0.0}}"
```

### Close the gripper:

```bash
source install/setup.bash
ros2 action send_goal /gripper_command control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"
```

Close with an effort limit:

```bash
source install/setup.bash
ros2 action send_goal /gripper_command control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 5.0}}"
```

`GripperCommand.command.position` is the jaw opening in meters. The default test config maps `0.09` m to the configured servo open position and `0.0` m to the configured servo close position.

The Feetech test wrapper now mirrors the updated torque-monitoring behavior from the low-level Feetech node:

- Position mode stops and holds position if measured torque exceeds `safety_torque_limit`.
- Torque mode uses `control_torque` unless the goal provides a non-zero `max_effort`.
- The implementation is still approximate because the underlying servo path uses a torque-limit register rather than true current-based position control.