# Gripper Feetech Test

This package provides a small C++ (rclcpp) test node that simulates a Feetech gripper for integration and testing.

This node follows the guidelines of [Gripper-level ROS2 interface](ros2-interface.md) and exposes, 
- `/open_gripper` 
- `/close_gripper`

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
ros2 action send_goal /open_gripper gripper_msgs/action/OpenGripper "{torque: 0.0, use_torque_mode: false}"
```

### Close the gripper:

```bash
source install/setup.bash
ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close: true, torque: 0.0, use_torque_mode: false}"
```

Close in approximate torque mode:

```bash
source install/setup.bash
ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close: true, torque: 0.5, use_torque_mode: true}"
```

The Feetech test wrapper now mirrors the updated torque-monitoring behavior from the low-level Feetech node:

- Position mode stops and holds position if measured torque exceeds `safety_torque_limit`.
- Torque mode uses `control_torque` unless the goal provides a non-zero `torque`.
- The implementation is still approximate because the underlying servo path uses a torque-limit register rather than true current-based position control.