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
ros2 action send_goal /servo_control gripper_msgs/action/ServoControl "{position: 900.0, torque: 0.0}"
```

Servo command with torque/current limiting (if supported by the active driver):

```bash
source install/setup.bash
ros2 action send_goal /servo_control gripper_msgs/action/ServoControl "{position: 900.0, torque: 80.0}"
```