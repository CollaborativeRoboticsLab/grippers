# Gripper Two Fingers

ROS 2 gripper-level wrapper for the two-finger Dynamixel gripper.

This package owns the gripper-facing action interface:

- `/open_gripper`
- `/close_gripper`

It also publishes the gripper articulation `joint_states` that drive the finger motion in `robot_state_publisher`.

## What This Package Does

- Wraps the low-level Dynamixel driver from `gripper_servo_dynamixel`
- Exposes gripper-level open/close actions
- Maps servo feedback into the two slider joints used by the current URDF model
- Keeps the simulated and physical gripper articulation aligned through the gripper config

The current preferred articulation model is:

- `gripper_planar_4` for the left finger slider
- `gripper_planar_5` for the right finger slider

Those joint positions are derived from the configured servo open/close range and published on `/joint_states`.

## Launches

The launch ownership is intentionally split:

- `gripper_ros/gripper_soft_two_fingers.launch.py`: independent hardware/gripper launch
- `gripper_ros/gripper_sim.launch.py`: visualization-only launch for `robot_state_publisher` and RViz

Run the hardware/gripper node:

```bash
source install/setup.bash
ros2 launch gripper_ros gripper_soft_two_fingers.launch.py
```

Run the visualization alongside it:

```bash
source install/setup.bash
ros2 launch gripper_ros gripper_sim.launch.py model:=two-finger-gripper-standalone
```

The sim launch does not start this node. It depends on the live `/joint_states` published by `gripper_two_fingers_node`.

## Actions

Open the gripper:

```bash
source install/setup.bash
ros2 action send_goal /open_gripper gripper_msgs/action/OpenGripper "{torque: 0.0, use_torque_mode: false}"
```

Close the gripper:

```bash
source install/setup.bash
ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close: true, torque: 0.0, use_torque_mode: false}"
```

## Configuration

The gripper-level runtime config is:

- `gripper_ros/config/grippers/soft_two_finger_dynamixel.yaml`

Key fields:

- `gripper_left_finger.joint_name`
- `gripper_left_finger.joint_open_position`
- `gripper_left_finger.joint_close_position`
- `gripper_right_finger.joint_name`
- `gripper_right_finger.joint_open_position`
- `gripper_right_finger.joint_close_position`

The current configuration maps servo feedback into the slider travel directly:

- open maps to `-0.045`
- close maps to `0.0`

If simulated motion is reversed relative to hardware, adjust only the `joint_open_position` and `joint_close_position` values in the gripper YAML.

## TF Tree

See [tf-tree.md](./tf-tree.md) for the current TF-tree note and image.