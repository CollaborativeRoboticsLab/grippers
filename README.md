* for low-level servo control and gripper-level open/close control.

## Packages

- `gripper_servo_dynamixel`: low-level DynamixelSDK (Protocol 2.0) servo/action package (Python).
- `gripper_servo_feetech`: low-level Feetech STS/SCS servo/action package (C++).
- `gripper_two_fingers`: gripper-level two-finger Dynamixel wrapper (Python).
- `gripper_feetech_test`: gripper-level Feetech test wrapper (C++).
- `gripper_ros`: launch files + centralized motor/gripper parameter YAMLs.

## Action API

This ros packages use two action layers:

- low-level servo control: `/servo_control`
- gripper-level control: `/gripper_command` using `control_msgs/action/GripperCommand`

See [docs/servo/ros2-interface.md](docs/servo/ros2-interface.md) for low-level servo ros2 specifications and examples.
See [docs/gripper/ros2-interface.md](docs/gripper/ros2-interface.md) for gripper level ros2 specifications and examples.

## Installation and Setup

Follow the Setup instructions in [docs/installation.md](docs/installation.md) to install dependencies and build the workspace.

## Servo-level launch and testing

See [gripper_servo_dynamixel/README.md](gripper_servo_dynamixel/README.md) for testing Dynamixel servos.
See [gripper_servo_feetech/README.md](gripper_servo_feetech/README.md) for testing Feetech servos.

## Gripper-level launch:

### Two-finger Dynamixel gripper

```bash
source install/setup.bash
ros2 launch gripper_ros gripper_soft_two_fingers.launch.py
```

load the visualization for the two-finger gripper:

```bash
source install/setup.bash
ros2 launch gripper_ros gripper_sim.launch.py model:=two-finger-gripper-standalone
```

This starts the gripper URDF in `robot_state_publisher` and `rviz2` only.
Run it alongside `gripper_soft_two_fingers.launch.py` when you want RViz to reflect live `/joint_states` from the gripper node.

### Gripper control

Use the action CLI to send gripper goals (see [docs/gripper/ros2-interface.md](docs/gripper/ros2-interface.md) for details):
	
To open gripper:

```bash
source install/setup.bash
ros2 action send_goal /gripper_command control_msgs/action/GripperCommand "{command: {position: 0.09, max_effort: 0.0}}"
```

To close gripper:

```bash
source install/setup.bash
ros2 action send_goal /gripper_command control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"
```

For calibration/debugging only, the gripper wrapper also exposes a `bypass_max_effort` parameter. When enabled, gripper-level `max_effort` bypasses `max_effort_to_torque_factor` and is treated directly as low-level torque.

### Gripper calibration

Start the gripper stack with `bypass_max_effort` enabled.

```bash
source install/setup.bash
ros2 launch gripper_ros gripper_soft_two_fingers.launch.py bypass_max_effort:=true
```

Use the helper node to step torque through `/gripper_command`, collect measured force samples, and update the plot after each sample:

```bash
source install/setup.bash
ros2 run gripper_ros estimate_mett_node.py --ros-args \
	-p target_position:=0.0 \
	-p release_position:=0.09 \
	-p start_torque:=0.2 \
	-p torque_increment:=0.2 \
	-p max_torque:=1.0
```

The node interaction is intentionally simple:

- press Enter to apply the current torque
- press Enter again to release it
- enter the measured force in newtons
- inspect the matplotlib plot
- press Enter to continue to the next torque increment

Read the [gripper action interface docs](docs/gripper/ros2-interface.md) for more details on `GripperCommand` goal fields and CLI usage.
Read the [servo action interface docs](docs/servo/ros2-interface.md) for direct low-level servo commands.


## Additional documentation

- [Detailed Startup Instructions](./docs/detailed-start.md)
- [Adding new grippers, descriptions, configs, and launch files](./docs/gripper/adding-new-grippers.md)
- [Servo-specific parameter documentation](./docs/servo/parameters.md)
- [Gripper-specific parameter documentation](./docs/gripper/parameters.md)
- [Dynamixel Troubleshooting](./docs/servo/dynamixel-troubleshooting.md)