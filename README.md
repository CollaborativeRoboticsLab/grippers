# grippers

ROS 2 gripper drivers that expose a **common action-based API** for low-level servo control and gripper-level open/close control.

## Packages

- `gripper_msgs`: ROS 2 action definitions (`ServoControl`, `OpenGripper`, `CloseGripper`).
- `gripper_servo_dynamixel`: low-level DynamixelSDK (Protocol 2.0) servo/action package (Python).
- `gripper_two_fingers`: gripper-level two-finger Dynamixel wrapper (Python).
- `gripper_servo_feetech`: low-level Feetech STS/SCS servo/action package (C++).
- `gripper_feetech_test`: gripper-level Feetech test wrapper (C++).
- `gripper_ros`: launch files + centralized motor/gripper parameter YAMLs.

No superseded source package directories remain in this repository. The active package set above is the current supported stack.

## Action API

This workspace uses two action layers:

- low-level servo control: `/servo_control`
- gripper-level control: `/open_gripper` and `/close_gripper`

See [docs/servo_action_interface.md](docs/servo_action_interface.md) for low-level servo command fields and CLI examples.
See [docs/gripper_action_interface.md](docs/gripper_action_interface.md) for gripper open/close fields and CLI examples.

### Dependencies

```bash
sudo apt update
sudo apt install -y \
	python3-colcon-common-extensions \
	python3-rosdep \
	python3-pip \
    python3-serial
```

## Cloning

If you are adding this repo into a ROS 2 colcon workspace:

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone --recurse-submodules https://github.com/CollaborativeRoboticsLab/grippers.git
```

### ROS deps via rosdep

From the root of your colcon workspace:

```bash
cd ~/colcon_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro "$ROS_DISTRO"
```

### Serial port permissions

Most USB serial adapters require your user to be in `dialout`:

```bash
sudo usermod -a -G dialout $USER
```

Log out/in after changing groups.

## Build

```bash
cd ~/colcon_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Testing Motors

### Feetech (STS/SCS)

```bash
ros2 launch gripper_ros feetech.launch.py
```
See [docs/feetech.md](docs/feetech.md) for parameter details.

### Dynamixel

```bash
source install/setup.bash
ros2 launch gripper_ros dynamixel.launch.py
```
See [docs/dynamixel.md](docs/dynamixel.md) for motor model presets and parameter details.

### Gripper-level launch:

```bash
source install/setup.bash
ros2 launch gripper_ros gripper_soft_two_fingers.launch.py
```

This is the independent runtime launch for the two-finger Dynamixel gripper node.

### Gripper URDF simulation in RViz:

```bash
source install/setup.bash
ros2 launch gripper_ros gripper_sim.launch.py model:=two-finger-gripper-standalone
```

This starts the gripper URDF in `robot_state_publisher` and `rviz2` only.
Run it alongside `gripper_soft_two_fingers.launch.py` when you want RViz to reflect live `/joint_states` from the gripper node.

If a USB serial adapter is unplugged and replugged, the device path may change from `/dev/ttyUSB0` to `/dev/ttyUSB1` or similar. Recheck `/dev/ttyUSB*` before assuming the motor settings changed.

### Gripper control

Use the action CLI to send open/close goals (see [docs/gripper_action_interface.md](docs/gripper_action_interface.md) for details):

To open gripper:

```bash
source install/setup.bash
ros2 action send_goal /open_gripper gripper_msgs/action/OpenGripper "{torque: 0.0, use_torque_mode: false}"
```

To close gripper:

```bash
source install/setup.bash
ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close: true, torque: 0.0, use_torque_mode: false}"
```

Read the [gripper action interface docs](docs/gripper_action_interface.md) for more details on open/close goal fields and CLI usage.
Read the [servo action interface docs](docs/servo_action_interface.md) for direct low-level servo commands.

### Current two-finger articulation model

The current two-finger Dynamixel stack uses two public slider joints:

- `gripper_planar_4`
- `gripper_planar_5`

`gripper_two_fingers_node` maps servo feedback into those joints using per-finger `joint_open_position` and `joint_close_position` values from `gripper_ros/config/grippers/soft_two_finger_dynamixel.yaml`.
`robot_state_publisher` then expands the moving TF tree from those joint states.

## Refactor status

The package-role refactor is complete:

- low-level servo packages own `/servo_control`
- gripper-level wrapper packages own `/open_gripper` and `/close_gripper`
- the two-finger Dynamixel path uses the slider-joint model with separate hardware and visualization launches

See [refactor.md](./refactor.md) for the maintained refactor record.


### Polling for Dynamixel motors

Quick ID + baudrate sweep:

```bash
python3 src/grippers/gripper_servo_dynamixel/find_id.py \
	--device /dev/ttyUSB0 \
	--baudrate-sweep 57600 115200\
	--scan-start 0 \
	--scan-end 252 \
	--no-poll
```

Quick polling of a known servo:

```bash
python3 src/grippers/gripper_servo_dynamixel/find_id.py \
	--device /dev/ttyUSB0 \
	--id 1 \
	--baudrate 57600 \
	--count 0 \
	--interval 0.5
```


## Adding new grippers

- [Adding new grippers, descriptions, configs, and launch files](./docs/adding_new_grippers.md)
- [Motor-specific configuration details for Dynamixel](./docs/dynamixel.md)
- [Motor-specific configuration details for Feetech](./docs/feetech.md)