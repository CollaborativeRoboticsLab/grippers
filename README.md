# grippers

ROS 2 gripper drivers that expose a **common action-based API** for opening/closing a custom built gripper.

## Packages

- `gripper_msgs`: ROS 2 action definitions (`OpenGripper`, `CloseGripper`).
- `gripper_dynamixel`: DynamixelSDK (Protocol 2.0) based action server (Python).
- `gripper_feetech`: Feetech STS/SCS based action server (C++).
- `gripper_ros`: launch files + centralized parameter YAMLs.

## Action API

All implementations aim to expose the same action names:

- `/open_gripper` (`gripper_msgs/action/OpenGripper`)
- `/close_gripper` (`gripper_msgs/action/CloseGripper`)

See [docs/action_interface.md](docs/action_interface.md) for goal/result/feedback fields and CLI examples.

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

## Running

### Dynamixel

- Config: `gripper_ros/config/dynamixel.yaml`
- Launch:

```bash
ros2 launch gripper_ros dyanmixel.launch.py
```

Override params file:

```bash
ros2 launch gripper_ros dyanmixel.launch.py params_file:=/abs/path/to/dynamixel.yaml
```

See [docs/dynamixel.md](docs/dynamixel.md) for motor model presets and parameter details.

### Feetech (STS/SCS)

- Config: `gripper_ros/config/feetech.yaml`
- Launch:

```bash
ros2 launch gripper_ros feetech.launch.py
```

Override params file:

```bash
ros2 launch gripper_ros feetech.launch.py params_file:=/abs/path/to/feetech.yaml
```

See [docs/feetech.md](docs/feetech.md) for parameter details.

## Docs

- [docs/action_interface.md](docs/action_interface.md)
- [docs/dynamixel.md](docs/dynamixel.md)
- [docs/feetech.md](docs/feetech.md)