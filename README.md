# grippers

ROS 2 gripper drivers that expose a **common action-based API** for opening/closing a custom built gripper.

## Packages

- `gripper_msgs`: ROS 2 action definitions (`OpenGripper`, `CloseGripper`).
- `gripper_dynamixel`: DynamixelSDK (Protocol 2.0) based action server (Python).
- `gripper_feetech`: Feetech STS/SCS based action server (C++).
- `gripper_ros`: launch files + centralized motor/gripper parameter YAMLs.

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

## Testing Motors

### Dynamixel

- Motor config: `gripper_ros/config/motors/dynamixel.yaml`
- Gripper config example: `gripper_ros/config/grippers/soft_two_finger_dynamixel.yaml`
- Probe / scan script: `gripper_dynamixel/find_id.py`
- Motor-only launch:

```bash
ros2 launch gripper_ros dyanmixel.launch.py
```

Override motor params file:

```bash
ros2 launch gripper_ros dyanmixel.launch.py params_file:=/abs/path/to/dynamixel.yaml
```

- Gripper-level launch:

```bash
ros2 launch gripper_ros gripper_soft_two_finger.launch.py
```

Override both parameter layers:

```bash
ros2 launch gripper_ros gripper_soft_two_finger.launch.py \
	motor_params_file:=/abs/path/to/dynamixel.yaml \
	gripper_params_file:=/abs/path/to/soft_two_finger_dynamixel.yaml
```

See [docs/dynamixel.md](docs/dynamixel.md) for motor model presets and parameter details.

Quick ID + baudrate sweep:

```bash
python3 src/grippers/gripper_dynamixel/find_id.py \
	--device /dev/ttyUSB0 \
	--baudrate-sweep 57600 115200 1000000 2000000 3000000 4000000 \
	--scan-start 0 \
	--scan-end 252 \
	--no-poll
```

Quick polling of a known servo:

```bash
python3 src/grippers/gripper_dynamixel/find_id.py \
	--device /dev/ttyUSB0 \
	--id 1 \
	--baudrate 57600 \
	--count 0 \
	--interval 0.5
```

If a USB serial adapter is unplugged and replugged, the device path may change from `/dev/ttyUSB0` to `/dev/ttyUSB1` or similar. Recheck `/dev/ttyUSB*` before assuming the motor settings changed.

### Feetech (STS/SCS)

- Motor config: `gripper_ros/config/motors/feetech.yaml`
- Launch:

```bash
ros2 launch gripper_ros feetech.launch.py
```

Override params file:

```bash
ros2 launch gripper_ros feetech.launch.py params_file:=/abs/path/to/feetech.yaml
```

See [docs/feetech.md](docs/feetech.md) for parameter details.


## Running grippers

## Adding new grippers

- [Adding new grippers, descriptions, configs, and launch files](./docs/adding_new_grippers.md)
- [Motor-specific configuration details for Dynamixel](./docs/dynamixel.md)
- [Motor-specific configuration details for Feetech](./docs/feetech.md)