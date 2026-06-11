
# Detailed Start

### Hardware / OS prep

- Connect the motor via a U2D2 (or equivalent USB2Serial adapter).
- Ensure your user can access the serial port (often requires `dialout` group):

```bash
sudo usermod -a -G dialout $USER
```
Log out/in after changing groups.

### Set your main config

If you do not know the motor ID or baudrate yet, use the probe script to find them:

```bash
source install/setup.bash
python3 src/grippers/gripper_servo_dynamixel/find_id.py \
  --device /dev/ttyUSB0 \
  --baudrate-sweep 57600 115200 1000000 2000000 3000000 4000000 \
  --scan-start 0 \
  --scan-end 252 \
  --no-poll
```

Edit `gripper_ros/config/servos/dynamixel.yaml`:

- `motor_model`: `XM430-W350` (or your custom preset name)
- `device_name`: e.g. `/dev/ttyUSB0`
- `baudrate`: e.g. `57600` or whatever you configured on the motor
- `servo_id`: the motor ID
- `open_position` / `close_position`: your gripper’s open/close targets

## Run Servo Node (Direct Servo Control and No gripper-level articulation)

Launch via `gripper_ros`:

```bash
source install/setup.bash
ros2 launch gripper_ros dynamixel.launch.py
```

Direct servo command:

```bash
source install/setup.bash
ros2 action send_goal /servo_control gripper_msgs/action/ServoControl "{position: 900.0, torque: 0.0}"
```

See [Servo-level ROS2 interface](./servo/ros2-interface.md) for low-level servo action examples.

## Run Gripper Node (Gripper-level Open/Close Actions and Articulation)

Launch the gripper-level two-finger wrapper:

```bash
source install/setup.bash
ros2 launch gripper_ros gripper_soft_two_fingers.launch.py
```

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

See [Gripper-level ROS2 interface](./gripper/ros2-interface.md) for gripper-level open/close examples.

