# Setup

```bash
sudo apt update
sudo apt install -y python3-serial ros-${ROS_DISTRO}-dynamixel-sdk
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
```