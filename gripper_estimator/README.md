# Gripper Estimator

This package contains the gripper force-calibration and retention-force estimation tools.

## Gripping Force Calibration

```bash
source install/setup.bash
ros2 run gripper_estimator gripping_force_estimate --ros-args \
	-p target_position:=0.0 \
	-p release_position:=0.09
```

## Retention Force Measurement

```bash
source install/setup.bash
ros2 run gripper_estimator retention_force_estimate --ros-args -p port:=/dev/ttyACM0
```