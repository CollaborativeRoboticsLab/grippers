# Force Estimation Workflows

This document collects the gripper force-estimation and force-measurement workflows that were previously split between helper scripts and the top-level README.

## Gripping Force Calibration

Use this workflow to calibrate the relationship between gripper `max_effort` and measured gripping force.

### Purpose

The helper node steps the gripper through a sequence of torque values, waits for a manual force reading from an external gauge or scale, and updates the calibration plot after each accepted sample.

This is intended for tuning the `max_effort_to_torque_factor` used by the gripper wrapper.

### Prerequisites

- Build and source the workspace.
- Start the gripper stack with `bypass_max_effort` enabled.
- Place the gripper against an external force gauge or scale.

```bash
source install/setup.bash
ros2 launch gripper_ros gripper_soft_two_fingers.launch.py bypass_max_effort:=true
```

### Run the gripping-force helper

```bash
source install/setup.bash
ros2 run gripper_ros gripping_force_estimate.py --ros-args \
	-p target_position:=0.0 \
	-p release_position:=0.09 \
	-p start_torque:=0.2 \
	-p torque_increment:=0.2 \
	-p max_torque:=1.0
```

### Interaction flow

- Press Enter to apply the current torque.
- Press Enter again to release it.
- Enter the measured gripping force in newtons.
- Inspect the updated matplotlib plot.
- Press Enter to accept and continue to the next torque increment.
- Type `r` after release or after the plot preview to redo the current sample.
- Type `q` at the prompts to stop early.

### Important parameters

- `target_position`: Closing target used while applying torque. Required.
- `release_position`: Position commanded after each sample is released.
- `start_torque`: First torque value to test.
- `torque_increment`: Increment applied after each accepted sample.
- `max_torque`: Upper limit of the sweep.
- `output_dir`: Directory used for the CSV and plot outputs.
- `output_basename`: Base name used for generated files.

### Outputs

The helper writes:

- A CSV file with sample index, torque units, and measured force.
- A plot showing the accepted samples and fitted calibration lines.

The script requires at least two accepted samples before the calibration is considered valid.

## Retention Force Measurement

Use this workflow to record load-cell data from the Arduino-based retention-force setup.

### Purpose

The retention-force node connects to the Arduino over serial, optionally tares the sensor, performs a short test-read phase, runs a timed acquisition, publishes live force values on ROS topics, and saves the resulting CSV and plot.

### Serial setup

The current Arduino sketch is expected to stream lines in the form:

```text
force,200,0
```

The node parses the first comma-separated field as the force value in newtons.

### Run the retention-force node

```bash
source install/setup.bash
ros2 run gripper_ros retention_force_estimate.py --ros-args -p port:=/dev/ttyACM0
```

If the Arduino enumerates as a different device, replace `/dev/ttyACM0` with the visible serial path such as `/dev/ttyUSB0`.

### Default interaction flow

The node currently defaults to `interactive_mode:=true`, which restores the same operator flow as the original Arduino script:

- Show the visible serial ports.
- Ask whether to tare the sensor.
- Perform five test reads.
- Prompt for object name, trial number, and acquisition duration.
- Start a timed acquisition after pressing Enter.
- Print force readings during the run.
- Save a CSV file and plot when the run completes.

### Published ROS interfaces

- Topic `retention_force/force` with message type `std_msgs/msg/Float64`
- Topic `retention_force/valid` with message type `std_msgs/msg/Bool`
- Service `tare` with type `std_srvs/srv/Trigger`

### Important parameters

- `port`: Serial device path for the Arduino.
- `auto_select_port`: If true, use the first visible serial device when `port` is empty.
- `baudrate`: Serial baudrate. Default: `9600`.
- `serial_timeout_sec`: Serial read timeout.
- `interactive_mode`: Run the prompt-driven acquisition workflow when true.
- `tare_on_startup`: Tare automatically during node startup.
- `poll_rate_hz`: Poll rate used in non-interactive mode.
- `test_read_count`: Number of test reads shown before acquisition.
- `default_object_name`: Default object label for saved files.
- `default_trial_number`: Default trial identifier for saved files.
- `default_duration_sec`: Default acquisition duration.
- `output_dir`: Output directory. By default this is `retention_force_data` under the colcon workspace root.
- `save_plot`: Save the matplotlib plot after acquisition.
- `show_plot`: Attempt to display the plot interactively.

### Output files

For an object named `cube` and trial `1`, the saved files look like:

```text
/home/ubuntu/colcon_ws/retention_force_data/1_cube_20260811-023928.csv
/home/ubuntu/colcon_ws/retention_force_data/1_cube_20260811-023928.png
```

### Non-interactive mode

If you only want continuous ROS publishing without the prompt-driven acquisition workflow, disable interactive mode:

```bash
source install/setup.bash
ros2 run gripper_ros retention_force_estimate.py --ros-args \
	-p port:=/dev/ttyACM0 \
	-p interactive_mode:=false \
	-p record_data:=true
```

In this mode the node publishes continuously and optionally records samples for export when the node shuts down.
