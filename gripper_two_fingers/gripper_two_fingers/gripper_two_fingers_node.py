from __future__ import annotations
from collections.abc import Sequence
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from gripper_msgs.action import CloseGripper, OpenGripper
from rclpy.action import ActionServer
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from gripper_servo_dynamixel.gripper_action_node import DynamixelServoActionNode


class GripperTwoFingersNode(DynamixelServoActionNode):
    def __init__(self) -> None:
        super().__init__(
            node_name='gripper_two_fingers_node',
            default_status_publish_rate_hz=0.0,
            enable_ros_interface=False,
        )

        self._open_server = ActionServer(
            self,
            OpenGripper,
            'open_gripper',
            execute_callback=self._execute_open,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self._close_server = ActionServer(
            self,
            CloseGripper,
            'close_gripper',
            execute_callback=self._execute_close,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        self.declare_parameter('publish_gripper_joint_states', False)
        self.declare_parameter('gripper_joint_state_topic', 'joint_states')
        self.declare_parameter('gripper_joint_name_prefix', '')
        self.declare_parameter('gripper_frame_prefix', '')
        self.declare_parameter('gripper_left_finger.joint_name', '')
        self.declare_parameter('gripper_left_finger.multiplier', 0.0)
        self.declare_parameter('gripper_left_finger.offset', 0.0)
        self.declare_parameter('gripper_left_finger.support_parent_frame', 'gripper_mgnr09r315hm')
        self.declare_parameter('gripper_left_finger.support_child_frame', 'gripper_gripper_support_1')
        self.declare_parameter('gripper_left_finger.support_origin_xyz', [0.129112, -0.00361501, -0.0045])
        self.declare_parameter('gripper_left_finger.support_axis_xyz', [-1.0, 0.0, 0.0])
        self.declare_parameter('gripper_right_finger.joint_name', '')
        self.declare_parameter('gripper_right_finger.multiplier', 0.0)
        self.declare_parameter('gripper_right_finger.offset', 0.0)
        self.declare_parameter('gripper_right_finger.support_parent_frame', 'gripper_mgnr09r315hm_1')
        self.declare_parameter('gripper_right_finger.support_child_frame', 'gripper_gripper_support')
        self.declare_parameter('gripper_right_finger.support_origin_xyz', [0.0189804, -0.0025, 0.0045])
        self.declare_parameter('gripper_right_finger.support_axis_xyz', [1.0, 0.0, 0.0])
        self.declare_parameter('gripper_joint_state_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('gripper_joint_state_multipliers', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('gripper_joint_state_offsets', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('gripper_joint_state_rate_hz', 10.0)
        self.declare_parameter('publish_support_connectivity_tf', False)

        self._joint_state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter('gripper_joint_state_topic').value),
            10,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._joint_state_timer = None

        if bool(self.get_parameter('publish_gripper_joint_states').value):
            rate_hz = float(self.get_parameter('gripper_joint_state_rate_hz').value)
            if rate_hz > 0.0:
                self._joint_state_timer = self.create_timer(1.0 / rate_hz, self._publish_gripper_joint_states_timer_cb)
            else:
                self._publish_gripper_joint_states_from_feedback()

        self.get_logger().info('Two-finger gripper node ready.')

    def _parameter_array(self, name: str) -> list:
        value = self.get_parameter(name).value
        if value is None:
            return []
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
            return list(value)
        raise RuntimeError(f'{name} must be configured as an array parameter.')

    def _joint_state_names(self) -> list[str]:
        names = self._parameter_array('gripper_joint_state_names')
        prefix = str(self.get_parameter('gripper_joint_name_prefix').value)
        return [prefix + str(name) for name in names]

    def _joint_state_multipliers(self) -> list[float]:
        return [float(value) for value in self._parameter_array('gripper_joint_state_multipliers')]

    def _joint_state_offsets(self) -> list[float]:
        return [float(value) for value in self._parameter_array('gripper_joint_state_offsets')]

    def _frame_prefix(self) -> str:
        explicit_prefix = str(self.get_parameter('gripper_frame_prefix').value)
        if explicit_prefix:
            return explicit_prefix
        return str(self.get_parameter('gripper_joint_name_prefix').value)

    def _prefixed_joint_name(self, base_name: str) -> str:
        prefix = str(self.get_parameter('gripper_joint_name_prefix').value)
        if not prefix or base_name.startswith(prefix):
            return base_name
        return prefix + base_name

    def _prefixed_frame_name(self, base_name: str) -> str:
        prefix = self._frame_prefix()
        if not prefix or base_name.startswith(prefix):
            return base_name
        return prefix + base_name

    def _parameter_float_triplet(self, name: str) -> tuple[float, float, float]:
        values = self._parameter_array(name)
        if len(values) != 3:
            raise RuntimeError(f'{name} must contain exactly three numeric values.')
        return (float(values[0]), float(values[1]), float(values[2]))

    def _finger_configs(self) -> list[dict[str, float | str]]:
        configs: list[dict[str, float | str]] = []
        for side in ('left', 'right'):
            base = f'gripper_{side}_finger'
            joint_name = str(self.get_parameter(f'{base}.joint_name').value)
            if not joint_name:
                continue

            configs.append(
                {
                    'side': side,
                    'joint_name': self._prefixed_joint_name(joint_name),
                    'multiplier': float(self.get_parameter(f'{base}.multiplier').value),
                    'offset': float(self.get_parameter(f'{base}.offset').value),
                    'support_parent_frame': self._prefixed_frame_name(
                        str(self.get_parameter(f'{base}.support_parent_frame').value)
                    ),
                    'support_child_frame': self._prefixed_frame_name(
                        str(self.get_parameter(f'{base}.support_child_frame').value)
                    ),
                    'support_origin_xyz': self._parameter_float_triplet(f'{base}.support_origin_xyz'),
                    'support_axis_xyz': self._parameter_float_triplet(f'{base}.support_axis_xyz'),
                }
            )

        return configs

    def _connectivity_joint_names(self) -> set[str]:
        finger_configs = self._finger_configs()
        if finger_configs:
            return {str(config['joint_name']) for config in finger_configs}

        return {
            self._prefixed_joint_name('gripper_planar_4'),
            self._prefixed_joint_name('gripper_planar_4_1'),
            self._prefixed_joint_name('gripper_planar_4_2'),
            self._prefixed_joint_name('gripper_planar_5'),
            self._prefixed_joint_name('gripper_planar_5_1'),
            self._prefixed_joint_name('gripper_planar_5_2'),
        }

    def _legacy_joint_state_entries(self, command_position_value: Optional[float], *, include_connectivity: bool = True) -> list[tuple[str, float]]:
        names = self._joint_state_names()
        if not names:
            return []

        multipliers = self._joint_state_multipliers()
        offsets = self._joint_state_offsets()
        if len(multipliers) != len(names):
            raise RuntimeError(
                'publish_gripper_joint_states is enabled but gripper_joint_state_multipliers does not match gripper_joint_state_names in length.'
            )

        if offsets and len(offsets) != len(names):
            raise RuntimeError(
                'publish_gripper_joint_states is enabled but gripper_joint_state_offsets does not match gripper_joint_state_names in length.'
            )

        if not offsets:
            offsets = [0.0] * len(names)

        scalar = 0.0 if command_position_value is None else float(command_position_value)
        skipped = self._connectivity_joint_names() if not include_connectivity else set()
        entries: list[tuple[str, float]] = []
        for name, multiplier, offset in zip(names, multipliers, offsets):
            if name in skipped:
                continue
            entries.append((name, float(offset) + (float(multiplier) * scalar)))
        return entries

    def _joint_state_entries(self, command_position_value: Optional[float], *, include_connectivity: bool = True) -> list[tuple[str, float]]:
        finger_configs = self._finger_configs()
        if finger_configs:
            scalar = 0.0 if command_position_value is None else float(command_position_value)
            skipped = self._connectivity_joint_names() if not include_connectivity else set()
            entries: list[tuple[str, float]] = []
            for config in finger_configs:
                joint_name = str(config['joint_name'])
                if joint_name in skipped:
                    continue
                entries.append(
                    (
                        joint_name,
                        float(config['offset']) + (float(config['multiplier']) * scalar),
                    )
                )
            return entries

        return self._legacy_joint_state_entries(command_position_value, include_connectivity=include_connectivity)

    def _joint_state_position_map(self, command_position_value: Optional[float]) -> dict[str, float]:
        return dict(self._joint_state_entries(command_position_value, include_connectivity=True))

    def _publish_support_connectivity_tf(self, command_position_value: Optional[float]) -> None:
        if not bool(self.get_parameter('publish_support_connectivity_tf').value):
            return

        positions = self._joint_state_position_map(command_position_value)

        finger_configs = self._finger_configs()
        if finger_configs:
            specs: list[dict[str, str | tuple[float, float, float]]] = []
            for config in finger_configs:
                specs.append(
                    {
                        'joint_name': str(config['joint_name']),
                        'parent': str(config['support_parent_frame']),
                        'child': str(config['support_child_frame']),
                        'origin': tuple(config['support_origin_xyz']),
                        'axis': tuple(config['support_axis_xyz']),
                    }
                )
        else:
            specs = [
                {
                    'joint_name': self._prefixed_joint_name('gripper_planar_4'),
                    'parent': self._prefixed_frame_name('gripper_mgnr09r315hm'),
                    'child': self._prefixed_frame_name('gripper_gripper_support_1'),
                    'origin': (0.129112, -0.00361501, -0.0045),
                    'axis': (-1.0, 0.0, 0.0),
                },
                {
                    'joint_name': self._prefixed_joint_name('gripper_planar_5'),
                    'parent': self._prefixed_frame_name('gripper_mgnr09r315hm_1'),
                    'child': self._prefixed_frame_name('gripper_gripper_support'),
                    'origin': (0.0189804, -0.0025, 0.0045),
                    'axis': (1.0, 0.0, 0.0),
                },
            ]

        stamp = self.get_clock().now().to_msg()
        transforms: list[TransformStamped] = []
        for spec in specs:
            q = float(positions.get(spec['joint_name'], 0.0))

            tx = spec['origin'][0] + (spec['axis'][0] * q)
            ty = spec['origin'][1] + (spec['axis'][1] * q)
            tz = spec['origin'][2] + (spec['axis'][2] * q)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = spec['parent']
            tf_msg.child_frame_id = spec['child']
            tf_msg.transform.translation.x = tx
            tf_msg.transform.translation.y = ty
            tf_msg.transform.translation.z = tz
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = 0.0
            tf_msg.transform.rotation.w = 1.0
            transforms.append(tf_msg)

        self._tf_broadcaster.sendTransform(transforms)

    def _publish_gripper_joint_states(self, command_position_value: Optional[float]) -> None:
        if not bool(self.get_parameter('publish_gripper_joint_states').value):
            return

        try:
            entries = self._joint_state_entries(
                command_position_value,
                include_connectivity=not bool(self.get_parameter('publish_support_connectivity_tf').value),
            )
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return

        if not entries:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [name for name, _ in entries]
        msg.position = [position for _, position in entries]
        self._joint_state_pub.publish(msg)

    def _publish_gripper_joint_states_from_feedback(self) -> None:
        if not bool(self.get_parameter('publish_gripper_joint_states').value):
            return

        if self._servo is None:
            self._publish_gripper_joint_states(command_position_value=0.0)
            self._publish_support_connectivity_tf(command_position_value=0.0)
            return

        try:
            current_ticks = self._read_present_position()
            command_units = self._servo.ticks_to_command_units(current_ticks)
            self._publish_gripper_joint_states(command_units)
            self._publish_support_connectivity_tf(command_units)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f'Failed to publish gripper joint states from feedback: {type(exc).__name__}: {exc}'
            )

    def _publish_gripper_joint_states_timer_cb(self) -> None:
        self._publish_gripper_joint_states_from_feedback()

    def _handle_position_feedback(self, current_ticks: int) -> None:
        if self._servo is None:
            return
        command_units = self._servo.ticks_to_command_units(current_ticks)
        self._publish_gripper_joint_states(command_units)
        self._publish_support_connectivity_tf(command_units)

    def _handle_servo_unavailable_feedback(self) -> None:
        self._publish_gripper_joint_states(command_position_value=0.0)
        self._publish_support_connectivity_tf(command_position_value=0.0)

    def _execute_open(self, goal_handle) -> OpenGripper.Result:
        goal = goal_handle.request
        return self._execute_position_goal(
            goal_handle,
            OpenGripper.Feedback,
            OpenGripper.Result,
            target_position=float(self._servo_config.open_position),
            torque=self._resolve_torque(float(goal.torque)),
            use_torque_mode=self._resolve_use_torque_mode(bool(goal.use_torque_mode)),
            already_message='Servo already at open position.',
            success_message='Open command sent.',
            timeout_message='Open timed out or was canceled.',
            failure_prefix='Open failed',
        )

    def _execute_close(self, goal_handle) -> CloseGripper.Result:
        goal = goal_handle.request
        close_requested = bool(goal.close) or bool(self._servo_config.close_default)
        if not close_requested:
            goal_handle.succeed()
            return CloseGripper.Result(success=True, message='Close goal flag was false; no action taken.')

        return self._execute_position_goal(
            goal_handle,
            CloseGripper.Feedback,
            CloseGripper.Result,
            target_position=float(self._servo_config.close_position),
            torque=self._resolve_torque(float(goal.torque)),
            use_torque_mode=self._resolve_use_torque_mode(bool(goal.use_torque_mode)),
            already_message='Servo already at close position.',
            success_message='Close command sent.',
            timeout_message='Close timed out or was canceled.',
            failure_prefix='Close failed',
        )

    def destroy_node(self) -> bool:
        self._open_server.destroy()
        self._close_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[GripperTwoFingersNode] = None
    try:
        node = GripperTwoFingersNode()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('gripper_two_fingers_node').fatal(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
