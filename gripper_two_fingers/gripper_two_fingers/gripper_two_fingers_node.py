from __future__ import annotations
import math
from typing import Optional

from control_msgs.action import GripperCommand
import rclpy
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState

from gripper_servo_dynamixel.gripper_action_node import DynamixelServoActionNode


class GripperTwoFingersNode(DynamixelServoActionNode):
    def __init__(self) -> None:
        super().__init__(
            node_name='gripper_two_fingers_node',
            default_status_publish_rate_hz=0.0,
            enable_ros_interface=False,
        )

        self._gripper_command_server = ActionServer(
            self,
            GripperCommand,
            'gripper_command',
            execute_callback=self._execute_gripper_command,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        self.declare_parameter('publish_gripper_joint_states', False)
        self.declare_parameter('gripper_joint_state_topic', 'joint_states')
        self.declare_parameter('gripper_joint_name_prefix', '')
        self.declare_parameter('gripper_left_finger.joint_name', '')
        self.declare_parameter('gripper_left_finger.joint_open_position', float('nan'))
        self.declare_parameter('gripper_left_finger.joint_close_position', float('nan'))
        self.declare_parameter('gripper_right_finger.joint_name', '')
        self.declare_parameter('gripper_right_finger.joint_open_position', float('nan'))
        self.declare_parameter('gripper_right_finger.joint_close_position', float('nan'))
        self.declare_parameter('gripper_joint_state_rate_hz', 10.0)
        self.declare_parameter('gripper_open_width', float('nan'))
        self.declare_parameter('gripper_closed_width', 0.0)
        self.declare_parameter('gripper_position_tolerance', 0.0005)

        self._joint_state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter('gripper_joint_state_topic').value),
            10,
        )
        self._joint_state_timer = None

        if bool(self.get_parameter('publish_gripper_joint_states').value):
            rate_hz = float(self.get_parameter('gripper_joint_state_rate_hz').value)
            if rate_hz > 0.0:
                self._joint_state_timer = self.create_timer(1.0 / rate_hz, self._publish_gripper_joint_states_timer_cb)
            else:
                self._publish_gripper_joint_states_from_feedback()

        self.get_logger().info('Two-finger gripper node ready.')

    def _prefixed_joint_name(self, base_name: str) -> str:
        prefix = str(self.get_parameter('gripper_joint_name_prefix').value)
        if not prefix or base_name.startswith(prefix):
            return base_name
        return prefix + base_name

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
                    'joint_open_position': float(self.get_parameter(f'{base}.joint_open_position').value),
                    'joint_close_position': float(self.get_parameter(f'{base}.joint_close_position').value),
                }
            )

        return configs

    def _finger_joint_position(self, config: dict[str, float | str], command_position_value: Optional[float]) -> float:
        scalar = 0.0 if command_position_value is None else float(command_position_value)

        joint_open = float(config['joint_open_position'])
        joint_close = float(config['joint_close_position'])
        if math.isnan(joint_open) or math.isnan(joint_close):
            joint_name = str(config['joint_name'])
            raise RuntimeError(
                f'{joint_name} requires both joint_open_position and joint_close_position to be configured.'
            )

        command_open = float(self._servo_config.open_position)
        command_close = float(self._servo_config.close_position)
        if math.isclose(command_close, command_open):
            return joint_open

        alpha = (scalar - command_open) / (command_close - command_open)
        alpha = max(0.0, min(1.0, alpha))
        return joint_open + (alpha * (joint_close - joint_open))

    def _joint_state_entries(self, command_position_value: Optional[float]) -> list[tuple[str, float]]:
        finger_configs = self._finger_configs()
        entries: list[tuple[str, float]] = []
        for config in finger_configs:
            joint_name = str(config['joint_name'])
            entries.append(
                (
                    joint_name,
                    self._finger_joint_position(config, command_position_value),
                )
            )
        return entries

    def _publish_gripper_joint_states(self, command_position_value: Optional[float]) -> None:
        if not bool(self.get_parameter('publish_gripper_joint_states').value):
            return

        try:
            entries = self._joint_state_entries(command_position_value)
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
            return

        try:
            current_ticks = self._read_present_position()
            command_units = self._servo.ticks_to_command_units(current_ticks)
            self._publish_gripper_joint_states(command_units)
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

    def _handle_servo_unavailable_feedback(self) -> None:
        self._publish_gripper_joint_states(command_position_value=0.0)

    def _log_gripper_action_request(self, action_name: str, *, torque: float, use_torque_mode: bool) -> None:
        self.get_logger().info(
            f'{action_name} requested (torque={torque:.3f}, use_torque_mode={use_torque_mode}, safety_torque_limit={self._servo_config.safety_torque_limit:.3f})'
        )

    def _log_gripper_action_result(self, action_name: str, result_message: str, *, success: bool) -> None:
        if success:
            self.get_logger().info(f'{action_name} result: {result_message}')
        else:
            self.get_logger().warning(f'{action_name} result: {result_message}')

    def _configured_gripper_open_width(self) -> float:
        configured_width = float(self.get_parameter('gripper_open_width').value)
        if not math.isnan(configured_width):
            return configured_width

        finger_configs = self._finger_configs()
        if not finger_configs:
            raise RuntimeError('gripper_open_width must be configured when finger joint mappings are not available.')

        width = 0.0
        for config in finger_configs:
            joint_open = float(config['joint_open_position'])
            joint_close = float(config['joint_close_position'])
            if math.isnan(joint_open) or math.isnan(joint_close):
                raise RuntimeError('gripper_open_width could not be derived because finger joint mappings are incomplete.')
            width += abs(joint_open - joint_close)
        return width

    def _gripper_width_limits(self) -> tuple[float, float]:
        open_width = self._configured_gripper_open_width()
        closed_width = float(self.get_parameter('gripper_closed_width').value)
        if open_width < closed_width:
            raise RuntimeError('gripper_open_width must be greater than or equal to gripper_closed_width.')
        return open_width, closed_width

    def _gripper_width_to_command_position(self, width_m: float) -> float:
        open_width, closed_width = self._gripper_width_limits()
        requested_width = max(closed_width, min(open_width, float(width_m)))
        if math.isclose(open_width, closed_width):
            close_ratio = 0.0
        else:
            close_ratio = (open_width - requested_width) / (open_width - closed_width)

        open_position = float(self._servo_config.open_position)
        close_position = float(self._servo_config.close_position)
        return open_position + ((close_position - open_position) * close_ratio)

    def _command_position_to_gripper_width(self, command_position: float) -> float:
        open_width, closed_width = self._gripper_width_limits()
        open_position = float(self._servo_config.open_position)
        close_position = float(self._servo_config.close_position)
        if math.isclose(open_position, close_position):
            return open_width

        close_ratio = (float(command_position) - open_position) / (close_position - open_position)
        close_ratio = max(0.0, min(1.0, close_ratio))
        return open_width + ((closed_width - open_width) * close_ratio)

    def _current_gripper_width(self, fallback_width: float) -> float:
        if self._servo is None:
            return float(fallback_width)

        try:
            current_ticks = self._read_present_position()
            command_position = float(self._servo.ticks_to_command_units(current_ticks))
            return self._command_position_to_gripper_width(command_position)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f'Failed to read gripper width for action result: {type(exc).__name__}: {exc}'
            )
            return float(fallback_width)

    def _make_gripper_command_feedback(self, target_width: float, effort: float):
        def make_feedback(progress: float = 0.0) -> GripperCommand.Feedback:
            progress = max(0.0, min(1.0, float(progress)))
            current_width = self._current_gripper_width(float(target_width))
            tolerance = float(self.get_parameter('gripper_position_tolerance').value)
            feedback = GripperCommand.Feedback()
            feedback.position = current_width
            feedback.effort = float(effort)
            feedback.stalled = False
            feedback.reached_goal = progress >= 1.0 and math.isclose(current_width, float(target_width), rel_tol=0.0, abs_tol=tolerance)
            return feedback

        return make_feedback

    def _make_gripper_command_result(self, target_width: float, effort: float, action_name: str):
        def make_result(success: bool, message: str) -> GripperCommand.Result:
            stalled = 'effort' in message.lower() or 'torque' in message.lower() or 'safety' in message.lower()
            result = GripperCommand.Result()
            result.position = self._current_gripper_width(float(target_width))
            result.effort = float(effort)
            result.stalled = bool(success and stalled)
            result.reached_goal = bool(success and not stalled)
            self._log_gripper_action_result(action_name, message, success=bool(success))
            return result

        return make_result

    def _execute_gripper_command(self, goal_handle) -> GripperCommand.Result:
        goal = goal_handle.request
        target_width = float(goal.command.position)
        target_position = self._gripper_width_to_command_position(target_width)
        requested_effort = float(goal.command.max_effort)
        torque = self._resolve_torque(requested_effort)
        use_torque_mode = self._resolve_use_torque_mode(abs(requested_effort) > 0.0)
        self._log_gripper_action_request('gripper_command', torque=torque, use_torque_mode=use_torque_mode)

        return self._execute_position_goal(
            goal_handle,
            self._make_gripper_command_feedback(target_width, torque),
            self._make_gripper_command_result(target_width, torque, 'gripper_command'),
            target_position=target_position,
            torque=torque,
            use_torque_mode=use_torque_mode,
            already_message='Gripper already at requested position.',
            success_message='Gripper command sent.',
            torque_reached_message='Gripper reached the requested effort and is holding position.',
            safety_limit_message='Gripper stopped at the safety effort limit and is holding position.',
            timeout_message='Gripper command timed out or was canceled.',
            canceled_message='Gripper command was canceled.',
            failure_prefix='Gripper command failed',
        )

    def destroy_node(self) -> bool:
        self._gripper_command_server.destroy()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
