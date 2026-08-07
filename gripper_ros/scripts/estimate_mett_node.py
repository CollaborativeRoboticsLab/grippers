#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import matplotlib
if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
	matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
from control_msgs.action import GripperCommand
from matplotlib.figure import Figure
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node


@dataclass(frozen=True)
class Sample:
	index: int
	torque: float
	force: float


@dataclass(frozen=True)
class FitSummary:
	slope: float
	intercept: float
	rmse: float
	r_squared: float


class EstimateMettNode(Node):
	def __init__(self) -> None:
		super().__init__('estimate_mett_node')
		self.declare_parameter('action_name', '/gripper_command')
		self.declare_parameter('controlled_node_name', '/gripper_two_fingers_node')
		self.declare_parameter('bypass_parameter_name', 'bypass_max_effort')
		self.declare_parameter('target_position', float('nan'))
		self.declare_parameter('release_position', float('nan'))
		self.declare_parameter('start_torque', 0.1)
		self.declare_parameter('torque_increment', 0.1)
		self.declare_parameter('max_torque', 1.0)
		self.declare_parameter('output_dir', '.')
		self.declare_parameter('output_basename', 'effort_to_torque_calibration')

		self._action_name = str(self.get_parameter('action_name').value)
		self._client = ActionClient(self, GripperCommand, self._action_name)
		self._original_bypass: Optional[bool] = None

	def run(self) -> None:
		target_position = float(self.get_parameter('target_position').value)
		release_position = float(self.get_parameter('release_position').value)
		start_torque = float(self.get_parameter('start_torque').value)
		torque_increment = float(self.get_parameter('torque_increment').value)
		max_torque = float(self.get_parameter('max_torque').value)

		if math.isnan(target_position):
			raise RuntimeError('target_position must be set to the gripper command closing target used for the calibration.')
		if torque_increment <= 0.0:
			raise RuntimeError('torque_increment must be greater than 0.0.')
		if max_torque < start_torque:
			raise RuntimeError('max_torque must be greater than or equal to start_torque.')

		self._capture_original_bypass()

		try:
			self.get_logger().info(f"Waiting for action server '{self._action_name}'...")
			if not self._client.wait_for_server(timeout_sec=5.0):
				raise RuntimeError(f"Action server '{self._action_name}' is not available.")

			self._set_remote_bool_parameter(str(self.get_parameter('bypass_parameter_name').value), True)

			print('Calibration workflow')
			print('- Place the gripper against the external force gauge or scale.')
			print('- This temporarily enables bypass_max_effort on the gripper wrapper.')
			print('- Press Enter to start each torque step and press Enter again to release it.')
			print('- After release, enter the measured gripping force in newtons.')

			samples: list[Sample] = []
			sample_index = 1
			torque = start_torque
			while torque <= max_torque + 1e-9:
				ready_input = input(f'Press Enter to apply torque {torque:.4f}, or type q to quit: ').strip().lower()
				if ready_input == 'q':
					break

				self.get_logger().info(f'Applying sample {sample_index}: torque={torque:.4f}')
				result = self._send_goal(position=target_position, max_effort=torque)
				if not result.reached_goal and not result.stalled:
					self.get_logger().warning(
						f'Sample {sample_index} did not report success cleanly: reached_goal={result.reached_goal}, stalled={result.stalled}'
					)

				stop_input = input('Press Enter to release this torque, or type q to quit: ').strip().lower()
				if not math.isnan(release_position):
					self._send_goal(position=release_position, max_effort=0.0)
				if stop_input == 'q':
					break

				measured_text = input(f'Measured force for torque {torque:.4f} in N: ').strip().lower()
				if measured_text == 'q':
					break

				try:
					measured_force = float(measured_text)
				except ValueError as exc:
					raise RuntimeError(f"Invalid force value '{measured_text}'.") from exc

				samples.append(Sample(index=sample_index, torque=float(torque), force=measured_force))
				output_paths = self._write_outputs(samples)
				self.get_logger().info(f'Updated calibration CSV: {output_paths[0]}')
				self.get_logger().info(f'Updated calibration plot: {output_paths[1]}')

				figure = self._build_plot(samples)
				self._display_plot(figure)
				if self._handle_post_plot_prompt(figure, sample_index):
					plt.close(figure)
					break
				plt.close(figure)

				torque += torque_increment
				sample_index += 1

			if len(samples) < 2:
				raise RuntimeError('At least two samples are required to estimate max_effort_to_torque_factor.')

			output_paths = self._write_outputs(samples)
			self.get_logger().info(f'Wrote calibration CSV to {output_paths[0]}')
			self.get_logger().info(f'Wrote calibration plot to {output_paths[1]}')
		finally:
			self._restore_bypass()

	def _parameter_service_prefix(self) -> str:
		node_name = str(self.get_parameter('controlled_node_name').value).strip()
		if not node_name:
			raise RuntimeError('controlled_node_name must not be empty.')
		if not node_name.startswith('/'):
			node_name = '/' + node_name
		return node_name.rstrip('/')

	def _capture_original_bypass(self) -> None:
		parameter_name = str(self.get_parameter('bypass_parameter_name').value)
		self._original_bypass = self._get_remote_bool_parameter(parameter_name)

	def _restore_bypass(self) -> None:
		if self._original_bypass is None:
			return

		parameter_name = str(self.get_parameter('bypass_parameter_name').value)
		self._set_remote_bool_parameter(parameter_name, self._original_bypass)
		self.get_logger().info(
			f"Restored {self._parameter_service_prefix()}.{parameter_name}={self._original_bypass}."
		)
		self._original_bypass = None

	def _get_remote_bool_parameter(self, parameter_name: str) -> bool:
		client = self.create_client(GetParameters, f'{self._parameter_service_prefix()}/get_parameters')
		if not client.wait_for_service(timeout_sec=5.0):
			raise RuntimeError(f"Parameter service '{client.srv_name}' is not available.")

		request = GetParameters.Request()
		request.names = [parameter_name]
		future = client.call_async(request)
		rclpy.spin_until_future_complete(self, future)
		response = future.result()
		if response is None or len(response.values) != 1:
			raise RuntimeError(f"Failed to read remote parameter '{parameter_name}'.")

		value = response.values[0]
		if value.type != ParameterType.PARAMETER_BOOL:
			raise RuntimeError(f"Remote parameter '{parameter_name}' is not a bool parameter.")
		return bool(value.bool_value)

	def _set_remote_bool_parameter(self, parameter_name: str, parameter_value: bool) -> None:
		client = self.create_client(SetParameters, f'{self._parameter_service_prefix()}/set_parameters')
		if not client.wait_for_service(timeout_sec=5.0):
			raise RuntimeError(f"Parameter service '{client.srv_name}' is not available.")

		request = SetParameters.Request()
		request.parameters = [
			ParameterMsg(
				name=parameter_name,
				value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(parameter_value)),
			)
		]
		future = client.call_async(request)
		rclpy.spin_until_future_complete(self, future)
		response = future.result()
		if response is None or len(response.results) != 1 or not response.results[0].successful:
			reason = '' if response is None or len(response.results) != 1 else response.results[0].reason
			raise RuntimeError(f"Failed to set remote parameter '{parameter_name}': {reason}")

	def _send_goal(self, *, position: float, max_effort: float) -> GripperCommand.Result:
		goal = GripperCommand.Goal()
		goal.command.position = float(position)
		goal.command.max_effort = float(max_effort)

		send_future = self._client.send_goal_async(goal)
		rclpy.spin_until_future_complete(self, send_future)
		goal_handle = send_future.result()
		if goal_handle is None or not goal_handle.accepted:
			raise RuntimeError('Calibration goal was rejected by the action server.')

		result_future = goal_handle.get_result_async()
		rclpy.spin_until_future_complete(self, result_future)
		wrapped_result = result_future.result()
		if wrapped_result is None:
			raise RuntimeError('Calibration goal did not return a result.')
		return wrapped_result.result

	def _write_outputs(self, samples: list[Sample]) -> tuple[Path, Path]:
		output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser().resolve()
		output_dir.mkdir(parents=True, exist_ok=True)
		base_name = str(self.get_parameter('output_basename').value)
		csv_path = output_dir / f'{base_name}.csv'
		svg_path = output_dir / f'{base_name}.svg'

		with csv_path.open('w', newline='', encoding='ascii') as handle:
			writer = csv.writer(handle)
			writer.writerow(['sample_index', 'force_newtons', 'torque_units'])
			for sample in samples:
				writer.writerow([sample.index, f'{sample.force:.6f}', f'{sample.torque:.6f}'])

		figure = self._build_plot(samples)
		figure.savefig(svg_path, format='svg')
		plt.close(figure)

		self._print_summary(samples)
		return csv_path, svg_path

	def _build_plot(self, samples: list[Sample]) -> Figure:
		origin_fit = self._fit_through_origin(samples) if self._can_fit_through_origin(samples) else None
		affine_fit = self._fit_affine(samples) if samples else None
		forces = [sample.force for sample in samples]
		torques = [sample.torque for sample in samples]
		max_force = max(max(forces), 1.0) * 1.1
		line_forces = [0.0, max_force]

		figure, axis = plt.subplots(figsize=(8.0, 5.0))
		axis.scatter(forces, torques, color='#0f766e', label='Samples')
		if origin_fit is not None:
			axis.plot(
				line_forces,
				[origin_fit.slope * force for force in line_forces],
				color='#dc2626',
				linewidth=2.0,
				label='Through-origin fit',
			)
		if affine_fit is not None:
			axis.plot(
				line_forces,
				[(affine_fit.slope * force) + affine_fit.intercept for force in line_forces],
				color='#2563eb',
				linewidth=2.0,
				linestyle='--',
				label='Affine fit',
			)
		axis.set_title('Effort to torque calibration')
		axis.set_xlabel('Measured force (N)')
		axis.set_ylabel('Applied torque units')
		axis.grid(True, alpha=0.3)
		axis.legend(loc='best')
		axis.set_xlim(left=0.0)
		axis.set_ylim(bottom=0.0)
		figure.tight_layout()
		return figure

	def _display_plot(self, figure: Figure) -> None:
		try:
			figure.show()
			plt.show(block=False)
			plt.pause(0.001)
		except Exception as exc:  # noqa: BLE001
			self.get_logger().warning(f'Unable to display plot interactively: {exc}')

	def _handle_post_plot_prompt(self, figure: Figure, sample_index: int) -> bool:
		while True:
			choice = input(
				'Press Enter to continue, type s to save this plot, or type q to quit: '
			).strip()
			choice_lower = choice.lower()
			if choice_lower == '':
				return False
			if choice_lower == 'q':
				return True
			if choice_lower == 's' or choice_lower.startswith('s '):
				path_text = choice[1:].strip()
				default_path = self._default_interactive_plot_path(sample_index)
				if not path_text:
					path_text = input(f'Plot output path [{default_path}]: ').strip()
				path = default_path if not path_text else Path(path_text).expanduser().resolve()
				self._save_plot(figure, path)
				continue
			print("Unknown option. Use Enter to continue, 's' to save, or 'q' to quit.")

	def _default_interactive_plot_path(self, sample_index: int) -> Path:
		output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser().resolve()
		base_name = str(self.get_parameter('output_basename').value)
		return output_dir / f'{base_name}_sample_{sample_index}.svg'

	def _save_plot(self, figure: Figure, path: Path) -> None:
		path.parent.mkdir(parents=True, exist_ok=True)
		figure.savefig(path)
		self.get_logger().info(f'Saved calibration plot to {path}')

	def _print_summary(self, samples: list[Sample]) -> None:
		origin_fit = self._fit_through_origin(samples) if self._can_fit_through_origin(samples) else None
		affine_fit = self._fit_affine(samples) if samples else None
		print('Calibration summary')
		print(f'- Samples: {len(samples)}')
		if origin_fit is not None:
			print(
				f'- Through-origin factor (recommended initial max_effort_to_torque_factor): '
				f'{origin_fit.slope:.6f} torque_units_per_newton'
			)
		if affine_fit is not None:
			print(
				f'- Affine fit: torque = {affine_fit.slope:.6f} * force + {affine_fit.intercept:.6f} '
				f'(RMSE={affine_fit.rmse:.6f}, R^2={affine_fit.r_squared:.6f})'
			)

	def _can_fit_through_origin(self, samples: list[Sample]) -> bool:
		return any(not math.isclose(sample.force, 0.0) for sample in samples)

	def _fit_through_origin(self, samples: list[Sample]) -> FitSummary:
		sum_force_sq = sum(sample.force * sample.force for sample in samples)
		if math.isclose(sum_force_sq, 0.0):
			raise RuntimeError('Cannot fit a through-origin factor when all force samples are zero.')

		slope = sum(sample.force * sample.torque for sample in samples) / sum_force_sq
		residuals = [sample.torque - (slope * sample.force) for sample in samples]
		rmse = math.sqrt(sum(residual * residual for residual in residuals) / len(residuals))
		mean_torque = sum(sample.torque for sample in samples) / len(samples)
		total_sum_sq = sum((sample.torque - mean_torque) ** 2 for sample in samples)
		if math.isclose(total_sum_sq, 0.0):
			r_squared = 1.0
		else:
			residual_sum_sq = sum(residual * residual for residual in residuals)
			r_squared = 1.0 - (residual_sum_sq / total_sum_sq)
		return FitSummary(slope=slope, intercept=0.0, rmse=rmse, r_squared=r_squared)

	def _fit_affine(self, samples: list[Sample]) -> FitSummary:
		count = float(len(samples))
		sum_force = sum(sample.force for sample in samples)
		sum_torque = sum(sample.torque for sample in samples)
		sum_force_sq = sum(sample.force * sample.force for sample in samples)
		sum_force_torque = sum(sample.force * sample.torque for sample in samples)

		denominator = (count * sum_force_sq) - (sum_force * sum_force)
		if math.isclose(denominator, 0.0):
			slope = 0.0
			intercept = sum_torque / count
		else:
			slope = ((count * sum_force_torque) - (sum_force * sum_torque)) / denominator
			intercept = (sum_torque - (slope * sum_force)) / count

		residuals = [sample.torque - ((slope * sample.force) + intercept) for sample in samples]
		rmse = math.sqrt(sum(residual * residual for residual in residuals) / len(residuals))
		mean_torque = sum_torque / count
		total_sum_sq = sum((sample.torque - mean_torque) ** 2 for sample in samples)
		if math.isclose(total_sum_sq, 0.0):
			r_squared = 1.0
		else:
			residual_sum_sq = sum(residual * residual for residual in residuals)
			r_squared = 1.0 - (residual_sum_sq / total_sum_sq)
		return FitSummary(slope=slope, intercept=intercept, rmse=rmse, r_squared=r_squared)


def main(args: Optional[list[str]] = None) -> None:
	rclpy.init(args=args)
	node: Optional[EstimateMettNode] = None
	try:
		node = EstimateMettNode()
		node.run()
	except Exception as exc:  # noqa: BLE001
		rclpy.logging.get_logger('estimate_mett_node').fatal(str(exc))
		raise
	finally:
		if node is not None:
			node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


if __name__ == '__main__':
	main()
