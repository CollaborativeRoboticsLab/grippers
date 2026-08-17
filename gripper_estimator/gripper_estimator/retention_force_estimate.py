#!/usr/bin/env python3

from __future__ import annotations

import csv
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import matplotlib
if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
	matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from serial import Serial, SerialException
import serial.tools.list_ports
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class ForceSample:
	elapsed_seconds: float
	force_newtons: float
	valid: bool


@dataclass(frozen=True)
class AcquisitionResult:
	samples: list[ForceSample]
	success_count: int
	total_count: int


def list_serial_ports() -> list[str]:
	return [port.device for port in serial.tools.list_ports.comports()]


def parse_force_line(raw_bytes: bytes) -> Optional[float]:
	if not raw_bytes:
		return None

	try:
		line = raw_bytes.decode('utf-8', errors='ignore').strip()
		parts = line.split(',')
		if len(parts) != 3:
			return None
		return float(parts[0])
	except ValueError:
		return None


def detect_workspace_root() -> Path:
	cwd = Path.cwd().resolve()
	if (cwd / 'src').is_dir():
		return cwd

	for variable_name in ('COLCON_PREFIX_PATH', 'AMENT_PREFIX_PATH'):
		prefixes = os.environ.get(variable_name, '')
		for prefix in prefixes.split(os.pathsep):
			if not prefix:
				continue
			candidate = Path(prefix).expanduser().resolve().parent
			if (candidate / 'src').is_dir():
				return candidate

	for parent in Path(__file__).resolve().parents:
		if (parent / 'src').is_dir() and (parent / 'install').is_dir():
			return parent

	return cwd


class RetentionForceEstimateNode(Node):
	def __init__(self) -> None:
		super().__init__('retention_force_estimate')

		default_output_dir = detect_workspace_root() / 'retention_force_data'
		self.declare_parameter('port', '')
		self.declare_parameter('auto_select_port', True)
		self.declare_parameter('baudrate', 9600)
		self.declare_parameter('serial_timeout_sec', 0.2)
		self.declare_parameter('startup_delay_sec', 2.0)
		self.declare_parameter('interactive_mode', True)
		self.declare_parameter('tare_on_startup', False)
		self.declare_parameter('tare_settle_sec', 0.5)
		self.declare_parameter('poll_rate_hz', 10.0)
		self.declare_parameter('test_read_count', 5)
		self.declare_parameter('test_read_delay_sec', 0.2)
		self.declare_parameter('default_object_name', 'test_object')
		self.declare_parameter(
			'default_trial_number',
			'001',
			ParameterDescriptor(dynamic_typing=True),
		)
		self.declare_parameter('default_duration_sec', 10.0)
		self.declare_parameter('force_topic', 'retention_force/force')
		self.declare_parameter('valid_topic', 'retention_force/valid')
		self.declare_parameter('record_data', False)
		self.declare_parameter('wait_for_start_trigger', False)
		self.declare_parameter('output_dir', str(default_output_dir))
		self.declare_parameter('output_basename', 'retention_force')
		self.declare_parameter('save_plot', True)
		self.declare_parameter('show_plot', True)

		self._serial: Optional[Serial] = None
		self._samples: list[ForceSample] = []
		self._last_force: Optional[float] = None
		self._start_time = time.time()
		self._serial_port = ''
		self._timer = None
		self._recording_active = False
		self._recording_saved = False

		self._force_publisher = self.create_publisher(
			Float64,
			str(self.get_parameter('force_topic').value),
			10,
		)
		self._valid_publisher = self.create_publisher(
			Bool,
			str(self.get_parameter('valid_topic').value),
			10,
		)
		self.create_service(Trigger, 'tare', self._handle_tare)
		self.create_service(Trigger, 'trigger_retention_recording', self._handle_start_recording)

		self._connect_serial()
		if bool(self.get_parameter('tare_on_startup').value):
			self._tare_sensor()
		self._configure_recording_state_for_mode()

		if not self.is_interactive_mode():
			poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
			if poll_rate_hz <= 0.0:
				raise RuntimeError('poll_rate_hz must be greater than 0.0.')

			self._timer = self.create_timer(1.0 / poll_rate_hz, self._poll_force)
		self.get_logger().info('Retention force estimate node started.')

	def is_interactive_mode(self) -> bool:
		return bool(self.get_parameter('interactive_mode').value)

	def _configure_recording_state_for_mode(self) -> None:
		if self.is_interactive_mode():
			self._recording_active = False
			return

		if not bool(self.get_parameter('record_data').value):
			self._recording_active = False
			return

		self._recording_active = not bool(self.get_parameter('wait_for_start_trigger').value)

	def _start_recording_window(self) -> None:
		self._samples = []
		self._last_force = None
		self._start_time = time.time()
		self._recording_active = True
		self._recording_saved = False

	def _handle_start_recording(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
		if not bool(self.get_parameter('record_data').value):
			response.success = False
			response.message = 'record_data is disabled.'
			return response

		self._start_recording_window()
		response.success = True
		response.message = 'Retention-force recording started.'
		self.get_logger().info(response.message)
		return response

	def _connect_serial(self) -> None:
		configured_port = str(self.get_parameter('port').value).strip()
		auto_select_port = bool(self.get_parameter('auto_select_port').value)
		available_ports = list_serial_ports()

		if configured_port:
			port = configured_port
		elif auto_select_port and available_ports:
			port = available_ports[0]
			self.get_logger().info(f"No port configured, auto-selected '{port}'.")
		else:
			raise RuntimeError(
				'No serial port configured and auto_select_port is disabled or no ports are available.'
			)

		try:
			self._serial = Serial(
				port=port,
				baudrate=int(self.get_parameter('baudrate').value),
				timeout=float(self.get_parameter('serial_timeout_sec').value),
			)
		except SerialException as exc:
			raise RuntimeError(f"Failed to open serial port '{port}': {exc}") from exc

		time.sleep(float(self.get_parameter('startup_delay_sec').value))
		self._serial.reset_input_buffer()
		self._serial_port = port
		self.get_logger().info(f"Connected to Arduino on '{port}'.")

	def _handle_tare(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
		try:
			self._tare_sensor()
			response.success = True
			response.message = 'Sensor tared.'
		except RuntimeError as exc:
			response.success = False
			response.message = str(exc)
		return response

	def _tare_sensor(self) -> None:
		if self._serial is None:
			raise RuntimeError('Serial connection is not available.')

		self._serial.write(b't')
		time.sleep(float(self.get_parameter('tare_settle_sec').value))
		while self._serial.in_waiting:
			self._serial.readline()
		self.get_logger().info('Sensor tared.')

	def run_interactive(self) -> None:
		available_ports = list_serial_ports()
		print('=' * 70)
		print(' Arduino Load Cell Data Acquisition System')
		print(' Config: DOUT=3, CLK=2, Calibration=-148.631')
		print('=' * 70)
		print('\nVisible serial ports:')
		if available_ports:
			for index, port in enumerate(available_ports):
				marker = ' (connected)' if port == self._serial_port else ''
				print(f'  [{index}] {port}{marker}')
		else:
			print('  No serial ports reported by pyserial.')

		if input('\nTare the sensor? (y/n, default: y): ').strip().lower() != 'n':
			self._tare_sensor()

		self._run_test_reads()
		object_name, trial_number, duration = self._prompt_run_configuration()

		print(f'\nTest configuration:')
		print(f'  Object: {object_name}')
		print(f'  Trial: {trial_number}')
		print(f'  Duration: {duration:g}s')
		print(f"  Save path: {Path(str(self.get_parameter('output_dir').value)).expanduser().resolve()}")

		input('\nPress Enter to start data acquisition...\n')
		result = self._collect_interactive_samples(duration)
		csv_path, plot_path = self._write_session_outputs(trial_number, object_name, result.samples)
		self._print_summary(result, csv_path, plot_path)

	def _run_test_reads(self) -> None:
		print('\nTest reading data (5 times)...')
		read_count = max(0, int(self.get_parameter('test_read_count').value))
		delay_sec = max(0.0, float(self.get_parameter('test_read_delay_sec').value))
		for index in range(read_count):
			sample = self._acquire_sample(start_time=time.time(), publish=True)
			if sample.valid:
				print(f'  Reading {index + 1}: {sample.force_newtons:.2f} N')
			time.sleep(delay_sec)

	def _prompt_run_configuration(self) -> tuple[str, str, float]:
		default_object_name = str(self.get_parameter('default_object_name').value)
		default_trial_number = str(self.get_parameter('default_trial_number').value)
		default_duration = float(self.get_parameter('default_duration_sec').value)

		print('\n' + '=' * 70)
		object_name = input(f'Test object name (default: {default_object_name}): ').strip()
		trial_number = input(f'Trial number (default: {default_trial_number}): ').strip()
		duration_text = input(f'Acquisition duration (s, default: {default_duration:g}): ').strip()

		object_name = object_name if object_name else default_object_name
		trial_number = trial_number if trial_number else default_trial_number
		duration = float(duration_text) if duration_text else default_duration
		if duration <= 0.0:
			raise RuntimeError('Acquisition duration must be greater than 0.0 seconds.')
		return object_name, trial_number, duration

	def _collect_interactive_samples(self, duration: float) -> AcquisitionResult:
		print('=' * 70)
		print('Acquiring data... (press Ctrl+C to stop early)')
		print('=' * 70)

		samples: list[ForceSample] = []
		success_count = 0
		total_count = 0
		start_time = time.time()

		try:
			while time.time() - start_time < duration:
				sample = self._acquire_sample(start_time=start_time, publish=True)
				samples.append(sample)
				total_count += 1
				if sample.valid:
					success_count += 1
					print(f'Time: {sample.elapsed_seconds:.2f}s, Force: {sample.force_newtons:.2f}N')

			print('\n' + '=' * 70)
			print('Data acquisition complete!')
			print('=' * 70)
		except KeyboardInterrupt:
			print('\n\nAcquisition interrupted by user')
			print('=' * 70)

		return AcquisitionResult(samples=samples, success_count=success_count, total_count=total_count)

	def _poll_force(self) -> None:
		sample = self._acquire_sample(start_time=self._start_time, publish=True)
		if bool(self.get_parameter('record_data').value) and self._recording_active:
			self._samples.append(sample)
			default_duration_sec = float(self.get_parameter('default_duration_sec').value)
			if default_duration_sec > 0.0 and sample.elapsed_seconds >= default_duration_sec:
				self._recording_active = False
				if not self._recording_saved:
					self._write_outputs()
					self._recording_saved = True
		if sample.valid:
			self.get_logger().debug(f'Force={sample.force_newtons:.3f} N at t={sample.elapsed_seconds:.3f}s')

	def _acquire_sample(self, *, start_time: float, publish: bool) -> ForceSample:
		force = self._read_force_from_arduino()
		elapsed = time.time() - start_time
		valid = force is not None

		if valid:
			measured_force = float(force)
			self._last_force = measured_force
		elif self._last_force is not None:
			measured_force = float(self._last_force)
		else:
			measured_force = 0.0

		sample = ForceSample(elapsed_seconds=elapsed, force_newtons=measured_force, valid=valid)
		if publish:
			self._force_publisher.publish(Float64(data=sample.force_newtons))
			self._valid_publisher.publish(Bool(data=sample.valid))
		return sample

	def _read_force_from_arduino(self) -> Optional[float]:
		if self._serial is None:
			return None

		latest_raw = None
		while self._serial.in_waiting:
			latest_raw = self._serial.readline()

		if latest_raw is None:
			latest_raw = self._serial.readline()

		return parse_force_line(latest_raw)

	def _write_outputs(self) -> None:
		if not self._samples:
			return
		self._write_session_outputs(
			trial_number=str(self.get_parameter('default_trial_number').value),
			object_name=str(self.get_parameter('default_object_name').value),
			samples=self._samples,
		)

	def _write_session_outputs(self, trial_number: str, object_name: str, samples: list[ForceSample]) -> tuple[Path, Optional[Path]]:
		if not samples:
			raise RuntimeError('No valid data collected.')

		output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser().resolve()
		output_dir.mkdir(parents=True, exist_ok=True)
		timestamp = time.strftime('%Y%m%d-%H%M%S')
		filename = f'{trial_number}_{object_name}_{timestamp}'
		csv_path = output_dir / f'{filename}.csv'

		with csv_path.open('w', newline='', encoding='ascii') as handle:
			writer = csv.writer(handle)
			writer.writerow(['Time(s)', 'Force(N)', 'Valid'])
			for sample in samples:
				writer.writerow([
					f'{sample.elapsed_seconds:.6f}',
					f'{sample.force_newtons:.6f}',
					int(sample.valid),
				])

		self.get_logger().info(f'Wrote retention force CSV to {csv_path}')

		plot_path: Optional[Path] = None
		if bool(self.get_parameter('save_plot').value):
			plot_path = output_dir / f'{filename}.png'
			self._save_plot(samples, plot_path)
		return csv_path, plot_path

	def _save_plot(self, samples: list[ForceSample], plot_path: Path) -> None:
		figure, axis = plt.subplots(figsize=(10.0, 5.0))
		times = [sample.elapsed_seconds for sample in samples]
		forces = [sample.force_newtons for sample in samples]
		axis.plot(times, forces, color='#2563eb', linewidth=2.0)
		axis.set_title('Retention force measurement')
		axis.set_xlabel('Time (s)')
		axis.set_ylabel('Force (N)')
		axis.grid(True, alpha=0.3)
		if forces:
			max_force = max(forces)
			max_index = forces.index(max_force)
			axis.scatter([times[max_index]], [max_force], color='#dc2626', label=f'Max {max_force:.2f} N')
			axis.legend(loc='best')
		figure.tight_layout()
		figure.savefig(plot_path, dpi=150, bbox_inches='tight')
		if bool(self.get_parameter('show_plot').value):
			try:
				figure.show()
				plt.show(block=False)
				plt.pause(0.001)
			except Exception as exc:  # noqa: BLE001
				self.get_logger().warning(f'Unable to display plot interactively: {exc}')
		plt.close(figure)
		self.get_logger().info(f'Wrote retention force plot to {plot_path}')

	def _print_summary(self, result: AcquisitionResult, csv_path: Path, plot_path: Optional[Path]) -> None:
		if not result.samples:
			print('\nNo valid data collected')
			return

		forces = [sample.force_newtons for sample in result.samples]
		test_duration = result.samples[-1].elapsed_seconds
		success_rate = 0.0 if result.total_count == 0 else (result.success_count / result.total_count * 100.0)

		print('\n' + '=' * 70)
		print('Test summary:')
		print(f'  Data points collected: {len(result.samples)}')
		print(f'  Success rate: {result.success_count}/{result.total_count} ({success_rate:.1f}%)')
		print(f'  Test duration: {test_duration:.2f} s')
		print(f'  Max force: {max(forces):.2f} N')
		print(f'  Average force: {sum(forces) / len(forces):.2f} N')
		print(f'  Min force: {min(forces):.2f} N')
		print(f'  CSV file: {csv_path}')
		if plot_path is not None:
			print(f'  Plot file: {plot_path}')
		print('=' * 70)

	def destroy_node(self) -> bool:
		try:
			if bool(self.get_parameter('record_data').value) and self._samples and not self._recording_saved:
				self._write_outputs()
		finally:
			if self._serial is not None and self._serial.is_open:
				self._serial.close()
				self.get_logger().info('Closed serial port.')
		return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
	rclpy.init(args=args)
	node: Optional[RetentionForceEstimateNode] = None
	try:
		node = RetentionForceEstimateNode()
		if node.is_interactive_mode():
			node.run_interactive()
		else:
			rclpy.spin(node)
	except KeyboardInterrupt:
		pass
		
	except Exception as exc:
		if node is not None:
			node.get_logger().error(str(exc))
		else:
			print(f'retention_force_estimate failed: {exc}')
		raise
	finally:
		if node is not None:
			node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


if __name__ == '__main__':
	main()