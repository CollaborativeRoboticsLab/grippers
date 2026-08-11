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


class RetentionForceEstimateNode(Node):
	def __init__(self) -> None:
		super().__init__('retention_force_estimate')

		default_output_dir = Path.home() / 'retention_force_data'
		self.declare_parameter('port', '')
		self.declare_parameter('auto_select_port', True)
		self.declare_parameter('baudrate', 9600)
		self.declare_parameter('serial_timeout_sec', 0.2)
		self.declare_parameter('startup_delay_sec', 2.0)
		self.declare_parameter('tare_on_startup', True)
		self.declare_parameter('tare_settle_sec', 0.5)
		self.declare_parameter('poll_rate_hz', 10.0)
		self.declare_parameter('force_topic', 'retention_force/force')
		self.declare_parameter('valid_topic', 'retention_force/valid')
		self.declare_parameter('record_data', False)
		self.declare_parameter('output_dir', str(default_output_dir))
		self.declare_parameter('output_basename', 'retention_force')
		self.declare_parameter('save_plot', False)

		self._serial: Optional[Serial] = None
		self._samples: list[ForceSample] = []
		self._last_force: Optional[float] = None
		self._start_time = time.time()

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

		self._connect_serial()
		if bool(self.get_parameter('tare_on_startup').value):
			self._tare_sensor()

		poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
		if poll_rate_hz <= 0.0:
			raise RuntimeError('poll_rate_hz must be greater than 0.0.')

		self._timer = self.create_timer(1.0 / poll_rate_hz, self._poll_force)
		self.get_logger().info('Retention force estimate node started.')

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

	def _poll_force(self) -> None:
		force = self._read_force_from_arduino()
		elapsed = time.time() - self._start_time
		valid = force is not None

		if valid:
			self._last_force = force
		elif self._last_force is not None:
			force = self._last_force
		else:
			force = 0.0

		self._force_publisher.publish(Float64(data=float(force)))
		self._valid_publisher.publish(Bool(data=valid))

		if bool(self.get_parameter('record_data').value):
			self._samples.append(ForceSample(elapsed_seconds=elapsed, force_newtons=float(force), valid=valid))

		if valid:
			self.get_logger().debug(f'Force={force:.3f} N at t={elapsed:.3f}s')

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

		output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser().resolve()
		output_dir.mkdir(parents=True, exist_ok=True)
		basename = str(self.get_parameter('output_basename').value)
		timestamp = time.strftime('%Y%m%d-%H%M%S')
		csv_path = output_dir / f'{basename}_{timestamp}.csv'

		with csv_path.open('w', newline='', encoding='ascii') as handle:
			writer = csv.writer(handle)
			writer.writerow(['time_seconds', 'force_newtons', 'valid'])
			for sample in self._samples:
				writer.writerow([
					f'{sample.elapsed_seconds:.6f}',
					f'{sample.force_newtons:.6f}',
					int(sample.valid),
				])

		self.get_logger().info(f'Wrote retention force CSV to {csv_path}')

		if bool(self.get_parameter('save_plot').value):
			plot_path = output_dir / f'{basename}_{timestamp}.png'
			self._save_plot(plot_path)

	def _save_plot(self, plot_path: Path) -> None:
		figure, axis = plt.subplots(figsize=(10.0, 5.0))
		times = [sample.elapsed_seconds for sample in self._samples]
		forces = [sample.force_newtons for sample in self._samples]
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
		plt.close(figure)
		self.get_logger().info(f'Wrote retention force plot to {plot_path}')

	def destroy_node(self) -> bool:
		try:
			if bool(self.get_parameter('record_data').value):
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
		rclpy.shutdown()


if __name__ == '__main__':
	main()
