
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <gripper_msgs/action/open_gripper.hpp>
#include <gripper_msgs/action/close_gripper.hpp>
#include <gripper_msgs/action/servo_control.hpp>

// Ensure HardwareSerial is defined for the STS driver.
#include "gripper_servo_feetech/Feetech-STSServo/linux_serial.hpp"
#include "gripper_servo_feetech/Feetech-STSServo/STSServoDriver.hpp"

namespace gripper_servo_feetech
{

class FeetechGripperActionNode : public rclcpp::Node
{
public:
	explicit FeetechGripperActionNode(const rclcpp::NodeOptions & options)
	: FeetechGripperActionNode("gripper_servo_feetech_action_node", options, true)
	{
	}

	explicit FeetechGripperActionNode(
		const std::string & node_name = "gripper_servo_feetech_action_node",
		const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
		bool enable_ros_interface = true)
	: rclcpp::Node(node_name, options)
	{
		// Transport
		this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
		this->declare_parameter<int>("baudrate", 1000000);
		this->declare_parameter<int>("servo_id", 1);

		// Targets
		this->declare_parameter<double>("open_position", 0.0);
		this->declare_parameter<double>("close_position", 2048.0);

		// Units/scaling
		this->declare_parameter<bool>("position_is_radians", false);
		this->declare_parameter<int>("ticks_per_rev", 4096);
		this->declare_parameter<int>("direction", 1);
		this->declare_parameter<int>("zero_offset_ticks", 0);

		// Motion behavior
		this->declare_parameter<int>("speed", 4095);
		this->declare_parameter<int>("goal_tolerance_ticks", 20);
		this->declare_parameter<double>("motion_timeout_sec", 3.0);
		this->declare_parameter<double>("poll_rate_hz", 30.0);

		// Torque limiting behavior (used as an approximation of torque-mode)
		this->declare_parameter<bool>("use_torque_mode", false);
		this->declare_parameter<double>("default_torque_limit", 0.0);
		this->declare_parameter<int>("torque_limit_register", static_cast<int>(STSRegisters::TORQUE_LIMIT));
		this->declare_parameter<bool>("close_default", true);

		// Optional gripper-level joint-state publication. This bridges the articulated
		// mechanism into robot_state_publisher without hard-coding a specific linkage.
		this->declare_parameter<bool>("publish_gripper_joint_states", false);
		this->declare_parameter<std::string>("gripper_joint_state_topic", "joint_states");
		this->declare_parameter<std::string>("gripper_joint_name_prefix", "");
		this->declare_parameter<std::vector<std::string>>("gripper_joint_state_names", std::vector<std::string>{});
		this->declare_parameter<std::vector<double>>("gripper_joint_state_multipliers", std::vector<double>{});
		this->declare_parameter<std::vector<double>>("gripper_joint_state_offsets", std::vector<double>{});
		this->declare_parameter<double>("gripper_joint_state_rate_hz", 10.0);

		joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
			this->get_parameter("gripper_joint_state_topic").as_string(), 10);

		if (enable_ros_interface) {
			servo_control_server_ = rclcpp_action::create_server<ServoControl>(
				this,
				"servo_control",
				std::bind(&FeetechGripperActionNode::handle_goal_servo_control, this, std::placeholders::_1, std::placeholders::_2),
				std::bind(&FeetechGripperActionNode::handle_cancel_servo_control, this, std::placeholders::_1),
				std::bind(&FeetechGripperActionNode::handle_accepted_servo_control, this, std::placeholders::_1));
		}

		if (this->get_parameter("publish_gripper_joint_states").as_bool()) {
			const double rate_hz = this->get_parameter("gripper_joint_state_rate_hz").as_double();
			if (rate_hz > 0.0) {
				joint_state_timer_ = this->create_wall_timer(
					std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_hz)),
					std::bind(&FeetechGripperActionNode::publish_gripper_joint_states_timer_cb, this));
			} else {
				publish_gripper_joint_states_from_feedback();
			}
		}

		RCLCPP_INFO(this->get_logger(), "Feetech gripper action node ready.");
	}

	~FeetechGripperActionNode() override = default;


protected:
	using OpenGripper = gripper_msgs::action::OpenGripper;
	using CloseGripper = gripper_msgs::action::CloseGripper;
	using ServoControl = gripper_msgs::action::ServoControl;
	using GoalHandleOpen = rclcpp_action::ServerGoalHandle<OpenGripper>;
	using GoalHandleClose = rclcpp_action::ServerGoalHandle<CloseGripper>;
	using GoalHandleServoControl = rclcpp_action::ServerGoalHandle<ServoControl>;

	rclcpp_action::GoalResponse handle_goal_servo_control(
		const rclcpp_action::GoalUUID &, std::shared_ptr<const ServoControl::Goal>)
	{
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handle_cancel_servo_control(const std::shared_ptr<GoalHandleServoControl>)
	{
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted_servo_control(const std::shared_ptr<GoalHandleServoControl> goal_handle)
	{
		std::thread{std::bind(&FeetechGripperActionNode::execute_servo_control, this, goal_handle)}.detach();
	}

	bool resolve_use_torque_mode(bool goal_flag) const
	{
		return goal_flag || this->get_parameter("use_torque_mode").as_bool();
	}

	bool resolve_servo_control_use_torque_mode(double requested_torque) const
	{
		return resolve_use_torque_mode(std::abs(requested_torque) > 0.0);
	}

	bool is_at_target(int current_ticks, int target_ticks, int tolerance_ticks) const
	{
		return std::abs(target_ticks - current_ticks) <= tolerance_ticks;
	}

	int resolve_torque_limit(double requested) const
	{
		double v = requested;
		if (std::abs(v) <= 0.0) {
			v = this->get_parameter("default_torque_limit").as_double();
		}
		return static_cast<int>(std::lround(v));
	}

	int position_to_ticks(double position_value) const
	{
		if (!this->get_parameter("position_is_radians").as_bool()) {
			return static_cast<int>(std::lround(position_value));
		}

		constexpr double two_pi = 6.2831853071795864769;
		const double ticks_per_rev = static_cast<double>(this->get_parameter("ticks_per_rev").as_int());
		const int direction = this->get_parameter("direction").as_int();
		const int zero = this->get_parameter("zero_offset_ticks").as_int();

		const double ticks_per_rad = ticks_per_rev / two_pi;
		return static_cast<int>(
			std::lround(static_cast<double>(zero) + static_cast<double>(direction) * (position_value * ticks_per_rad)));
	}

	double ticks_to_command_units(int position_ticks) const
	{
		if (!this->get_parameter("position_is_radians").as_bool()) {
			return static_cast<double>(position_ticks);
		}

		constexpr double two_pi = 6.2831853071795864769;
		const double ticks_per_rev = static_cast<double>(this->get_parameter("ticks_per_rev").as_int());
		const int direction = this->get_parameter("direction").as_int();
		const int zero = this->get_parameter("zero_offset_ticks").as_int();
		const double ticks_per_rad = ticks_per_rev / two_pi;

		if (ticks_per_rad == 0.0 || direction == 0) {
			return 0.0;
		}

		return (static_cast<double>(position_ticks) - static_cast<double>(zero)) /
			(static_cast<double>(direction) * ticks_per_rad);
	}

	std::vector<std::string> joint_state_names() const
	{
		const auto prefix = this->get_parameter("gripper_joint_name_prefix").as_string();
		auto names = this->get_parameter("gripper_joint_state_names").as_string_array();
		for (auto & name : names) {
			name = prefix + name;
		}
		return names;
	}

	std::vector<double> joint_state_multipliers() const
	{
		return this->get_parameter("gripper_joint_state_multipliers").as_double_array();
	}

	std::vector<double> joint_state_offsets() const
	{
		return this->get_parameter("gripper_joint_state_offsets").as_double_array();
	}

	void publish_gripper_joint_states(double command_position_value)
	{
		if (!this->get_parameter("publish_gripper_joint_states").as_bool()) {
			return;
		}

		const auto names = joint_state_names();
		if (names.empty()) {
			return;
		}

		const auto multipliers = joint_state_multipliers();
		auto offsets = joint_state_offsets();

		if (multipliers.size() != names.size()) {
			RCLCPP_ERROR(
				this->get_logger(),
				"publish_gripper_joint_states is enabled but gripper_joint_state_multipliers does not match gripper_joint_state_names in length.");
			return;
		}

		if (!offsets.empty() && offsets.size() != names.size()) {
			RCLCPP_ERROR(
				this->get_logger(),
				"publish_gripper_joint_states is enabled but gripper_joint_state_offsets does not match gripper_joint_state_names in length.");
			return;
		}

		if (offsets.empty()) {
			offsets.assign(names.size(), 0.0);
		}

		sensor_msgs::msg::JointState msg;
		msg.header.stamp = this->get_clock()->now();
		msg.name = names;
		msg.position.reserve(names.size());
		for (std::size_t index = 0; index < names.size(); ++index) {
			msg.position.push_back(offsets[index] + (multipliers[index] * command_position_value));
		}

		joint_state_pub_->publish(msg);
	}

	void publish_gripper_joint_states_from_feedback()
	{
		if (!this->get_parameter("publish_gripper_joint_states").as_bool()) {
			return;
		}

		try {
			ensure_connected();
			const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
			const int pos = driver_->getCurrentPosition(servo_id);
			publish_gripper_joint_states(ticks_to_command_units(pos));
		} catch (const std::exception & e) {
			RCLCPP_WARN(this->get_logger(), "Failed to publish gripper joint states from feedback: %s", e.what());
		}
	}

	void publish_gripper_joint_states_timer_cb()
	{
		publish_gripper_joint_states_from_feedback();
	}

	void ensure_connected()
	{
		if (driver_) {
			return;
		}

		const auto device_name = this->get_parameter("device_name").as_string();
		const auto baudrate = this->get_parameter("baudrate").as_int();
		const auto servo_id = this->get_parameter("servo_id").as_int();

		// The vendored LinuxSerial currently configures B1000000 regardless of the requested baudrate.
		if (baudrate != 1000000) {
			RCLCPP_WARN(
				this->get_logger(),
				"linux_serial.hpp forces 1000000 baud; configured baudrate=%ld will be ignored.",
				static_cast<long>(baudrate));
		}

		serial_ = std::make_unique<LinuxSerial>(device_name.c_str());
		if (!(*serial_)) {
			throw std::runtime_error("Failed to open Feetech serial port: " + device_name);
		}

		driver_ = std::make_unique<STSServoDriver>();
		const bool ok = driver_->init(serial_.get(), baudrate);
		if (!ok) {
			driver_.reset();
			throw std::runtime_error("Failed to initialize STSServoDriver on: " + device_name);
		}
		if (!driver_->ping(static_cast<byte>(servo_id))) {
			RCLCPP_WARN(
				this->get_logger(), "Servo ID %d did not respond to ping (check id/baud/cabling).", static_cast<int>(servo_id));
		}

		driver_->setMode(static_cast<byte>(servo_id), STSMode::POSITION);
	}

	void apply_torque_limit(int torque_limit_raw)
	{
		ensure_connected();
		const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
		const auto reg = static_cast<byte>(this->get_parameter("torque_limit_register").as_int());
		(void)driver_->writeTwoBytesRegister(servo_id, reg, static_cast<int16_t>(torque_limit_raw));
	}

	void apply_position_target(int target_ticks, int speed)
	{
		ensure_connected();
		const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
		driver_->setMode(servo_id, STSMode::POSITION);
		(void)driver_->setTargetPosition(servo_id, target_ticks, speed, false);
	}

	template<typename FeedbackT>
	bool run_motion_loop(
		const std::function<bool()> & is_canceling,
		const std::function<void()> & on_cancel,
		const std::function<void(float)> & publish_feedback,
		int target_ticks,
		int start_pos,
		int tolerance,
		double timeout,
		double poll_rate,
		FeedbackT & feedback)
	{
		const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
		const int start_err = std::max(1, std::abs(target_ticks - start_pos));
		const auto start_time = std::chrono::steady_clock::now();

		while (rclcpp::ok()) {
			if (is_canceling()) {
				on_cancel();
				return false;
			}

			const int pos = driver_->getCurrentPosition(servo_id);
			publish_gripper_joint_states(ticks_to_command_units(pos));
			const int err = std::abs(target_ticks - pos);
			if (err <= tolerance) {
				feedback->progress = 1.0f;
				publish_feedback(1.0f);
				return true;
			}

			const float progress = static_cast<float>(std::clamp(
				1.0 - (static_cast<double>(err) / static_cast<double>(start_err)), 0.0, 1.0));
			feedback->progress = progress;
			publish_feedback(progress);

			const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
				std::chrono::steady_clock::now() - start_time)
										.count();
			if (elapsed > timeout) {
				return false;
			}

			std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / poll_rate));
		}

		return false;
	}

	void execute_servo_control(const std::shared_ptr<GoalHandleServoControl> goal_handle)
	{
		const auto goal = goal_handle->get_goal();
		const int speed = this->get_parameter("speed").as_int();
		const int tolerance = this->get_parameter("goal_tolerance_ticks").as_int();
		const double timeout = this->get_parameter("motion_timeout_sec").as_double();
		double poll_rate = this->get_parameter("poll_rate_hz").as_double();
		if (poll_rate <= 0.0) {
			poll_rate = 30.0;
		}

		try {
			ensure_connected();
			const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
			if (resolve_servo_control_use_torque_mode(goal->torque)) {
				apply_torque_limit(resolve_torque_limit(goal->torque));
			}

			const int target_ticks = position_to_ticks(goal->position);
			const int current_pos = driver_->getCurrentPosition(servo_id);
			publish_gripper_joint_states(ticks_to_command_units(current_pos));
			if (is_at_target(current_pos, target_ticks, tolerance)) {
				auto result = std::make_shared<ServoControl::Result>();
				result->success = true;
				result->message = "Servo already at requested position.";
				goal_handle->succeed(result);
				return;
			}

			apply_position_target(target_ticks, speed);

			auto feedback = std::make_shared<ServoControl::Feedback>();
			const bool ok = run_motion_loop(
				[goal_handle]() { return goal_handle->is_canceling(); },
				[goal_handle]() {
					auto result = std::make_shared<ServoControl::Result>();
					result->success = false;
					result->message = "Servo control canceled.";
					goal_handle->canceled(result);
				},
				[goal_handle, feedback](float) { goal_handle->publish_feedback(feedback); },
				target_ticks,
				current_pos,
				tolerance,
				timeout,
				poll_rate,
				feedback);

			if (!ok) {
				auto result = std::make_shared<ServoControl::Result>();
				result->success = false;
				result->message = "Servo control timed out.";
				goal_handle->abort(result);
				return;
			}

			auto result = std::make_shared<ServoControl::Result>();
			result->success = true;
			result->message = "Servo command sent.";
			goal_handle->succeed(result);
		} catch (const std::exception & e) {
			auto result = std::make_shared<ServoControl::Result>();
			result->success = false;
			result->message = std::string("Servo control failed: ") + e.what();
			goal_handle->abort(result);
		}
	}

	void execute_open(const std::shared_ptr<GoalHandleOpen> goal_handle)
	{
		const auto goal = goal_handle->get_goal();
		const double open_pos = this->get_parameter("open_position").as_double();
		const int speed = this->get_parameter("speed").as_int();
		const int tolerance = this->get_parameter("goal_tolerance_ticks").as_int();
		const double timeout = this->get_parameter("motion_timeout_sec").as_double();
		double poll_rate = this->get_parameter("poll_rate_hz").as_double();
		if (poll_rate <= 0.0) {
			poll_rate = 30.0;
		}

		try {
			ensure_connected();
			const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
			const bool use_torque_mode = resolve_use_torque_mode(goal->use_torque_mode);
			if (use_torque_mode) {
				apply_torque_limit(resolve_torque_limit(goal->torque));
			}

			const int target_ticks = position_to_ticks(open_pos);
			const int current_pos = driver_->getCurrentPosition(servo_id);
			publish_gripper_joint_states(ticks_to_command_units(current_pos));
			if (is_at_target(current_pos, target_ticks, tolerance)) {
				auto result = std::make_shared<OpenGripper::Result>();
				result->success = true;
				result->message = "Gripper already open.";
				goal_handle->succeed(result);
				return;
			}

			apply_position_target(target_ticks, speed);
			auto feedback = std::make_shared<OpenGripper::Feedback>();

			const bool ok = run_motion_loop(
				[goal_handle]() { return goal_handle->is_canceling(); },
				[goal_handle]() {
					auto result = std::make_shared<OpenGripper::Result>();
					result->success = false;
					result->message = "Open canceled.";
					goal_handle->canceled(result);
				},
				[goal_handle, feedback](float) { goal_handle->publish_feedback(feedback); },
				target_ticks,
				current_pos,
				tolerance,
				timeout,
				poll_rate,
				feedback);
			if (!ok) {
				auto result = std::make_shared<OpenGripper::Result>();
				result->success = false;
				result->message = "Open timed out.";
				goal_handle->abort(result);
				return;
			}

			auto result = std::make_shared<OpenGripper::Result>();
			result->success = true;
			result->message = "Open command sent.";
			goal_handle->succeed(result);
		} catch (const std::exception & e) {
			auto result = std::make_shared<OpenGripper::Result>();
			result->success = false;
			result->message = std::string("Open failed: ") + e.what();
			goal_handle->abort(result);
		}
	}

	void execute_close(const std::shared_ptr<GoalHandleClose> goal_handle)
	{
		const auto goal = goal_handle->get_goal();
		const bool close_default = this->get_parameter("close_default").as_bool();
		const bool close_requested = goal->close || close_default;

		if (!close_requested) {
			auto result = std::make_shared<CloseGripper::Result>();
			result->success = true;
			result->message = "Close goal flag was false; no action taken.";
			goal_handle->succeed(result);
			return;
		}

		const double close_pos = this->get_parameter("close_position").as_double();
		const int speed = this->get_parameter("speed").as_int();
		const int tolerance = this->get_parameter("goal_tolerance_ticks").as_int();
		const double timeout = this->get_parameter("motion_timeout_sec").as_double();
		double poll_rate = this->get_parameter("poll_rate_hz").as_double();
		if (poll_rate <= 0.0) {
			poll_rate = 30.0;
		}

		try {
			ensure_connected();
			const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
			const bool use_torque_mode = resolve_use_torque_mode(goal->use_torque_mode);
			if (use_torque_mode) {
				apply_torque_limit(resolve_torque_limit(goal->torque));
			}

			const int target_ticks = position_to_ticks(close_pos);
			const int current_pos = driver_->getCurrentPosition(servo_id);
			publish_gripper_joint_states(ticks_to_command_units(current_pos));
			if (is_at_target(current_pos, target_ticks, tolerance)) {
				auto result = std::make_shared<CloseGripper::Result>();
				result->success = true;
				result->message = "Gripper already closed.";
				goal_handle->succeed(result);
				return;
			}

			apply_position_target(target_ticks, speed);
			auto feedback = std::make_shared<CloseGripper::Feedback>();

			const bool ok = run_motion_loop(
				[goal_handle]() { return goal_handle->is_canceling(); },
				[goal_handle]() {
					auto result = std::make_shared<CloseGripper::Result>();
					result->success = false;
					result->message = "Close canceled.";
					goal_handle->canceled(result);
				},
				[goal_handle, feedback](float) { goal_handle->publish_feedback(feedback); },
				target_ticks,
				current_pos,
				tolerance,
				timeout,
				poll_rate,
				feedback);
			if (!ok) {
				auto result = std::make_shared<CloseGripper::Result>();
				result->success = false;
				result->message = "Close timed out.";
				goal_handle->abort(result);
				return;
			}

			auto result = std::make_shared<CloseGripper::Result>();
			result->success = true;
			result->message = "Close command sent.";
			goal_handle->succeed(result);
		} catch (const std::exception & e) {
			auto result = std::make_shared<CloseGripper::Result>();
			result->success = false;
			result->message = std::string("Close failed: ") + e.what();
			goal_handle->abort(result);
		}
	}

private:

	std::unique_ptr<LinuxSerial> serial_;
	std::unique_ptr<STSServoDriver> driver_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
	rclcpp::TimerBase::SharedPtr joint_state_timer_;

	rclcpp_action::Server<ServoControl>::SharedPtr servo_control_server_;
};

}  // namespace gripper_servo_feetech

