
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

// Ensure HardwareSerial is defined for the STS driver.
#include "gripper_servo_feetech/Feetech-STSServo/linux_serial.hpp"
#include "gripper_servo_feetech/Feetech-STSServo/STSServoDriver.hpp"

namespace gripper_servo_feetech
{

class FeetechGripperActionNode : public rclcpp::Node
{
public:
	struct MotionLoopResult
	{
		bool success;
		std::string reason;
	};

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
		this->declare_parameter<double>("status_publish_rate_hz", 1.0);

		// Torque limiting behavior (used as an approximation of torque-mode)
		this->declare_parameter<bool>("use_torque_mode", false);
		this->declare_parameter<double>("torque_per_current_unit", 1.0);
		this->declare_parameter<double>("torque_limit_per_torque_unit", 1.0);
		this->declare_parameter<double>("control_torque", 0.0);
		this->declare_parameter<double>("safety_torque_limit", 0.0);
		this->declare_parameter<double>("default_torque_limit", 0.0);
		this->declare_parameter<int>("torque_limit_register", static_cast<int>(STSRegisters::TORQUE_LIMIT));
		this->declare_parameter<bool>("close_default", true);

		if (enable_ros_interface) {
			servo_control_server_ = rclcpp_action::create_server<ServoControl>(
				this,
				"servo_control",
				std::bind(&FeetechGripperActionNode::handle_goal_servo_control, this, std::placeholders::_1, std::placeholders::_2),
				std::bind(&FeetechGripperActionNode::handle_cancel_servo_control, this, std::placeholders::_1),
				std::bind(&FeetechGripperActionNode::handle_accepted_servo_control, this, std::placeholders::_1));
		}

		const double status_rate = this->get_parameter("status_publish_rate_hz").as_double();
		if (status_rate > 0.0) {
			status_timer_ = this->create_wall_timer(
				std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / status_rate)),
				std::bind(&FeetechGripperActionNode::publish_status, this));
		}

		RCLCPP_INFO(this->get_logger(), "Feetech gripper action node ready.");
	}

	~FeetechGripperActionNode() override = default;


protected:
	using ServoControl = control_msgs::action::GripperCommand;
	using GoalHandleServoControl = rclcpp_action::ServerGoalHandle<ServoControl>;

	virtual void handle_position_feedback(int) {}
	virtual void handle_torque_feedback(double) {}

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

	void publish_status()
	{
		try {
			const int ticks = read_present_position_ticks();
			handle_position_feedback(ticks);
			const double position = ticks_to_command_units(ticks);
			const double current = read_present_current();
			const double torque = current_to_torque(current);
			handle_torque_feedback(torque);
			RCLCPP_INFO(
				this->get_logger(),
				"Present position: %.3f (%d ticks), present current: %.4f A, present torque: %.4f",
				position,
				ticks,
				current,
				torque);
		} catch (const std::exception & e) {
			RCLCPP_WARN(this->get_logger(), "Unable to read Feetech status: %s", e.what());
		}
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

	double resolve_torque(double requested) const
	{
		double v = requested;
		if (std::abs(v) <= 0.0) {
			const double control_torque = this->get_parameter("control_torque").as_double();
			if (std::abs(control_torque) > 0.0) {
				return control_torque;
			}

			const double default_torque_limit = this->get_parameter("default_torque_limit").as_double();
			const double torque_limit_scale = this->get_parameter("torque_limit_per_torque_unit").as_double();
			if (std::abs(default_torque_limit) > 0.0 && torque_limit_scale > 0.0) {
				return default_torque_limit / torque_limit_scale;
			}
			v = default_torque_limit;
		}
		return v;
	}

	int torque_to_limit_raw(double torque) const
	{
		const double torque_limit_scale = this->get_parameter("torque_limit_per_torque_unit").as_double();
		if (torque_limit_scale > 0.0) {
			return static_cast<int>(std::lround(torque * torque_limit_scale));
		}
		return static_cast<int>(std::lround(torque));
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

	double current_to_torque(double current_value) const
	{
		return current_value * this->get_parameter("torque_per_current_unit").as_double();
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

	int read_present_position_ticks()
	{
		ensure_connected();
		const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
		return driver_->getCurrentPosition(servo_id);
	}

	double read_present_current()
	{
		ensure_connected();
		const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
		return static_cast<double>(driver_->getCurrentCurrent(servo_id));
	}

	double read_present_torque()
	{
		const double current_torque = current_to_torque(read_present_current());
		handle_torque_feedback(current_torque);
		return current_torque;
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

	int hold_current_position(std::optional<int> current_ticks = std::nullopt)
	{
		const int hold_ticks = current_ticks.has_value() ? *current_ticks : read_present_position_ticks();
		apply_position_target(hold_ticks, this->get_parameter("speed").as_int());
		return hold_ticks;
	}

	auto make_servo_control_result(double position, double effort, bool stalled, bool reached_goal) const
	{
		auto result = std::make_shared<ServoControl::Result>();
		result->position = position;
		result->effort = effort;
		result->stalled = stalled;
		result->reached_goal = reached_goal;
		return result;
	}

	auto make_servo_control_feedback(double position, double effort, bool stalled, bool reached_goal) const
	{
		auto feedback = std::make_shared<ServoControl::Feedback>();
		feedback->position = position;
		feedback->effort = effort;
		feedback->stalled = stalled;
		feedback->reached_goal = reached_goal;
		return feedback;
	}

	template<typename FeedbackT>
	MotionLoopResult run_motion_loop(
		const std::function<bool()> & is_canceling,
		const std::function<void()> & on_cancel,
		const std::function<void(float)> & publish_feedback,
		int target_ticks,
		int start_pos,
		int tolerance,
		double timeout,
		double poll_rate,
		bool use_torque_mode,
		double target_torque,
		double safety_torque_limit,
		FeedbackT & feedback)
	{
		const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
		const int start_err = std::max(1, std::abs(target_ticks - start_pos));
		const auto start_time = std::chrono::steady_clock::now();

		while (rclcpp::ok()) {
			if (is_canceling()) {
				on_cancel();
				return MotionLoopResult{false, "canceled"};
			}

			const int pos = driver_->getCurrentPosition(servo_id);
			handle_position_feedback(pos);
			const double applied_torque = read_present_torque();
			const int err = std::abs(target_ticks - pos);
			if (err <= tolerance) {
				feedback->position = ticks_to_command_units(pos);
				feedback->effort = applied_torque;
				feedback->stalled = false;
				feedback->reached_goal = true;
				publish_feedback(1.0f);
				return MotionLoopResult{true, "position_reached"};
			}

			if (use_torque_mode && std::abs(target_torque) > 0.0 && std::abs(applied_torque) >= std::abs(target_torque)) {
				hold_current_position(pos);
				feedback->position = ticks_to_command_units(pos);
				feedback->effort = applied_torque;
				feedback->stalled = true;
				feedback->reached_goal = false;
				publish_feedback(1.0f);
				return MotionLoopResult{true, "target_torque_reached"};
			}

			if (safety_torque_limit > 0.0 && std::abs(applied_torque) >= std::abs(safety_torque_limit)) {
				hold_current_position(pos);
				feedback->position = ticks_to_command_units(pos);
				feedback->effort = applied_torque;
				feedback->stalled = true;
				feedback->reached_goal = false;
				publish_feedback(1.0f);
				return MotionLoopResult{true, "safety_torque_limit_reached"};
			}

			const float progress = static_cast<float>(std::clamp(
				1.0 - (static_cast<double>(err) / static_cast<double>(start_err)), 0.0, 1.0));
			feedback->position = ticks_to_command_units(pos);
			feedback->effort = applied_torque;
			feedback->stalled = false;
			feedback->reached_goal = false;
			publish_feedback(progress);

			const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
				std::chrono::steady_clock::now() - start_time)
										.count();
			if (elapsed > timeout) {
				return MotionLoopResult{false, "timeout"};
			}

			std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / poll_rate));
		}

		return MotionLoopResult{false, "shutdown"};
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
		const double requested_effort = goal->command.max_effort;
		const double torque = resolve_torque(requested_effort);
		const bool use_torque_mode = resolve_servo_control_use_torque_mode(requested_effort);

		try {
			ensure_connected();
			if (use_torque_mode) {
				apply_torque_limit(torque_to_limit_raw(torque));
			}

			const int target_ticks = position_to_ticks(goal->command.position);
			const int current_pos = read_present_position_ticks();
			handle_position_feedback(current_pos);
			if (is_at_target(current_pos, target_ticks, tolerance)) {
				auto result = make_servo_control_result(ticks_to_command_units(current_pos), torque, false, true);
				goal_handle->succeed(result);
				return;
			}

			apply_position_target(target_ticks, speed);

			auto feedback = std::make_shared<ServoControl::Feedback>();
			const auto motion_result = run_motion_loop(
				[goal_handle]() { return goal_handle->is_canceling(); },
				[this, goal_handle, torque]() {
					auto result = make_servo_control_result(ticks_to_command_units(read_present_position_ticks()), torque, false, false);
					goal_handle->canceled(result);
				},
				[goal_handle, feedback](float) { goal_handle->publish_feedback(feedback); },
				target_ticks,
				current_pos,
				tolerance,
				timeout,
				poll_rate,
				use_torque_mode,
				torque,
				this->get_parameter("safety_torque_limit").as_double(),
				feedback);

			if (!motion_result.success) {
				if (motion_result.reason == "canceled") {
					return;
				}
				auto result = make_servo_control_result(ticks_to_command_units(read_present_position_ticks()), torque, false, false);
				goal_handle->abort(result);
				return;
			}

			const int final_ticks = read_present_position_ticks();
			const double final_position = ticks_to_command_units(final_ticks);
			const double final_torque = read_present_torque();
			if (motion_result.reason == "target_torque_reached") {
				auto result = make_servo_control_result(final_position, final_torque, true, false);
				goal_handle->succeed(result);
			} else if (motion_result.reason == "safety_torque_limit_reached") {
				auto result = make_servo_control_result(final_position, final_torque, true, false);
				goal_handle->succeed(result);
			} else {
				auto result = make_servo_control_result(final_position, final_torque, false, true);
				goal_handle->succeed(result);
			}
		} catch (const std::exception & e) {
			RCLCPP_WARN(this->get_logger(), "Servo control failed: %s", e.what());
			auto result = make_servo_control_result(goal->command.position, torque, false, false);
			goal_handle->abort(result);
		}
	}

private:

	std::unique_ptr<LinuxSerial> serial_;
	std::unique_ptr<STSServoDriver> driver_;

	rclcpp_action::Server<ServoControl>::SharedPtr servo_control_server_;
	rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace gripper_servo_feetech

