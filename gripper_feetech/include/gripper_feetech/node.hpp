
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <gripper_msgs/action/open_gripper.hpp>
#include <gripper_msgs/action/close_gripper.hpp>

// Ensure HardwareSerial is defined for the STS driver.
#include "gripper_feetech/Feetech-STSServo/linux_serial.hpp"
#include "gripper_feetech/Feetech-STSServo/STSServoDriver.hpp"

namespace gripper_feetech
{

class FeetechGripperActionNode : public rclcpp::Node
{
public:
	explicit FeetechGripperActionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
	: rclcpp::Node("gripper_feetech_action_node", options)
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

		open_server_ = rclcpp_action::create_server<OpenGripper>(
			this,
			"open_gripper",
			std::bind(&FeetechGripperActionNode::handle_goal_open, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&FeetechGripperActionNode::handle_cancel_open, this, std::placeholders::_1),
			std::bind(&FeetechGripperActionNode::handle_accepted_open, this, std::placeholders::_1));

		close_server_ = rclcpp_action::create_server<CloseGripper>(
			this,
			"close_gripper",
			std::bind(&FeetechGripperActionNode::handle_goal_close, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&FeetechGripperActionNode::handle_cancel_close, this, std::placeholders::_1),
			std::bind(&FeetechGripperActionNode::handle_accepted_close, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "Feetech gripper action node ready.");
	}

	~FeetechGripperActionNode() override = default;

private:
	using OpenGripper = gripper_msgs::action::OpenGripper;
	using CloseGripper = gripper_msgs::action::CloseGripper;
	using GoalHandleOpen = rclcpp_action::ServerGoalHandle<OpenGripper>;
	using GoalHandleClose = rclcpp_action::ServerGoalHandle<CloseGripper>;

	rclcpp_action::GoalResponse handle_goal_open(
		const rclcpp_action::GoalUUID &, std::shared_ptr<const OpenGripper::Goal>)
	{
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handle_cancel_open(const std::shared_ptr<GoalHandleOpen>)
	{
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted_open(const std::shared_ptr<GoalHandleOpen> goal_handle)
	{
		std::thread{std::bind(&FeetechGripperActionNode::execute_open, this, goal_handle)}.detach();
	}

	rclcpp_action::GoalResponse handle_goal_close(
		const rclcpp_action::GoalUUID &, std::shared_ptr<const CloseGripper::Goal>)
	{
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handle_cancel_close(const std::shared_ptr<GoalHandleClose>)
	{
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted_close(const std::shared_ptr<GoalHandleClose> goal_handle)
	{
		std::thread{std::bind(&FeetechGripperActionNode::execute_close, this, goal_handle)}.detach();
	}

	bool resolve_use_torque_mode(bool goal_flag) const
	{
		return goal_flag || this->get_parameter("use_torque_mode").as_bool();
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
			RCLCPP_WARN(this->get_logger(), "Servo ID %d did not respond to ping (check id/baud/cabling).", servo_id);
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
			const bool use_torque_mode = resolve_use_torque_mode(goal->use_torque_mode);
			if (use_torque_mode) {
				apply_torque_limit(resolve_torque_limit(goal->torque));
			}

			const int target_ticks = position_to_ticks(open_pos);
			apply_position_target(target_ticks, speed);

			const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
			const int start_pos = driver_->getCurrentPosition(servo_id);
			const int start_err = std::max(1, std::abs(target_ticks - start_pos));
			const auto start_time = std::chrono::steady_clock::now();

			auto feedback = std::make_shared<OpenGripper::Feedback>();

			while (rclcpp::ok()) {
				if (goal_handle->is_canceling()) {
					auto result = std::make_shared<OpenGripper::Result>();
					result->success = false;
					result->message = "Open canceled.";
					goal_handle->canceled(result);
					return;
				}

				const int pos = driver_->getCurrentPosition(servo_id);
				const int err = std::abs(target_ticks - pos);
				if (err <= tolerance) {
					feedback->progress = 1.0f;
					goal_handle->publish_feedback(feedback);
					break;
				}

				const float progress = static_cast<float>(std::clamp(
					1.0 - (static_cast<double>(err) / static_cast<double>(start_err)), 0.0, 1.0));
				feedback->progress = progress;
				goal_handle->publish_feedback(feedback);

				const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
					std::chrono::steady_clock::now() - start_time)
																.count();
				if (elapsed > timeout) {
					auto result = std::make_shared<OpenGripper::Result>();
					result->success = false;
					result->message = "Open timed out.";
					goal_handle->abort(result);
					return;
				}

				std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / poll_rate));
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
			const bool use_torque_mode = resolve_use_torque_mode(goal->use_torque_mode);
			if (use_torque_mode) {
				apply_torque_limit(resolve_torque_limit(goal->torque));
			}

			const int target_ticks = position_to_ticks(close_pos);
			apply_position_target(target_ticks, speed);

			const auto servo_id = static_cast<byte>(this->get_parameter("servo_id").as_int());
			const int start_pos = driver_->getCurrentPosition(servo_id);
			const int start_err = std::max(1, std::abs(target_ticks - start_pos));
			const auto start_time = std::chrono::steady_clock::now();

			auto feedback = std::make_shared<CloseGripper::Feedback>();

			while (rclcpp::ok()) {
				if (goal_handle->is_canceling()) {
					auto result = std::make_shared<CloseGripper::Result>();
					result->success = false;
					result->message = "Close canceled.";
					goal_handle->canceled(result);
					return;
				}

				const int pos = driver_->getCurrentPosition(servo_id);
				const int err = std::abs(target_ticks - pos);
				if (err <= tolerance) {
					feedback->progress = 1.0f;
					goal_handle->publish_feedback(feedback);
					break;
				}

				const float progress = static_cast<float>(std::clamp(
					1.0 - (static_cast<double>(err) / static_cast<double>(start_err)), 0.0, 1.0));
				feedback->progress = progress;
				goal_handle->publish_feedback(feedback);

				const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
					std::chrono::steady_clock::now() - start_time)
																.count();
				if (elapsed > timeout) {
					auto result = std::make_shared<CloseGripper::Result>();
					result->success = false;
					result->message = "Close timed out.";
					goal_handle->abort(result);
					return;
				}

				std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / poll_rate));
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

	std::unique_ptr<LinuxSerial> serial_;
	std::unique_ptr<STSServoDriver> driver_;

	rclcpp_action::Server<OpenGripper>::SharedPtr open_server_;
	rclcpp_action::Server<CloseGripper>::SharedPtr close_server_;
};

}  // namespace gripper_feetech

