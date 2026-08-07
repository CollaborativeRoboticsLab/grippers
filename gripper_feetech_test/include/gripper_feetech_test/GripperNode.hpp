#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <control_msgs/action/gripper_command.hpp>

#include "gripper_servo_feetech/node.hpp"

namespace gripper_feetech_test
{

class GripperNode final : public gripper_servo_feetech::FeetechGripperActionNode
{
public:
	explicit GripperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
	: gripper_servo_feetech::FeetechGripperActionNode("gripper_feetech_test_node", options, false)
	{
		this->declare_parameter<bool>("publish_gripper_joint_states", false);
		this->declare_parameter<std::string>("gripper_joint_state_topic", "joint_states");
		this->declare_parameter<std::string>("gripper_joint_name_prefix", "");
		this->declare_parameter<std::vector<std::string>>("gripper_joint_state_names", std::vector<std::string>{});
		this->declare_parameter<std::vector<double>>("gripper_joint_state_multipliers", std::vector<double>{});
		this->declare_parameter<std::vector<double>>("gripper_joint_state_offsets", std::vector<double>{});
		this->declare_parameter<double>("gripper_joint_state_rate_hz", 10.0);
		this->declare_parameter<double>("gripper_open_width", 0.09);
		this->declare_parameter<double>("gripper_closed_width", 0.0);

		joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
			this->get_parameter("gripper_joint_state_topic").as_string(), 10);

		gripper_command_server_ = rclcpp_action::create_server<GripperCommand>(
			this,
			"gripper_command",
			std::bind(&GripperNode::handle_goal_gripper_command, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&GripperNode::handle_cancel_gripper_command, this, std::placeholders::_1),
			std::bind(&GripperNode::handle_accepted_gripper_command, this, std::placeholders::_1));

		if (this->get_parameter("publish_gripper_joint_states").as_bool()) {
			const double rate_hz = this->get_parameter("gripper_joint_state_rate_hz").as_double();
			if (rate_hz > 0.0) {
				joint_state_timer_ = this->create_wall_timer(
					std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_hz)),
					std::bind(&GripperNode::publish_gripper_joint_states_timer_cb, this));
			} else {
				publish_gripper_joint_states_from_feedback();
			}
		}

		RCLCPP_INFO(this->get_logger(), "Feetech test gripper node ready (reusing gripper_servo_feetech).");
	}

	~GripperNode() override = default;

private:
	using GripperCommand = control_msgs::action::GripperCommand;
	using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

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
			const int pos = read_present_position_ticks();
			publish_gripper_joint_states(ticks_to_command_units(pos));
		} catch (const std::exception & e) {
			RCLCPP_WARN(this->get_logger(), "Failed to publish gripper joint states from feedback: %s", e.what());
		}
	}

	void publish_gripper_joint_states_timer_cb()
	{
		publish_gripper_joint_states_from_feedback();
	}

	void handle_position_feedback(int position_ticks) override
	{
		publish_gripper_joint_states(ticks_to_command_units(position_ticks));
	}

	void handle_torque_feedback(double) override {}

	void log_gripper_action_request(const char * action_name, double torque, bool use_torque_mode)
	{
		RCLCPP_INFO(
			this->get_logger(),
			"%s requested (torque=%.3f, use_torque_mode=%s, safety_torque_limit=%.3f)",
			action_name,
			torque,
			use_torque_mode ? "true" : "false",
			this->get_parameter("safety_torque_limit").as_double());
	}

	void log_gripper_action_result(const char * action_name, const std::string & message, bool success)
	{
		if (success) {
			RCLCPP_INFO(this->get_logger(), "%s result: %s", action_name, message.c_str());
		} else {
			RCLCPP_WARN(this->get_logger(), "%s result: %s", action_name, message.c_str());
		}
	}

	std::pair<double, double> gripper_width_limits() const
	{
		const double open_width = this->get_parameter("gripper_open_width").as_double();
		const double closed_width = this->get_parameter("gripper_closed_width").as_double();
		if (open_width < closed_width) {
			throw std::runtime_error("gripper_open_width must be greater than or equal to gripper_closed_width.");
		}
		return {open_width, closed_width};
	}

	double gripper_width_to_command_position(double width_m) const
	{
		const auto [open_width, closed_width] = gripper_width_limits();
		const double requested_width = std::clamp(width_m, closed_width, open_width);
		double close_ratio = 0.0;
		if (std::abs(open_width - closed_width) > 0.0) {
			close_ratio = (open_width - requested_width) / (open_width - closed_width);
		}

		const double open_position = this->get_parameter("open_position").as_double();
		const double close_position = this->get_parameter("close_position").as_double();
		return open_position + ((close_position - open_position) * close_ratio);
	}

	double command_position_to_gripper_width(double command_position) const
	{
		const auto [open_width, closed_width] = gripper_width_limits();
		const double open_position = this->get_parameter("open_position").as_double();
		const double close_position = this->get_parameter("close_position").as_double();
		if (std::abs(close_position - open_position) <= 0.0) {
			return open_width;
		}

		const double close_ratio = std::clamp((command_position - open_position) / (close_position - open_position), 0.0, 1.0);
		return open_width + ((closed_width - open_width) * close_ratio);
	}

	double ticks_to_gripper_width(int position_ticks) const
	{
		return command_position_to_gripper_width(ticks_to_command_units(position_ticks));
	}

	rclcpp_action::GoalResponse handle_goal_gripper_command(
		const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommand::Goal>)
	{
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handle_cancel_gripper_command(const std::shared_ptr<GoalHandleGripperCommand>)
	{
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted_gripper_command(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
	{
		std::thread{std::bind(&GripperNode::execute_gripper_command, this, goal_handle)}.detach();
	}

	auto make_gripper_command_result(double position, double effort, bool stalled, bool reached_goal) const
	{
		auto result = std::make_shared<GripperCommand::Result>();
		result->position = position;
		result->effort = effort;
		result->stalled = stalled;
		result->reached_goal = reached_goal;
		return result;
	}

	auto make_gripper_command_feedback(double position, double effort, bool stalled, bool reached_goal) const
	{
		auto feedback = std::make_shared<GripperCommand::Feedback>();
		feedback->position = position;
		feedback->effort = effort;
		feedback->stalled = stalled;
		feedback->reached_goal = reached_goal;
		return feedback;
	}

	void execute_gripper_command(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
	{
		const auto goal = goal_handle->get_goal();
		const double target_width = goal->command.position;
		const double target_position = gripper_width_to_command_position(target_width);
		const double requested_effort = goal->command.max_effort;
		const bool use_torque_mode = resolve_use_torque_mode(std::abs(requested_effort) > 0.0);
		const double torque = resolve_torque(requested_effort, use_torque_mode);
		log_gripper_action_request("gripper_command", torque, use_torque_mode);

		const int speed = this->get_parameter("speed").as_int();
		const int tolerance = this->get_parameter("goal_tolerance_ticks").as_int();
		const double timeout = this->get_parameter("motion_timeout_sec").as_double();
		double poll_rate = this->get_parameter("poll_rate_hz").as_double();
		if (poll_rate <= 0.0) {
			poll_rate = 30.0;
		}

		try {
			ensure_connected();
			if (use_torque_mode) {
				apply_torque_limit(torque_to_limit_raw(torque));
			}

			const int target_ticks = position_to_ticks(target_position);
			const int current_pos = read_present_position_ticks();
			handle_position_feedback(current_pos);
			if (is_at_target(current_pos, target_ticks, tolerance)) {
				auto result = make_gripper_command_result(ticks_to_gripper_width(current_pos), torque, false, true);
				log_gripper_action_result("gripper_command", "Gripper already at requested position.", true);
				goal_handle->succeed(result);
				return;
			}

			apply_position_target(target_ticks, speed);
			const int start_err = std::max(1, std::abs(target_ticks - current_pos));
			const auto start_time = std::chrono::steady_clock::now();

			while (rclcpp::ok()) {
				if (goal_handle->is_canceling()) {
					auto result = make_gripper_command_result(ticks_to_gripper_width(read_present_position_ticks()), torque, false, false);
					log_gripper_action_result("gripper_command", "Gripper command canceled.", false);
					goal_handle->canceled(result);
					return;
				}

				const int pos = read_present_position_ticks();
				handle_position_feedback(pos);
				const double applied_torque = read_present_torque();
				const int err = std::abs(target_ticks - pos);
				const double gripper_width = ticks_to_gripper_width(pos);

				if (err <= tolerance) {
					auto result = make_gripper_command_result(gripper_width, applied_torque, false, true);
					goal_handle->publish_feedback(make_gripper_command_feedback(gripper_width, applied_torque, false, true));
					log_gripper_action_result("gripper_command", "Gripper command sent.", true);
					goal_handle->succeed(result);
					return;
				}

				if (use_torque_mode && std::abs(torque) > 0.0 && std::abs(applied_torque) >= std::abs(torque)) {
					hold_current_position(pos);
					auto result = make_gripper_command_result(gripper_width, applied_torque, true, false);
					goal_handle->publish_feedback(make_gripper_command_feedback(gripper_width, applied_torque, true, false));
					log_gripper_action_result("gripper_command", "Gripper reached the requested effort and is holding position.", true);
					goal_handle->succeed(result);
					return;
				}

				const double safety_torque_limit = this->get_parameter("safety_torque_limit").as_double();
				if (safety_torque_limit > 0.0 && std::abs(applied_torque) >= std::abs(safety_torque_limit)) {
					hold_current_position(pos);
					auto result = make_gripper_command_result(gripper_width, applied_torque, true, false);
					goal_handle->publish_feedback(make_gripper_command_feedback(gripper_width, applied_torque, true, false));
					log_gripper_action_result("gripper_command", "Gripper stopped at the safety effort limit and is holding position.", true);
					goal_handle->succeed(result);
					return;
				}

				const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
					std::chrono::steady_clock::now() - start_time)
										.count();
				if (elapsed > timeout) {
					auto result = make_gripper_command_result(gripper_width, applied_torque, false, false);
					log_gripper_action_result("gripper_command", "Gripper command timed out.", false);
					goal_handle->abort(result);
					return;
				}

				const double progress = std::clamp(1.0 - (static_cast<double>(err) / static_cast<double>(start_err)), 0.0, 1.0);
				(void)progress;
				goal_handle->publish_feedback(make_gripper_command_feedback(gripper_width, applied_torque, false, false));
				std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / poll_rate));
			}
		} catch (const std::exception & e) {
			auto result = make_gripper_command_result(target_width, torque, false, false);
			log_gripper_action_result("gripper_command", std::string("Gripper command failed: ") + e.what(), false);
			goal_handle->abort(result);
		}
	}

	rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_server_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
	rclcpp::TimerBase::SharedPtr joint_state_timer_;
};

}  // namespace gripper_feetech_test
