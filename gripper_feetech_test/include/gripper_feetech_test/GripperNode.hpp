#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <gripper_msgs/action/close_gripper.hpp>
#include <gripper_msgs/action/open_gripper.hpp>

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

		joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
			this->get_parameter("gripper_joint_state_topic").as_string(), 10);

		open_server_ = rclcpp_action::create_server<OpenGripper>(
			this,
			"open_gripper",
			std::bind(&GripperNode::handle_goal_open, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&GripperNode::handle_cancel_open, this, std::placeholders::_1),
			std::bind(&GripperNode::handle_accepted_open, this, std::placeholders::_1));

		close_server_ = rclcpp_action::create_server<CloseGripper>(
			this,
			"close_gripper",
			std::bind(&GripperNode::handle_goal_close, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&GripperNode::handle_cancel_close, this, std::placeholders::_1),
			std::bind(&GripperNode::handle_accepted_close, this, std::placeholders::_1));

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
	using OpenGripper = gripper_msgs::action::OpenGripper;
	using CloseGripper = gripper_msgs::action::CloseGripper;
	using GoalHandleOpen = rclcpp_action::ServerGoalHandle<OpenGripper>;
	using GoalHandleClose = rclcpp_action::ServerGoalHandle<CloseGripper>;

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
		std::thread{std::bind(&GripperNode::execute_open, this, goal_handle)}.detach();
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
			const bool use_torque_mode = resolve_use_torque_mode(goal->use_torque_mode);
			if (use_torque_mode) {
				apply_torque_limit(resolve_torque_limit(goal->torque));
			}

			const int target_ticks = position_to_ticks(open_pos);
			const int current_pos = read_present_position_ticks();
			handle_position_feedback(current_pos);
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
		std::thread{std::bind(&GripperNode::execute_close, this, goal_handle)}.detach();
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
			const bool use_torque_mode = resolve_use_torque_mode(goal->use_torque_mode);
			if (use_torque_mode) {
				apply_torque_limit(resolve_torque_limit(goal->torque));
			}

			const int target_ticks = position_to_ticks(close_pos);
			const int current_pos = read_present_position_ticks();
			handle_position_feedback(current_pos);
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

	rclcpp_action::Server<OpenGripper>::SharedPtr open_server_;
	rclcpp_action::Server<CloseGripper>::SharedPtr close_server_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
	rclcpp::TimerBase::SharedPtr joint_state_timer_;
};

}  // namespace gripper_feetech_test
