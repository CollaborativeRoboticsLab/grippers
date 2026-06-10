#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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

		RCLCPP_INFO(this->get_logger(), "Feetech test gripper node ready (reusing gripper_servo_feetech).");
	}

	~GripperNode() override = default;

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
		std::thread{std::bind(&GripperNode::execute_open, this, goal_handle)}.detach();
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

	rclcpp_action::Server<OpenGripper>::SharedPtr open_server_;
	rclcpp_action::Server<CloseGripper>::SharedPtr close_server_;
};

}  // namespace gripper_feetech_test
