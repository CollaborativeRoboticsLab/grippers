#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "gripper_servo_feetech/node.hpp"

namespace gripper_feetech_test
{

class GripperNode final : public gripper_servo_feetech::FeetechGripperActionNode
{
public:
	explicit GripperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
	: gripper_servo_feetech::FeetechGripperActionNode("gripper_feetech_test_node", options)
	{
		RCLCPP_INFO(this->get_logger(), "Feetech test gripper node ready (reusing gripper_servo_feetech).");
	}

	~GripperNode() override = default;
};

}  // namespace gripper_feetech_test
