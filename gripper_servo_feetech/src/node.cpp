#include <rclcpp/rclcpp.hpp>

#include "gripper_servo_feetech/node.hpp"

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<gripper_servo_feetech::FeetechGripperActionNode>(rclcpp::NodeOptions{});
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
