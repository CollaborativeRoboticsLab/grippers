
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
	params_file = LaunchConfiguration('params_file')

	gripper_node = Node(
		package='gripper_dynamixel',
		executable='gripper_dynamixel_action_node',
		name='gripper_dynamixel_action_node',
		output='screen',
		parameters=[params_file],
	)

	default_params = PathJoinSubstitution([FindPackageShare('gripper_ros'), 'config', 'dynamixel.yaml'])

	return LaunchDescription(
		[
			DeclareLaunchArgument(
				'params_file',
				default_value=default_params,
				description='Path to Dynamixel gripper parameter YAML.',
			),
			gripper_node,
		]
	)

