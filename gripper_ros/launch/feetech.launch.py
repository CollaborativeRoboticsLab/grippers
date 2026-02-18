from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration('params_file')

    gripper_node = Node(
        package='gripper_feetech',
        executable='gripper_feetech_action_node',
        name='gripper_feetech_action_node',
        output='screen',
        parameters=[params_file],
    )

    default_params = [FindPackageShare('gripper_ros'), 'config', 'feetech.yaml']

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='Path to Feetech gripper parameter YAML.',
            ),
            gripper_node,
        ]
    )
