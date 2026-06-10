from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    motor_params_file = LaunchConfiguration('motor_params_file')
    gripper_params_file = LaunchConfiguration('gripper_params_file')

    gripper_node = Node(
        package='gripper_feetech_test',
        executable='gripper_feetech_test_node',
        name='gripper_feetech_test_node',
        output='screen',
        parameters=[motor_params_file, gripper_params_file],
    )

    default_motor_params = PathJoinSubstitution([FindPackageShare('gripper_ros'), 'config', 'servos', 'feetech.yaml'])
    default_gripper_params = PathJoinSubstitution(
        [FindPackageShare('gripper_ros'), 'config', 'grippers', 'feetech_test.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'motor_params_file',
                default_value=default_motor_params,
                description='Path to the motor-model Feetech parameter YAML.',
            ),
            DeclareLaunchArgument(
                'gripper_params_file',
                default_value=default_gripper_params,
                description='Path to the gripper-level Feetech parameter YAML.',
            ),
            gripper_node,
        ]
    )