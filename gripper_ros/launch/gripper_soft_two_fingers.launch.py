from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    motor_params_file = LaunchConfiguration('motor_params_file')
    gripper_params_file = LaunchConfiguration('gripper_params_file')
    bypass_max_effort = LaunchConfiguration('bypass_max_effort')
    gripper_joint_state_topic = LaunchConfiguration('gripper_joint_state_topic')

    gripper_node = Node(
        package='gripper_two_fingers',
        executable='gripper_two_fingers_node',
        name='gripper_two_fingers_node',
        output='screen',
        remappings=[
            ('gripper_joint_states', gripper_joint_state_topic),
            ('/gripper_joint_states', gripper_joint_state_topic),
        ],
        parameters=[
            motor_params_file,
            gripper_params_file,
            {
                'bypass_max_effort': bypass_max_effort,
            },
        ],
    )

    default_motor_params = PathJoinSubstitution([FindPackageShare('gripper_ros'), 'config', 'servos', 'dynamixel.yaml'])
    default_gripper_params = PathJoinSubstitution(
        [FindPackageShare('gripper_ros'), 'config', 'grippers', 'soft_two_finger_dynamixel.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'motor_params_file',
                default_value=default_motor_params,
                description='Path to the motor-model Dynamixel parameter YAML.',
            ),
            DeclareLaunchArgument(
                'gripper_params_file',
                default_value=default_gripper_params,
                description='Path to the gripper-level Dynamixel parameter YAML.',
            ),
            DeclareLaunchArgument(
                'bypass_max_effort',
                default_value='false',
                description='When true, gripper-level command.max_effort bypasses force conversion and is treated as direct torque.',
            ),
            DeclareLaunchArgument(
                'gripper_joint_state_topic',
                default_value='gripper_joint_states',
                description='Topic used to publish gripper joint states.',
            ),
            gripper_node,
        ]
    )