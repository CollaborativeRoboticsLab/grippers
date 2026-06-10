from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description() -> LaunchDescription:
    model = LaunchConfiguration('model')
    prefix = LaunchConfiguration('prefix')
    rviz_config = LaunchConfiguration('rviz_config')

    description_file = PathJoinSubstitution(
        [FindPackageShare('gripper_description'), 'xacro', PythonExpression(["'", model, "' + '.urdf.xacro'"])]
    )

    robot_description = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            description_file,
            ' ',
            'prefix:=',
            prefix,
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='gripper_robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='gripper_rviz',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'model',
                default_value='two-finger-gripper-standalone',
                description='Gripper model name. Resolved as gripper_description/xacro/<model>.urdf.xacro',
            ),
            DeclareLaunchArgument(
                'prefix',
                default_value='',
                description='Optional TF/joint prefix for the simulated gripper model.',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=PathJoinSubstitution([FindPackageShare('gripper_ros'), 'rviz', 'gripper_sim.rviz']),
                description='Path to the RViz config for gripper simulation.',
            ),
            robot_state_publisher,
            rviz,
        ]
    )