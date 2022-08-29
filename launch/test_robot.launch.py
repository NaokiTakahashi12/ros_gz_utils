#!/usr/bin/env -S python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    EmitEvent
)
from launch.conditions import (
    IfCondition
)
from launch.substitutions import (
    LaunchConfiguration,
    ThisLaunchFileDir,
    PathJoinSubstitution,
    Command,
    AnonName
)
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import (
    Node,
    PushRosNamespace
)

def generate_launch_description():
    return LaunchDescription(
        generate_declare_launch_arguments()
        + generate_launch_nodes()
    )

def generate_declare_launch_arguments():
    this_pkg_share_dir = get_package_share_directory('ros_ign_utils')

    return [
        DeclareLaunchArgument(
            'namespace',
            default_value = [''],
            description = 'Namespace of ignition gazebo simulator (string)'
        ),
        DeclareLaunchArgument(
            'robot_model_file',
            default_value = ['test_robot.urdf.xacro'],
            description = 'Robot model file (string)'
        ),
        DeclareLaunchArgument(
            'robot_model_path',
            default_value = [
                os.path.join(
                    this_pkg_share_dir,
                    'models',
                    'urdf'
                )
            ],
            description = 'Robot model file path (string)'
        ),
        DeclareLaunchArgument(
            'joint_state_publisher_config_file',
            default_value = [
                os.path.join(
                    this_pkg_share_dir,
                    'config',
                    'joint_state_sources.yaml'
                )
            ],
            description = 'Joint state publisher node parameter file (string)'
        ),
        DeclareLaunchArgument(
            'ros2_controller_config_file',
            default_value = [
                os.path.join(
                    this_pkg_share_dir,
                    'config',
                    'test_robot_controllers.yaml'
                )
            ],
            description = 'ros2_controller node parameter file (string)'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value = ['false'],
            description = 'Enable fake ros2 hardware (boolean)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = ['true'],
            description = 'Enable simulation time (boolean)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value = ['true'],
            description = 'Enable rviz for test_robot (boolean)'
        )
    ]

def generate_launch_nodes():
    output = 'screen'

    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_file = PathJoinSubstitution([
        LaunchConfiguration('robot_model_path'),
        LaunchConfiguration('robot_model_file')
    ])

    robot_description = {
        'robot_description': Command([
            'xacro ',
            urdf_file,
            ' use_fake_hardware:=',
            LaunchConfiguration('use_fake_hardware')
        ])
    }

    exit_event = EmitEvent(
        event = Shutdown()
    )

    controller_name = 'controller_manager'

    return [
        GroupAction(actions = [
            PushRosNamespace(
                namespace = LaunchConfiguration('namespace')
            ),
            Node(
                package = 'robot_state_publisher',
                executable = 'robot_state_publisher',
                name = 'robot_state_publisher',
                output = output,
                parameters = [
                    robot_description,
                    {'use_sim_time': use_sim_time}
                ],
                on_exit = exit_event
            ),
            # TODO https://github.com/ros/joint_state_publisher/issues/82
            Node(
                package = 'joint_state_publisher',
                executable = 'joint_state_publisher',
                name = 'joint_state_merger',
                output = output,
                parameters = [
                    {'use_sim_time': use_sim_time},
                    {'ignore_timestamp': False},
                    {'publish_default_efforts': True},
                    {'publish_default_velocities': True},
                    {'publish_default_positions': True},
                    LaunchConfiguration('joint_state_publisher_config_file')
                ]
            ),
            Node(
                package = 'controller_manager',
                executable = 'ros2_control_node',
                output = output,
                parameters = [
                    robot_description,
                    LaunchConfiguration('ros2_controller_config_file'),
                    {'use_sim_time': use_sim_time}
                ],
                remappings = [
                    ('joint_states', 'joint_state_broadcaster/joint_states'),
                    # TODO https://github.com/ros-controls/ros2_control/issues/785
                    ('controller_manager:__name', controller_name)
                ],
                condition = IfCondition(
                    LaunchConfiguration('use_fake_hardware')
                )
            ),
            Node(
                package = 'controller_manager',
                executable = 'spawner',
                name = AnonName('controller_spawner'),
                arguments = [
                    'joint_state_broadcaster',
                    '--controller-manager',
                    controller_name
                ]
            ),
            Node(
                package = 'controller_manager',
                executable = 'spawner',
                name = AnonName('controller_spawner'),
                arguments = [
                    'diff_drive_controller',
                    '--controller-manager',
                    controller_name
                ]
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(),
                    '/test_robot_visualization.launch.py'
                ]),
                launch_arguments = {
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
                condition = IfCondition(
                    LaunchConfiguration('use_rviz')
                )
            )
        ])
    ]


if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()

