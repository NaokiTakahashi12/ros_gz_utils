#!/usr/bin/env -S python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction
)
from launch.substitutions import (
    LaunchConfiguration,
    ThisLaunchFileDir
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import (
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
            default_value = ['simulator'],
            description = 'Namespace of ignition gazebo simulator (string)'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value = ['checker_ground_plane'],
            description = 'Simulation world name of ignition gazebo (string)'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value = [LaunchConfiguration('world_name'), '.sdf'],
            description = 'Simulation world file of ignition gazebo (string)'
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
                    'urdf',
                    'test_robot'
                )
            ],
            description = 'Robot model file path (string)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = ['true'],
            description = 'Enable simulation time (boolean)'
        ),
        DeclareLaunchArgument(
            'use_sim_gui',
            default_value = ['true'],
            description = 'Enable ignition gazebo gui (string)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value = ['true'],
            description = 'Enable rviz for test_robot (boolean)'
        )
    ]

def generate_launch_nodes():
    output = 'screen'

    namespace = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world_name')
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/ignition_gazebo.launch.py'
            ]),
            launch_arguments = {
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'world_name': world_name,
                'use_sim_gui': LaunchConfiguration('use_sim_gui')
            }.items()
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/ignition_spawn.launch.py'
            ]),
            launch_arguments = {
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'world_name': world_name,
                'robot_model_file': LaunchConfiguration('robot_model_file'),
                'robot_model_path': LaunchConfiguration('robot_model_path')
            }.items()
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                ThisLaunchFileDir(), '/test_robot.launch.py'
            ]),
            launch_arguments = {
                'namespace': '',  # Launch arguments chain guard
                'use_sim_time': use_sim_time,
                'use_rviz': LaunchConfiguration('use_rviz')
            }.items()
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                ThisLaunchFileDir(), '/multiple_ignition_bridge.launch.py'
            ])
        )
    ]

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()

