#!/usr/bin/env -S python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument
)
from launch.substitutions import (
    LaunchConfiguration,
    AnonName
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import (
    Node
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
            'rviz2_node_name',
            default_value = ['rviz2'],
            description = 'Namespace of ignition gazebo simulator (string)'
        ),
        DeclareLaunchArgument(
            'rviz2_config_file',
            default_value = [os.path.join(
                this_pkg_share_dir,
                'rviz',
                'test_robot.rviz'
            )],
            description = 'rviz2 config file path (string)'
        )
    ]

def generate_launch_nodes():
    output = 'screen'

    return [
        Node(
            package = 'rviz2',
            executable = 'rviz2',
            name = AnonName(
                LaunchConfiguration('rviz2_node_name')
            ),
            output = output,
            parameters = [
                {'use_sim_time': True},
            ],
            arguments = [
                '-d',
                LaunchConfiguration('rviz2_config_file')
            ]
        )
    ]

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()

