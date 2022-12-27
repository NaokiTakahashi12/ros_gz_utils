#!/usr/bin/env -S python3

# Copyright (c) 2022 Naoki Takahashi
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchService
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.substitutions import (
    LaunchConfiguration,
    ThisLaunchFileDir
)
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        generate_declare_launch_arguments()
        + generate_launch_nodes()
    )


def generate_declare_launch_arguments():
    this_pkg_share_dir = get_package_share_directory('ros_gz_utils')

    return [
        DeclareLaunchArgument(
            'namespace',
            default_value=['simulator'],
            description='Namespace of ignition gazebo simulator (string)'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value=['checker_ground_plane'],
            description='Simulation world name of ignition gazebo (string)'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value=[LaunchConfiguration('world_name'), '.sdf'],
            description='Simulation world file of ignition gazebo (string)'
        ),
        DeclareLaunchArgument(
            'robot_model_file',
            default_value=['test_robot/test_robot.urdf.xacro'],
            description='Robot model file (string)'
        ),
        DeclareLaunchArgument(
            'robot_model_path',
            default_value=[
                os.path.join(
                    this_pkg_share_dir,
                    'models',
                    'urdf'
                )
            ],
            description='Robot model file path (string)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=['true'],
            description='Enable simulation time (boolean)'
        ),
        DeclareLaunchArgument(
            'use_sim_gui',
            default_value=['true'],
            description='Enable ignition gazebo gui (string)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value=['true'],
            description='Enable rviz for test_robot (boolean)'
        )
    ]


def generate_launch_nodes():
    namespace = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world_name')
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gz_version_env_name = 'IGNITION_VERSION'
    old_style_plugin_option = ''

    if os.getenv(gz_version_env_name) is None:
        raise KeyError('Please export ' + gz_version_env_name)
    if os.getenv(gz_version_env_name) == 'garden':
        old_style_plugin_option = 'ign_compatible:=false'
    else:
        old_style_plugin_option = 'ign_compatible:=true'

    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/ignition_gazebo.launch.py'
            ]),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'world_name': world_name,
                'world_file': world_file,
                'use_sim_gui': LaunchConfiguration('use_sim_gui')
            }.items()
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/ignition_spawn.launch.py'
            ]),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'world_name': world_name,
                'robot_model_from_topic': 'false',
                'robot_model_file': LaunchConfiguration('robot_model_file'),
                'robot_model_path': LaunchConfiguration('robot_model_path'),
                'xacro_args': old_style_plugin_option
            }.items()
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                ThisLaunchFileDir(), '/test_robot.launch.py'
            ]),
            launch_arguments={
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
