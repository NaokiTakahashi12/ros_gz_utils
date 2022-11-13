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
    DeclareLaunchArgument
)
from launch.substitutions import (
    LaunchConfiguration,
    AnonName
)
from launch_ros.actions import (
    Node
)


def generate_launch_description():
    return LaunchDescription(
        generate_declare_launch_arguments()
        + generate_launch_nodes()
    )


def generate_declare_launch_arguments():
    this_pkg_share_dir = get_package_share_directory('ros_gz_utils')

    return [
        DeclareLaunchArgument(
            'rviz2_node_name',
            default_value=['rviz2'],
            description='Namespace of ignition gazebo simulator (string)'
        ),
        DeclareLaunchArgument(
            'rviz2_config_file',
            default_value=[os.path.join(
                this_pkg_share_dir,
                'rviz',
                'test_robot.rviz'
            )],
            description='rviz2 config file path (string)'
        )
    ]


def generate_launch_nodes():
    output = 'screen'

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name=AnonName(
                LaunchConfiguration('rviz2_node_name')
            ),
            output=output,
            parameters=[
                {'use_sim_time': True},
            ],
            arguments=[
                '-d',
                LaunchConfiguration('rviz2_config_file')
            ]
        )
    ]


if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
