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

from launch import LaunchService
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    EmitEvent
)
from launch.conditions import (
    IfCondition
)
from launch.substitutions import (
    LaunchConfiguration,
    AnonName
)
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        generate_declare_launch_arguments()
        + generate_launch_nodes()
    )


def generate_declare_launch_arguments():
    return [
        DeclareLaunchArgument(
            'with_stf',
            default_value=['true'],
            description='Launch with static transform publisher (boolean)'
        ),
        DeclareLaunchArgument(
            'ign_topic',
            description='Ignition gazebo topic name (string)'
        ),
        DeclareLaunchArgument(
            'ros_topic',
            description='ROS2 topic name (string)'
        ),
        DeclareLaunchArgument(
            'ign_frame_id',
            description='Ignition gazebo transform frame id (string)',
            condition=IfCondition(
                LaunchConfiguration('with_stf')
            ),
        ),
        DeclareLaunchArgument(
            'ros_frame_id',
            description='ROS2 transform frame id (string)',
            condition=IfCondition(
                LaunchConfiguration('with_stf')
            ),
        ),
        DeclareLaunchArgument(
            'bridge_node_name',
            default_value=AnonName('parameter_bridge'),
            description='Bridge node name (string)'
        ),
        DeclareLaunchArgument(
            'stf_node_name',
            default_value=AnonName('stf'),
            description='Static transform publisher node name (string)'
        )
    ]


def generate_launch_nodes():
    output = 'screen'

    ign_topic = LaunchConfiguration('ign_topic')
    ros_topic = LaunchConfiguration('ros_topic')
    ign_frame_id = LaunchConfiguration('ign_frame_id')
    ros_frame_id = LaunchConfiguration('ros_frame_id')

    exit_event = EmitEvent(
        event=Shutdown()
    )

    return [
        GroupAction([
            Node(
                package='ros_ign_image',
                executable='image_bridge',
                name=LaunchConfiguration('bridge_node_name'),
                on_exit=[
                    exit_event
                ],
                output=output,
                parameters=[
                    {'use_sim_time': True}
                ],
                # TODO https://github.com/gazebosim/ros_gz/pull/278
                arguments=[
                    [ign_topic]
                ],
                remappings=[
                    (ign_topic, ros_topic),
                    ([ign_topic, '/compressed'], [ros_topic, '/compressed']),
                    ([ign_topic, '/compressedDepth'], [ros_topic, '/compressedDepth']),
                    ([ign_topic, '/theora'], [ros_topic, '/theora'])
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=LaunchConfiguration('stf_node_name'),
                output=output,
                parameters=[
                    {'use_sim_time': True}
                ],
                arguments=[
                    '0', '0', '0', '0', '0', '0',
                    ros_frame_id, ign_frame_id
                ],
                condition=IfCondition(
                    LaunchConfiguration('with_stf')
                )
            )
        ])
    ]


if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
