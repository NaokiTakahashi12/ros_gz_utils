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
    SetEnvironmentVariable
)
from launch.conditions import (
    IfCondition,
    UnlessCondition
)
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    PathJoinSubstitution,
    Command,
    AnonName
)
from launch_ros.actions import (
    Node
)


def generate_launch_description():
    return LaunchDescription(
        generate_declare_launch_arguments()
        + generate_local_environment_variables()
        + generate_launch_nodes()
    )


def generate_declare_launch_arguments():
    return [
        DeclareLaunchArgument(
            'namespace',
            default_value=['simulator'],
            description='Namespace of ignition gazebo simulator (string)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=['true'],
            description='Subscribe /clock topic (boolean)'
        ),
        DeclareLaunchArgument(
            'spawn_position_x',
            default_value=['0.0'],
            description='Spawn world position x (float)'
        ),
        DeclareLaunchArgument(
            'spawn_position_y',
            default_value=['0.0'],
            description='Spawn world position y (float)'
        ),
        DeclareLaunchArgument(
            'spawn_position_z',
            default_value=['0.5'],
            description='Spawn world position z (float)'
        ),
        DeclareLaunchArgument(
            'spawn_orientation_r',
            default_value=['0.0'],
            description='Spawn world orientation roll (float)'
        ),
        DeclareLaunchArgument(
            'spawn_orientation_p',
            default_value=['0.0'],
            description='Spawn world orientation pitch (float)'
        ),
        DeclareLaunchArgument(
            'spawn_orientation_y',
            default_value=['0.0'],
            description='Spawn world orientation yaw (float)'
        ),
        DeclareLaunchArgument(
            'robot_model_from_topic',
            default_value=['false'],
            description='Get robot description from topic (boolean)'
        ),
        DeclareLaunchArgument(
            'robot_description_topic',
            default_value=['robot_description'],
            description='robot_description topic (string)',
            condition=IfCondition(
                LaunchConfiguration('robot_model_from_topic')
            )
        ),
        DeclareLaunchArgument(
            'robot_model_file',
            default_value=['test_robot.urdf.xacro'],
            description='Robot model file (string)',
            condition=UnlessCondition(
                LaunchConfiguration('robot_model_from_topic')
            )
        ),
        DeclareLaunchArgument(
            'robot_model_path',
            default_value=[
                os.path.join(
                    get_package_share_directory('ros_gz_utils'),
                    'models',
                    'urdf'
                )
            ],
            description='Robot model file path (string)',
            condition=UnlessCondition(
                LaunchConfiguration('robot_model_from_topic')
            )
        ),
        DeclareLaunchArgument(
            'xacro_args',
            default_value=[''],
            description='xacro arguments (string)'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value=['default'],
            description='Simulation world name of ignition gazebo (string)'
        ),
        DeclareLaunchArgument(
            'spawn_node_name',
            default_value=[AnonName('spawner_node')],
            description='Spawn robot node of ignition gazebo (string)'
        )
    ]


def generate_launch_nodes():
    output = 'screen'

    namespace = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world_name')

    use_sim_time = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    world = ['-world', world_name]
    spawn_model_pose = [
        '-x', LaunchConfiguration('spawn_position_x'),
        '-y', LaunchConfiguration('spawn_position_y'),
        '-z', LaunchConfiguration('spawn_position_z'),
        '-R', LaunchConfiguration('spawn_orientation_r'),
        '-P', LaunchConfiguration('spawn_orientation_p'),
        '-Y', LaunchConfiguration('spawn_orientation_y')
    ]

    gz_version_env_name = 'IGNITION_VERSION'
    ros_gz_package_name = 'ros_ign_gazebo'

    if os.getenv(gz_version_env_name) is None:
        raise KeyError('Please export ' + gz_version_env_name)
    if os.getenv(gz_version_env_name) == 'garden':
        ros_gz_package_name = 'ros_gz_sim'
    else:
        ros_gz_package_name = 'ros_ign_gazebo'

    return [
        Node(
            package=ros_gz_package_name,
            executable='create',
            name=LaunchConfiguration('spawn_node_name'),
            namespace=namespace,
            output=output,
            parameters=[
                use_sim_time
            ],
            arguments=[
                '-string',
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        LaunchConfiguration('robot_model_path'),
                        LaunchConfiguration('robot_model_file')
                    ]),
                    ' ',
                    LaunchConfiguration('xacro_args')
                ])
            ]
            + spawn_model_pose
            + world,
            condition=UnlessCondition(
                LaunchConfiguration('robot_model_from_topic')
            )
        ),
        Node(
            package=ros_gz_package_name,
            executable='create',
            name=LaunchConfiguration('spawn_node_name'),
            namespace=namespace,
            output=output,
            parameters=[
                use_sim_time
            ],
            arguments=[
                '-topic',
                LaunchConfiguration('robot_description_topic'),
            ]
            + spawn_model_pose
            + world,
            condition=IfCondition(
                LaunchConfiguration('robot_model_from_topic')
            )
        )
    ]


def generate_local_environment_variables():
    this_pkg_share_dir = get_package_share_directory('ros_gz_utils')

    gazebo_env_variables = []

    gz_version_env_name = 'IGNITION_VERSION'

    if os.getenv(gz_version_env_name) is None:
        raise KeyError('Please export ' + gz_version_env_name)
    if os.getenv(gz_version_env_name) == 'garden':
        gazebo_env_variables = [
            SetEnvironmentVariable(
                name='GZ_FILE_PATH',
                value=[
                    EnvironmentVariable(
                        'GZ_FILE_PATH',
                        default_value=''
                    ), ':',
                    os.path.join(
                        this_pkg_share_dir,
                        'models',
                        'urdf'
                    ),
                    LaunchConfiguration('robot_model_path')
                ]
            )
        ]
    else:
        gazebo_env_variables = [
            SetEnvironmentVariable(
                name='IGN_FILE_PATH',
                value=[
                    EnvironmentVariable(
                        'IGN_FILE_PATH',
                        default_value=''
                    ), ':',
                    os.path.join(
                        this_pkg_share_dir,
                        'models',
                        'urdf'
                    ),
                    LaunchConfiguration('robot_model_path')
                ]
            )
        ]

    return gazebo_env_variables + [
        SetEnvironmentVariable(
            name='SDF_PATH',
            value=[
                EnvironmentVariable(
                    'SDF_PATH',
                    default_value=''
                ), ':',
                os.path.join(
                    this_pkg_share_dir,
                    'models',
                    'urdf'
                ),
                LaunchConfiguration('robot_model_path')
            ]
        )
    ]


if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
