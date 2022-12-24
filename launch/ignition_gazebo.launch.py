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
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
    GroupAction,
    EmitEvent
)
from launch.conditions import (
    IfCondition,
    UnlessCondition
)
from launch.substitutions import (
    LaunchConfiguration,
    ThisLaunchFileDir,
    EnvironmentVariable
)
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import (
    PushRosNamespace
)


def generate_launch_description():
    return LaunchDescription(
        generate_declare_launch_arguments()
        + generate_local_environment_variables()
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
            'world_path',
            default_value=[''],
            description='Simulation world file path of ignition gazebo (string)'
        ),
        DeclareLaunchArgument(
            'resource_package_name',
            default_value=[''],
            description='Simulation world recource file path of ignition server (string)'
        ),
        DeclareLaunchArgument(
            'model_resource_path',
            default_value=[''],
            description='Simulation model recource file path of ignition server (string)'
        ),
        DeclareLaunchArgument(
            'ignition_gazebo_system_plugin_path',
            default_value=[''],
            description='Ignition gazebo system plugin path (string)'
        ),
        DeclareLaunchArgument(
            'ignition_gazebo_physics_engine_path',
            default_value=[''],
            description='Ignition gazebo physics engine path (string)'
        ),
        DeclareLaunchArgument(
            'use_sim_gui',
            default_value=['true'],
            description='Enable ignition gazebo gui (string)'
        ),
        DeclareLaunchArgument(
            'sim_gui_config_file_path',
            default_value=[
                os.path.join(
                    this_pkg_share_dir,
                    'config',
                    'ign_gui.config'
                )
            ],
            description='Ignition gazebo GUI configuration file full path (string)'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value=['false'],
            description='Enable debug output (boolean)'
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
                name='GZ_SIM_SYSTEM_PLUGIN_PATH',
                value=[
                    EnvironmentVariable(
                        'GZ_GAZEBO_SYSTEM_PLUGIN_PATH',
                        default_value=''
                    ), ':',
                    EnvironmentVariable(
                        'LD_LIBRARY_PATH',
                        default_value=''
                    ), ':',
                    LaunchConfiguration('ignition_gazebo_system_plugin_path')
                ]
            ),
            SetEnvironmentVariable(
                name='GZ_GAZEBO_PHYSICS_ENGINE_PATH',
                value=[
                    EnvironmentVariable(
                        'GZ_GAZEBO_PHYSICS_ENGINE_PATH',
                        default_value=''
                    ), ':',
                    EnvironmentVariable(
                        'LD_LIBRARY_PATH',
                        default_value=''
                    ), ':',
                    LaunchConfiguration('ignition_gazebo_physics_engine_path')
                ]
            ),
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
                    ), ':',
                    LaunchConfiguration('model_resource_path')
                ]
            ),
            SetEnvironmentVariable(
                name='GZ_SIM_RESOURCE_PATH',
                value=[
                    EnvironmentVariable(
                        'GZ_GAZEBO_RESOURCE_PATH',
                        default_value=''
                    ), ':',
                    os.path.join(
                        this_pkg_share_dir, 'worlds', 'ignition'
                    ), ':',
                    os.path.join(
                        this_pkg_share_dir, '..'
                    ), ':',
                    LaunchConfiguration('resource_package_name'), '/..:',
                    LaunchConfiguration('world_path')
                ]
            )
        ]
    else:
        gazebo_env_variables = [
            SetEnvironmentVariable(
                name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                value=[
                    EnvironmentVariable(
                        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                        default_value=''
                    ), ':',
                    addional_colcon_ws_gz_plugins,
                    EnvironmentVariable(
                        'LD_LIBRARY_PATH',
                        default_value=''
                    ), ':',
                    LaunchConfiguration('ignition_gazebo_system_plugin_path')
                ]
            ),
            SetEnvironmentVariable(
                name='IGN_GAZEBO_PHYSICS_ENGINE_PATH',
                value=[
                    EnvironmentVariable(
                        'IGN_GAZEBO_PHYSICS_ENGINE_PATH',
                        default_value=''
                    ), ':',
                    EnvironmentVariable(
                        'LD_LIBRARY_PATH',
                        default_value=''
                    ), ':',
                    LaunchConfiguration('ignition_gazebo_physics_engine_path')
                ]
            ),
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
                    ), ':',
                    LaunchConfiguration('model_resource_path')
                ]
            ),
            SetEnvironmentVariable(
                name='IGN_GAZEBO_RESOURCE_PATH',
                value=[
                    EnvironmentVariable(
                        'IGN_GAZEBO_RESOURCE_PATH',
                        default_value=''
                    ), ':',
                    os.path.join(
                        this_pkg_share_dir, 'worlds', 'ignition'
                    ), ':',
                    os.path.join(
                        this_pkg_share_dir, '..'
                    ), ':',
                    LaunchConfiguration('resource_package_name'), '/..:',
                    LaunchConfiguration('world_path')
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
                ), ':',
                LaunchConfiguration('model_resource_path')
            ]
        )
    ]


def generate_launch_nodes():
    output = 'screen'

    namespace = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world_name')
    world_file = LaunchConfiguration('world_file')
    sim_gui_config_file_path = LaunchConfiguration('sim_gui_config_file_path')
    debug_condition = LaunchConfiguration('debug')

    exit_event = EmitEvent(
        event=Shutdown()
    )

    gz_version_env_name = 'IGNITION_VERSION'
    simulator_command = []

    if os.getenv(gz_version_env_name) is None:
        raise KeyError('Please export ' + gz_version_env_name)
    if os.getenv(gz_version_env_name) == 'garden':
        simulator_command = ['gz', 'sim']
    else:
        simulator_command = ['ign', 'gazebo']

    return [
        GroupAction(actions=[
            PushRosNamespace(
                namespace=namespace
            ),
            ExecuteProcess(
                shell=True,
                cmd=simulator_command + [
                    '-s', '-r', world_file
                ],
                name='ign_gazebo',
                output=output,
                condition=UnlessCondition(debug_condition),
                on_exit=[
                    exit_event
                ]
            ),
            ExecuteProcess(
                shell=True,
                cmd=simulator_command + [
                    '-s', '-r', '-v4', world_file
                ],
                name='ign_gazebo',
                output=output,
                condition=IfCondition(debug_condition),
                on_exit=[
                    exit_event
                ]
            ),
            ExecuteProcess(
                shell=True,
                cmd=simulator_command + [
                    '-g', world_file,
                    '--gui-config', sim_gui_config_file_path
                ],
                name='ign_gui',
                output=output,
                respawn=True,
                condition=IfCondition(LaunchConfiguration('use_sim_gui'))
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/ignition_bridge.launch.py'
                ]),
                launch_arguments={
                    'with_stf': 'false',
                    'ign_topic': ['/world/', world_name, '/clock'],
                    'ros_topic': '/clock',
                    'convert_args': 'rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                }.items()
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/ignition_bridge.launch.py'
                ]),
                launch_arguments={
                    'with_stf': 'false',
                    'ign_topic': '/world/default/pose/info',
                    'ros_topic': 'tf_static',
                    'convert_args': 'tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                }.items()
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/ignition_bridge.launch.py'
                ]),
                launch_arguments={
                    'with_stf': 'false',
                    'ign_topic': '/world/default/dynamic_pose/info',
                    'ros_topic': 'tf',
                    'convert_args': 'tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                }.items()
            )
        ])
    ]


if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
