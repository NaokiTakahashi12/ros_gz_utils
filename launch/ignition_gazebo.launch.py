#!/usr/bin/env -S python3

import os

from ament_index_python.packages import get_package_share_directory

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
            'world_path',
            default_value = [''],
            description = 'Simulation world file path of ignition gazebo (string)'
        ),
        DeclareLaunchArgument(
            'resource_package_name',
            default_value = [''],
            description = 'Simulation world recource file path of ignition server (string)'
        ),
        DeclareLaunchArgument(
            'ignition_gazebo_system_plugin_path',
            default_value = [''],
            description = 'Ignition gazebo system plugin path (string)'
        ),
        DeclareLaunchArgument(
            'ignition_gazebo_physics_engine_path',
            default_value = [''],
            description = 'Ignition gazebo physics engine path (string)'
        ),
        DeclareLaunchArgument(
            'use_sim_gui',
            default_value = ['true'],
            description = 'Enable ignition gazebo gui (string)'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value = ['false'],
            description = 'Enable debug output (boolean)'
        )
    ]

def generate_local_environment_variables():
    this_pkg_share_dir = get_package_share_directory('ros_ign_utils')

    return [
        SetEnvironmentVariable(
            name = 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            value = [
                EnvironmentVariable(
                    'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                    default_value = ''
                ), ':',
                EnvironmentVariable(
                    'LD_LIBRARY_PATH',
                    default_value = ''
                ), ':',
                LaunchConfiguration('ignition_gazebo_system_plugin_path')
            ]
        ),
        SetEnvironmentVariable(
            name = 'IGN_GAZEBO_PHYSICS_ENGINE_PATH',
            value = [
                EnvironmentVariable(
                    'IGN_GAZEBO_PHYSICS_ENGINE_PATH',
                    default_value = ''
                ), ':',
                EnvironmentVariable(
                    'LD_LIBRARY_PATH',
                    default_value = ''
                ), ':',
                LaunchConfiguration('ignition_gazebo_physics_engine_path')
            ]
        ),
        SetEnvironmentVariable(
            name = 'IGN_GAZEBO_RESOURCE_PATH',
            value = [
                EnvironmentVariable(
                    'IGN_GAZEBO_RESOURCE_PATH',
                    default_value = ''
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

def generate_launch_nodes():
    output = 'screen'

    namespace = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world_name')
    world_file = LaunchConfiguration('world_file')
    debug_condition = LaunchConfiguration('debug')

    exit_event = EmitEvent(
        event = Shutdown()
    )

    return [
        GroupAction(actions = [
            PushRosNamespace(
                namespace = namespace
            ),
            ExecuteProcess(
                shell = True,
                cmd = [
                    'ign', 'gazebo', '-s', '-r', world_file
                ],
                name = 'ign_gazebo',
                output = output,
                condition = UnlessCondition(debug_condition),
                on_exit = [
                    exit_event
                ]
            ),
            ExecuteProcess(
                shell = True,
                cmd = [
                    'ign', 'gazebo', '-s', '-r', '-v4', world_file
                ],
                name = 'ign_gazebo',
                output = output,
                condition = IfCondition(debug_condition),
                on_exit = [
                    exit_event
                ]
            ),
            ExecuteProcess(
                shell = True,
                cmd = [
                    'ign', 'gazebo', '-g', world_file
                ],
                name = 'ign_gui',
                output = output,
                respawn = 'true',
                condition = IfCondition(LaunchConfiguration('use_sim_gui'))
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/ignition_bridge.launch.py'
                ]),
                launch_arguments = {
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
                launch_arguments = {
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
                launch_arguments = {
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

