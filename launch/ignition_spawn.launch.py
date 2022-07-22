
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
    EnvironmentVariable,
    PathJoinSubstitution,
    Command,
    AnonName
)
from launch.events import Shutdown
from launch_ros.actions import (
    Node,
    PushRosNamespace
)

def generate_launch_description():
    output = 'screen'
    this_pkg_share_dir = get_package_share_directory('ros_ign_utils')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model_file = LaunchConfiguration('robot_model_file')
    robot_model_path = LaunchConfiguration('robot_model_path')
    world_name = LaunchConfiguration('world_name')

    urdf_file = PathJoinSubstitution([
        robot_model_path, robot_model_file
    ])

    exit_event = EmitEvent(
        event = Shutdown()
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            'namespace',
            default_value = ['simulator'],
            description = 'Namespace of ignition gazebo simulator (string)'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = ['true'],
            description = 'Subscribe /clock topic (boolean)'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'robot_model_file',
            default_value = ['test_robot.urdf.xacro'],
            description = 'Robot model file (string)'
        )
    )
    ld.add_action(
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
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'world_name',
            default_value = ['default'],
            description = 'Simulation world name of ignition gazebo (string)'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'joint_state_publisher_config_file',
            default_value = [
                os.path.join(this_pkg_share_dir, 'config', 'joint_state_sources.yaml')
            ],
            description = 'Joint state publisher node parameter file (string)'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'spawn_node_name',
            default_value = ['spawner_node'],
            description = 'Spawn robot node of ignition gazebo (string)'
        )
    )

    ld.add_action(
        SetEnvironmentVariable(
            name = 'SDF_PATH',
            value = [
                EnvironmentVariable(
                    'SDF_PATH',
                    default_value = ''
                ),
                os.path.join(
                    this_pkg_share_dir,
                    'models',
                    'urdf'
                ),
                LaunchConfiguration('robot_model_path')
            ]
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name = 'IGN_FILE_PATH',
            value = [
                EnvironmentVariable(
                    'IGN_FILE_PATH',
                    default_value = ''
                ),
                os.path.join(
                    this_pkg_share_dir,
                    'models',
                    'urdf'
                ),
                LaunchConfiguration('robot_model_path')
            ]
        )
    )

    ld.add_action(
        GroupAction(actions = [
            PushRosNamespace(
                namespace = namespace
            ),
            # TODO Add prefix node name
            Node(
                package = 'robot_state_publisher',
                executable = 'robot_state_publisher',
                name = 'robot_state_publisher',
                output = output,
                parameters = [
                    {'use_sim_time': use_sim_time},
                    {'ignore_timestamp': False},
                    {'publish_frequency': 10.0},
                    {'robot_description': Command(['xacro ', urdf_file])}
                ],
                # TODO https://github.com/ros/joint_state_publisher/issues/82
                remappings = [
                    ('joint_states', 'test_bot_joint_state_bridge/joint_states')
                ]
            ),
            # TODO https://github.com/ros/joint_state_publisher/issues/82
            #Node(
            #    package = 'joint_state_publisher',
            #    executable = 'joint_state_publisher',
            #    name = 'joint_state_merger',
            #    output = output,
            #    parameters = [
            #        {'use_sim_time': use_sim_time},
            #        {'ignore_timestamp': False},
            #        {'publish_default_efforts': True},
            #        {'publish_default_velocities': True},
            #        {'publish_default_positions': True},
            #        LaunchConfiguration('joint_state_publisher_config_file')
            #    ]
            #),
            Node(
                package = 'ros_ign_gazebo',
                executable = 'create',
                name = AnonName(LaunchConfiguration('spawn_node_name')),
                output = output,
                parameters = [
                    {'use_sim_time': use_sim_time},
                ],
                arguments = [
                    '-world', world_name,
                    '-topic', 'robot_description',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.5',
                    '-R', '0',
                    '-P', '0',
                    '-Y', '0'
                ]
            )
        ])
    )

    return ld

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()

