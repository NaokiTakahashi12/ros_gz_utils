
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable
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
            default_value = ['simulator'],
            description = 'Namespace of ignition gazebo simulator (string)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = ['true'],
            description = 'Subscribe /clock topic (boolean)'
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
                    get_package_share_directory('ros_ign_utils'),
                    'models',
                    'urdf'
                )
            ],
            description = 'Robot model file path (string)'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value = ['default'],
            description = 'Simulation world name of ignition gazebo (string)'
        ),
        DeclareLaunchArgument(
            'spawn_node_name',
            default_value = [AnonName('spawner_node')],
            description = 'Spawn robot node of ignition gazebo (string)'
        )
    ]

def generate_launch_nodes():
    output = 'screen'

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model_file = LaunchConfiguration('robot_model_file')
    robot_model_path = LaunchConfiguration('robot_model_path')
    world_name = LaunchConfiguration('world_name')

    urdf_file = PathJoinSubstitution([
        robot_model_path, robot_model_file
    ])

    return [
        Node(
            package = 'ros_ign_gazebo',
            executable = 'create',
            name = LaunchConfiguration('spawn_node_name'),
            namespace = namespace,
            output = output,
            parameters = [
                {'use_sim_time': use_sim_time},
            ],
            arguments = [
                '-world', world_name,
                '-string', Command(['xacro ', urdf_file]),
                '-x', '0',
                '-y', '0',
                '-z', '0.5',
                '-R', '0',
                '-P', '0',
                '-Y', '0'
            ]
        )
    ]

def generate_local_environment_variables():
    this_pkg_share_dir = get_package_share_directory('ros_ign_utils')

    return [
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
        ),
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
    ]

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()

