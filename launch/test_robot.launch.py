
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
    Node,
    PushRosNamespace
)

def generate_launch_description():
    output = 'screen'
    this_pkg_share_dir = get_package_share_directory('ros_ign_utils')

    namespace = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world_name')
    world_file = LaunchConfiguration('world_file')

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
            'world_name',
            default_value = ['ground_plane'],
            description = 'Simulation world name of ignition gazebo (string)'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'world_file',
            default_value = [world_name, '.sdf'],
            description = 'Simulation world file of ignition gazebo (string)'
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
            'use_sim_gui',
            default_value = ['true'],
            description = 'Enable ignition gazebo gui (string)'
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/ignition_gazebo.launch.py'
            ]),
            launch_arguments = {
                'namespace': namespace,
                'use_sim_time': 'true',
                'world_name': world_name,
                'use_sim_gui': LaunchConfiguration('use_sim_gui')
            }.items()
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/ignition_spawn.launch.py'
            ]),
            launch_arguments = {
                'namespace': namespace,
                'use_sim_time': 'true',
                'world_name': world_name,
                'robot_model_file': LaunchConfiguration('robot_model_file')
            }.items()
        )
    )

    return ld

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()

