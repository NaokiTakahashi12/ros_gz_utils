
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
    EnvironmentVariable,
    AnonName
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
            'rviz2_node_name',
            default_value = ['rviz2'],
            description = 'Namespace of ignition gazebo simulator (string)'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'rviz2_config_file',
            default_value = [os.path.join(
                this_pkg_share_dir,
                'rviz',
                'test_robot.rviz'
            )],
            description = 'rviz2 config file path (string)'
        )
    )

    ld.add_action(
        GroupAction(actions = [
            PushRosNamespace(
                namespace = namespace
            ),
            Node(
                package = 'rviz2',
                executable = 'rviz2',
                name = AnonName(
                    LaunchConfiguration('rviz2_node_name')
                ),
                output = output,
                arguments = [
                    '-d',
                    LaunchConfiguration('rviz2_config_file')
                ]
            )
        ])
    )

    return ld

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()

