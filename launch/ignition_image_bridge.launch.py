
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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
    PythonExpression,
    AnonName
)
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    output = 'screen'

    ign_topic = LaunchConfiguration('ign_topic')
    ros_topic = LaunchConfiguration('ros_topic')
    ign_frame_id = LaunchConfiguration('ign_frame_id')
    ros_frame_id = LaunchConfiguration('ros_frame_id')

    exit_event = EmitEvent(
        event = Shutdown()
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            'with_stf',
            default_value = ['true'],
            description = 'Launch with static transform publisher (boolean)'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'ign_topic',
            description = 'Ignition gazebo topic name (string)'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'ros_topic',
            description = 'ROS2 topic name (string)'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'ign_frame_id',
            description = 'Ignition gazebo transform frame id (string)',
            condition = IfCondition(
                LaunchConfiguration('with_stf')
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'ros_frame_id',
            description = 'ROS2 transform frame id (string)',
            condition = IfCondition(
                LaunchConfiguration('with_stf')
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'bridge_node_name',
            default_value = AnonName('parameter_bridge'),
            description = 'Bridge node name (string)'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'stf_node_name',
            default_value = AnonName('stf'),
            description = 'Static transform publisher node name (string)'
        )
    )

    ld.add_action(
        GroupAction([
            Node(
                package = 'ros_ign_image',
                executable = 'image_bridge',
                name = LaunchConfiguration('bridge_node_name'),
                on_exit = [
                    exit_event
                ],
                output = output,
                parameters = [
                    {'use_sim_time': True}
                ],
                # TODO https://github.com/gazebosim/ros_gz/pull/278
                arguments = [
                    [ign_topic]
                ],
                remappings = [
                    (ign_topic, ros_topic),
                    ([ign_topic, '/compressed'], [ros_topic, '/compressed']),
                    ([ign_topic, '/compressedDepth'], [ros_topic, '/compressedDepth']),
                    ([ign_topic, '/theora'], [ros_topic, '/theora'])
                ]
            ),
            Node(
                package = 'tf2_ros',
                executable = 'static_transform_publisher',
                name = LaunchConfiguration('stf_node_name'),
                output = output,
                parameters = [
                    {'use_sim_time': True}
                ],
                arguments = [
                    '0', '0', '0', '0', '0', '0',
                    ros_frame_id, ign_frame_id
                ],
                condition = IfCondition(
                    LaunchConfiguration('with_stf')
                )
            )
        ])
    )

    return ld

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()

