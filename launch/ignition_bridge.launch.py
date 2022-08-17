
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
    output = 'screen'

    ign_topic = LaunchConfiguration('ign_topic')
    ros_topic = LaunchConfiguration('ros_topic')
    convert_args = LaunchConfiguration('convert_args')
    ign_frame_id = LaunchConfiguration('ign_frame_id')
    ros_frame_id = LaunchConfiguration('ros_frame_id')

    exit_event = EmitEvent(
        event = Shutdown()
    )
    bridge_node_argument = [ign_topic, '@', convert_args]

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
            'convert_args',
            description = 'Convert message argument for ros_ign parameter_bridge (string)'
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
            default_value = 'stf',
            description = 'Static transform publisher node name (string)'
        )
    )

    ld.add_action(
        GroupAction([
            Node(
                package = 'ros_ign_bridge',
                executable = 'parameter_bridge',
                name = LaunchConfiguration('bridge_node_name'),
                on_exit = [
                    exit_event
                ],
                output = output,
                parameters = [
                    {'use_sim_time': True}
                ],
                arguments = [
                    [ign_topic, '@', convert_args]
                ],
                remappings = [
                    (ign_topic, ros_topic)
                ]
            ),
            Node(
                package = 'tf2_ros',
                executable = 'static_transform_publisher',
                name = AnonName(
                    LaunchConfiguration('stf_node_name')
                ),
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

