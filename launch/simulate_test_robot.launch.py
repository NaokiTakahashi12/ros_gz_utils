#!/usr/bin/env -S python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction
)
from launch.substitutions import (
    LaunchConfiguration,
    ThisLaunchFileDir
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import (
    PushRosNamespace
)

def generate_launch_description():
    return LaunchDescription(
        generate_declare_launch_arguments()
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
            default_value = ['ground_plane'],
            description = 'Simulation world name of ignition gazebo (string)'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value = [LaunchConfiguration('world_name'), '.sdf'],
            description = 'Simulation world file of ignition gazebo (string)'
        ),
        DeclareLaunchArgument(
            'robot_model_file',
            default_value = ['test_robot.urdf.xacro'],
            description = 'Robot model file (string)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = ['true'],
            description = 'Enable simulation time (boolean)'
        ),
        DeclareLaunchArgument(
            'use_sim_gui',
            default_value = ['true'],
            description = 'Enable ignition gazebo gui (string)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value = ['true'],
            description = 'Enable rviz for test_robot (boolean)'
        )
    ]

def generate_launch_nodes():
    output = 'screen'

    namespace = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world_name')
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/ignition_gazebo.launch.py'
            ]),
            launch_arguments = {
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'world_name': world_name,
                'use_sim_gui': LaunchConfiguration('use_sim_gui')
            }.items()
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/ignition_spawn.launch.py'
            ]),
            launch_arguments = {
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'world_name': world_name,
                'robot_model_file': LaunchConfiguration('robot_model_file')
            }.items()
        ),
        GroupAction(actions = [
            PushRosNamespace(
                namespace = namespace
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/ignition_bridge.launch.py'
                ]),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': ['/world/', world_name, '/model/test_robot/joint_state'],
                    'ros_topic': 'joint_states',
                    'convert_args': 'sensor_msgs/msg/JointState[ignition.msgs.Model',
                    'bridge_node_name': 'test_robot_joint_state_bridge'
                }.items()
            )
        ]),
        GroupAction(actions = [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/test_robot.launch.py'
                ]),
                launch_arguments = {
                    # Launch arguments chain guard
                    'namespace': '',
                    'use_sim_time': use_sim_time,
                    'use_rviz': LaunchConfiguration('use_rviz')
                }.items()
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/ignition_bridge.launch.py'
                ]),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': ['test_robot/upper_body_link/upper_imu'],
                    'ros_frame_id': 'upper_imu_link',
                    'ign_topic': [
                        '/world/', world_name, '/model/test_robot/link/upper_body_link/sensor/upper_imu/imu'
                    ],
                    'ros_topic': 'upper_imu/data_raw',
                    'convert_args': 'sensor_msgs/msg/Imu[ignition.msgs.IMU',
                }.items()
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/ignition_bridge.launch.py'
                ]),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': ['test_robot/base_link/lower_imu'],
                    'ros_frame_id': 'lower_imu_link',
                    'ign_topic': [
                        '/world/', world_name, '/model/test_robot/link/base_link/sensor/lower_imu/imu'
                    ],
                    'ros_topic': 'lower_imu/data_raw',
                    'convert_args': 'sensor_msgs/msg/Imu[ignition.msgs.IMU',
                }.items()
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/ignition_bridge.launch.py'
                ]),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': ['test_robot/upper_body_link/front_camera'],
                    'ros_frame_id': 'front_camera_link',
                    'ign_topic': [
                        '/world/', world_name, '/model/test_robot/link/upper_body_link/sensor/front_camera/camera_info'
                    ],
                    'ros_topic': 'front_camera/camera_info',
                    'convert_args': 'sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
                }.items()
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/ignition_image_bridge.launch.py'
                ]),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/test_robot/link/upper_body_link/sensor/front_camera/image'
                    ],
                    'ros_topic': 'front_camera/image'
                }.items()
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/ignition_image_bridge.launch.py'
                ]),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/test_robot/link/upper_body_link/sensor/front_camera/depth_image'
                    ],
                    'ros_topic': 'front_camera/depth_image'
                }.items()
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    ThisLaunchFileDir(), '/ignition_bridge.launch.py'
                ]),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/test_robot/link/upper_body_link/sensor/front_camera/points'
                    ],
                    'ros_topic': 'front_camera/points',
                    'convert_args': 'sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
                }.items()
            )
        ])
    ]

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()

