#!/usr/bin/env -S python3

import os
import sys
import yaml
import string
import re

from typing import NamedTuple

from ament_index_python.packages import get_package_share_directory

from launch import (
    LaunchDescription,
    LaunchContext
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction
)
from launch.substitutions import (
    LaunchConfiguration,
    ThisLaunchFileDir,
    AnonName
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import (
    Node,
    PushRosNamespace
)


def generate_launch_description():
    launch_description = LaunchDescription(
        generate_declare_launch_arguments()
    )

    def load_launch_arguments(
        context: LaunchContext,
        config_file,
        robot_name,
        world_name):

        config_file_str = context.perform_substitution(config_file)
        robot_name_str = context.perform_substitution(robot_name)
        world_name_str = context.perform_substitution(world_name)

        config_parser = MultipleIgnBridgeConfigParser()

        config_parser.set_robot_name(robot_name_str)
        config_parser.set_world_name(world_name_str)
        config_parser.open(config_file_str)

        bridge_launch = {
            'bridge':
                [ThisLaunchFileDir(), '/ignition_bridge.launch.py'],
            'image_bridge':
                [ThisLaunchFileDir(), '/ignition_image_bridge.launch.py']
        }

        for bridge_node_name in config_parser.bridge_configs:
            launch_file = bridge_launch[
                config_parser.get_bridge_type(
                    bridge_node_name
                )
            ]

            launch_description.add_action(GroupAction(actions = [
                    PushRosNamespace(
                        namespace = config_parser.get_bridge_namespace(
                            bridge_node_name
                        )
                    ),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            launch_file
                        ),
                        launch_arguments = config_parser.get_launch_arguments(
                            bridge_node_name
                        )
                    )
                ])
            )

    launch_description.add_action(
        GroupAction(actions = [
            OpaqueFunction(
                function = load_launch_arguments,
                args = [
                    LaunchConfiguration('multiple_bridge_config'),
                    LaunchConfiguration('robot_name'),
                    LaunchConfiguration('world_name')
                ]
            )
        ])
    )

    return launch_description


def generate_declare_launch_arguments():
    this_pkg_share_dir = get_package_share_directory('ros_gz_utils')

    return [
        DeclareLaunchArgument(
            'multiple_bridge_config',
            default_value = [
                os.path.join(
                    this_pkg_share_dir,
                    'config',
                    'multiple_ros_ign_bridge.yaml'
                )
            ],
            description = 'Multiple ros_ign bridge configulation file path (string)'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value = ['checker_ground_plane'],
            description = 'Simulation world name of ignition gazebo (string)'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value = ['test_robot'],
            description = 'Simulate robot model name (string)'
        ),
    ]

class MultipleIgnBridgeConfigParser(object):
    def __init__(self):
        self.config_namespace = 'multiple_ignition_bridge'
        self.bridge_configs = {}
        self.robot_name = ''
        self.world_name = ''

    def open(self, filename):
        with open(filename, 'r') as file:
            yaml_loader = yaml.SafeLoader

            def string_constructor(loader, node):
                context = {
                    'robot': self.robot_name,
                    'world': self.world_name
                }
                template = string.Template(node.value)
                return template.substitute(context)

            string_resolver = 'tag:yaml.org.2002:str'

            yaml_loader.add_constructor(
                string_resolver,
                string_constructor
            )
            yaml_loader.add_implicit_resolver(
                string_resolver,
                re.compile(r'(.*)\$\{(.*)\}'),
                None
            )
            yaml_config = yaml.load(
                file,
                Loader = yaml_loader
            )

            if yaml_config.get(self.config_namespace) == None:
                print('Not found bridge configuration namespace')
                sys.exit(1)

            self.bridge_configs = yaml_config[self.config_namespace]

    def get_node_names(self):
        return self.bridge_configs.values()

    def get_bridge_type(self, node_name):
        type = self.bridge_configs[node_name].get('type')

        assert type != None

        return type

    def get_bridge_namespace(self, node_name):
        namespace = self.bridge_configs[node_name].get('namespace')

        if namespace == None:
            namespace = ''

        return namespace

    def get_launch_arguments(self, node_name):
        launch_arguments = self._arguments_filter(node_name)
        return launch_arguments.items()

    def set_robot_name(self, name):
        self.robot_name = name

    def set_world_name(self, name):
        self.world_name = name

    def _arguments_filter(self, node_name):
        config = self.bridge_configs[node_name]

        # Default launch arguments
        with_stf = self._get_param_from_config(config, 'with_stf', 'false')
        ign_frame_id = self._get_param_from_config(config, 'ign_frame_id')
        ros_frame_id = self._get_param_from_config(config, 'ros_frame_id')
        ign_topic = self._get_param_from_config(config, 'ign_topic')
        ros_topic = self._get_param_from_config(config, 'ros_topic')
        convert_args = self._get_param_from_config(config, 'convert_args')
        type = self._get_param_from_config(config, 'type')

        assert type
        assert ign_topic or ros_topic

        if with_stf == 'true':
            assert ign_frame_id
            assert ros_frame_id
        if type != 'image_bridge':
            assert convert_args

        return {
            'with_stf': with_stf,
            'ign_frame_id': ign_frame_id,
            'ros_frame_id': ros_frame_id,
            'ign_topic': ign_topic,
            'ros_topic': ros_topic,
            'convert_args': convert_args,
            'bridge_node_name': node_name,
            'stf_node_name': node_name + '_stf'
        }

    def _get_param_from_config(self, config, key, default=''):
        param = config.get(key)

        if param == None:
            param = default

        return param


if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()

