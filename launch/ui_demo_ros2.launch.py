#!/usr/bin/env python3
"""Launch the ground control UI (ui_demo_ros2) as a pure ROS2 client.

This is intentionally SEPARATE from spirit_triage.launch.py: the UI must NOT come
up with the driver / on container restart. Launch it on demand.

It runs under the same namespace as spirit_driver (default /<drone>/gremsy) and
reads the same per-drone config so its pre-existing-topic subscriptions
(gimbal_orientation) match the driver.

Args:
  drone         Robot name; selects config/<drone>.yaml (default $ROBOT_NAME|spiritnx3).
  topic_prefix  Prefix for the ROS namespace (default = drone).
  namespace     Full node namespace (default /<topic_prefix>/gremsy).
  config_file   Override the per-drone config file path.

Note: this only launches the ROS node. To make the window appear on your laptop,
run it through X11 forwarding -- see scripts/run_ui_demo_ros2.sh.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    drone = LaunchConfiguration('drone').perform(context)
    topic_prefix = LaunchConfiguration('topic_prefix').perform(context) or drone
    namespace = LaunchConfiguration('namespace').perform(context) or f'/{topic_prefix}/gremsy'

    config_file = LaunchConfiguration('config_file').perform(context)
    if not config_file:
        share = get_package_share_directory('gremsy_ros2')
        config_file = os.path.join(share, 'config', f'{drone}.yaml')
    parameters = [config_file] if os.path.isfile(config_file) else []

    node = Node(
        package='gremsy_ros2',
        executable='ui_demo_ros2',
        name='ui_demo_ros2',
        namespace=namespace,
        output='screen',
        parameters=parameters,
    )
    return [node]


def generate_launch_description():
    default_drone = os.environ.get('ROBOT_NAME', 'spiritnx3')
    return LaunchDescription([
        DeclareLaunchArgument('drone', default_value=default_drone),
        DeclareLaunchArgument('topic_prefix', default_value=default_drone),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('config_file', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
