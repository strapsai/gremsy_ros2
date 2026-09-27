#!/usr/bin/env python3
"""Launch the ground control UI (ui_demo_ros2) as a pure ROS2 client.

This is intentionally SEPARATE from spirit_triage.launch.py: the UI must NOT come
up with the driver / on container restart. Launch it on demand.

The UI addresses the drone by NAME: on Connect it binds every topic to
/<drone>/gremsy/... (absolute), so the node namespace no longer has to match
the driver's. It still reads the same per-drone config so its pre-existing-topic
subscriptions (gimbal_orientation) match the driver.

Args:
  drone         Robot name; selects config/<drone>.yaml (default $ROBOT_NAME|spiritnx3).
  topic_prefix  Prefix for the ROS namespace (default = drone).
  namespace     Full node namespace (default /<topic_prefix>/gremsy).
  config_file   Override the per-drone config file path.

Intended to run ON THE GROUND MACHINE (x86 container, see launch/spirit/x86.env
and the dtc-drivers-ui service) with only ROS2/DDS crossing the network. It can
also run on the robot over X11 forwarding (scripts/run_ui_demo_ros2.sh).
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
        # Same order as spirit_triage.launch.py: fleet-level file, then the
        # package copy — the UI must read whatever the driver read.
        fleet = os.path.join(os.environ.get('AIRLAB_PATH', ''),
                             'config', 'spirit_drivers', f'{drone}.yaml')
        share = get_package_share_directory('gremsy_ros2')
        packaged = os.path.join(share, 'config', f'{drone}.yaml')
        config_file = fleet if os.path.isfile(fleet) else packaged
    parameters = [config_file] if os.path.isfile(config_file) else []
    # The drone name prefills the UI's "Drone" field and selects the
    # /<drone>/gremsy topic tree it binds to on Connect.
    parameters.append({'drone': drone})

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
