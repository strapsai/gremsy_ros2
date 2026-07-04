#!/usr/bin/env python3
"""Launch the onboard Gremsy bridge (spirit_driver).

Per-drone settings live in config/<drone>.yaml (gremsy_ip and the absolute names
of the pre-existing cross-system topics: gimbal_command from the basestation,
gimbal_orientation / move_gimbal_angle used by logging). The drone is chosen from
$ROBOT_NAME (or the `drone` arg), which selects the config file -- nothing is
hardcoded in the build.

New gremsy-specific command/telemetry topics (cmd/*, params/*, camera/*, ...) are
relative and live under the node namespace (default /<drone>/gremsy).

Args:
  drone         Robot name; selects config/<drone>.yaml (default $ROBOT_NAME|spiritnx3).
  gate          Gate/mission label, passed through as a parameter (optional).
  topic_prefix  Prefix for the ROS namespace (default = drone).
  namespace     Full node namespace (default /<topic_prefix>/gremsy).
  config_file   Override the per-drone config file path.
  gremsy_ip     Override the Gremsy IP from config (optional).

Run the ground UI (same namespace so the relative topics line up):
  ros2 run gremsy_ros2 ui_demo_ros2 --ros-args -r __ns:=/<topic_prefix>/gremsy \
       --params-file <config/<drone>.yaml>
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
    gate = LaunchConfiguration('gate').perform(context)
    gremsy_ip = LaunchConfiguration('gremsy_ip').perform(context)

    config_file = LaunchConfiguration('config_file').perform(context)
    if not config_file:
        share = get_package_share_directory('gremsy_ros2')
        config_file = os.path.join(share, 'config', f'{drone}.yaml')

    parameters = []
    if os.path.isfile(config_file):
        parameters.append(config_file)
    else:
        print(f'[spirit_triage] WARNING: config file not found: {config_file} '
              f'(using node defaults).')
    # Launch-arg overrides (win over the config file).
    overrides = {'gate': gate}
    if gremsy_ip:
        overrides['gremsy_ip'] = gremsy_ip
    parameters.append(overrides)

    node = Node(
        package='gremsy_ros2',
        executable='spirit_driver',
        name='spirit_driver',
        namespace=namespace,
        output='screen',
        parameters=parameters,
    )
    return [node]


def generate_launch_description():
    default_drone = os.environ.get('ROBOT_NAME', 'spiritnx3')
    return LaunchDescription([
        DeclareLaunchArgument('drone', default_value=default_drone),
        DeclareLaunchArgument('gate', default_value=''),
        DeclareLaunchArgument('topic_prefix', default_value=default_drone),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('gremsy_ip', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
