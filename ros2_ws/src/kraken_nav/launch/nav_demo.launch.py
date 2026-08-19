# SPDX-License-Identifier: BSD-3-Clause
"""Flat-ground navigation demo: sim, fault injector, EKF and Nav2, no scenario.

Nothing is faulted and nothing is scored. This exists to exercise the Nav2
plumbing by hand:

    ros2 launch kraken_nav nav_demo.launch.py
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
        "{pose: {header: {frame_id: map}, pose: {position: {x: 8.0, y: 4.0}, \\
          orientation: {w: 1.0}}}}"

The fault injector is here because it owns the odom->base_link transform and
the /..../faulted topics the filter subscribes to, not because anything is
being broken.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channels = os.path.join(
        get_package_share_directory('kraken_faults'), 'config', 'channels.yaml')
    localisation = os.path.join(
        get_package_share_directory('kraken_localisation'), 'launch', 'localisation.launch.py')
    navigation = os.path.join(
        get_package_share_directory('kraken_nav'), 'launch', 'navigation.launch.py')
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='robust',
                              choices=['naive', 'robust'],
                              description='EKF tuning under the navigator'),
        # The simulator owns /clock, so it is the one node that must not consume it.
        Node(
            package='kraken_sim', executable='headless_sim', name='headless_sim',
            output='screen', parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='kraken_faults', executable='fault_injector', name='fault_injector',
            output='screen', parameters=[channels, sim_time],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localisation),
            launch_arguments={
                'profile': LaunchConfiguration('profile'),
                'use_sim_time': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
    ])
