# SPDX-License-Identifier: BSD-3-Clause
"""Brings up navsat_transform and one EKF profile.

The `profile` argument selects which filter tuning is under test: `naive` has
no gyro, `robust` fuses IMU yaw rate, `radar` also takes forward speed from a
ground-speed radar instead of the wheels. Everything else about the stack is
identical between them, which is what makes the comparison meaningful.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = get_package_share_directory('kraken_localisation')
    profile = LaunchConfiguration('profile').perform(context)
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    ekf_config = os.path.join(share, 'config', 'ekf_%s.yaml' % profile)
    if not os.path.exists(ekf_config):
        raise RuntimeError('unknown profile %r, expected naive, robust or radar' % profile)

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
            parameters=[use_sim_time],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[os.path.join(share, 'config', 'navsat_transform.yaml'), use_sim_time],
            remappings=[
                ('imu', '/imu/data/faulted'),
                ('gps/fix', '/gnss/fix/faulted'),
                ('odometry/filtered', '/odometry/filtered'),
                ('odometry/gps', '/odometry/gps'),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, use_sim_time],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='robust',
                              choices=['naive', 'robust', 'radar'],
                              description='EKF tuning under test'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])
