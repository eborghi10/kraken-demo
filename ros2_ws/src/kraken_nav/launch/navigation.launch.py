# SPDX-License-Identifier: BSD-3-Clause
"""Nav2 servers layered on top of an already-running localisation stack.

This launch file assumes someone else supplies /clock, the map->odom->base_link
transform chain and /odometry/filtered. It starts no simulator and no filter,
so it can sit under either the headless sim or O3DE unchanged.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SERVERS = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('kraken_nav'), 'config', 'nav2.yaml')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[params, use_sim_time]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params, use_sim_time]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[params, use_sim_time]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[params, use_sim_time]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[use_sim_time, {'autostart': True, 'node_names': SERVERS}],
        ),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *nodes,
    ])
