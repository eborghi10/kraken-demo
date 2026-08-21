# SPDX-License-Identifier: BSD-3-Clause
"""Navigation for a Kraken already spawned in the O3DE orchard.

    ros2 run kraken_scenarios sim_admin spawn line4 kraken1
    ros2 launch kraken_nav orchard.launch.py namespace:=kraken1

O3DE supplies /clock, the lidar cloud and odometry, but publishes no transform
and speaks AckermannDrive rather than Twist, so three small nodes sit between
it and Nav2. map is pinned to odom: there is no global localisation here, both
costmaps are rolling windows, and every goal is given relative to where the
robot believes it is. Wheel odometry drifts, so a long mission will need the
EKF and GNSS underneath this rather than a static link.

The simulator odometry argument is deliberately not called odom_topic: an
included launch file inherits the parent's configurations, and a name shared
with navigation.launch.py would silently repoint Nav2 at the raw best effort
topic it cannot subscribe to.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    sim_odom = LaunchConfiguration('sim_odom').perform(context)
    sim_time = {'use_sim_time': True}

    navigation = os.path.join(
        get_package_share_directory('kraken_nav'), 'launch', 'navigation.launch.py')

    return [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom', namespace=namespace, output='screen',
            arguments=['--frame-id', 'map', '--child-frame-id', namespace + '/odom'],
            parameters=[sim_time],
        ),
        Node(
            package='kraken_sim', executable='odom_tf', name='odom_tf',
            namespace=namespace, output='screen', parameters=[sim_time],
            remappings=[('odom', sim_odom)],
        ),
        Node(
            package='kraken_sim', executable='ackermann_bridge', name='ackermann_bridge',
            namespace=namespace, output='screen', parameters=[sim_time],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation),
            launch_arguments={
                'use_sim_time': 'true',
                'namespace': namespace,
                'frame_prefix': namespace + '/',
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='kraken1'),
        DeclareLaunchArgument('sim_odom', default_value='wheel/odom',
                              description='O3DE odometry to broadcast as tf and republish'),
        OpaqueFunction(function=_nodes),
    ])
