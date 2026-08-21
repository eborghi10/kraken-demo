# SPDX-License-Identifier: BSD-3-Clause
"""Navigation for a Kraken already spawned in the O3DE orchard.

    ros2 run kraken_scenarios sim_admin spawn line4 kraken1
    ros2 launch kraken_nav orchard.launch.py namespace:=kraken1

O3DE supplies /clock, the lidar cloud and odometry, but publishes no transform
and speaks AckermannDrive rather than Twist, so small nodes sit between it and
Nav2.

The `localisation` argument decides where the map frame comes from, and it is
the difference between driving one row and covering an orchard:

`static` pins map to odom. Nav2 then works entirely in the frame the wheels
believe in. Measured against the simulator's ground truth, that frame is
excellent along a row and falls apart in the turn: 20 m of straight cost 1.78 m
of error, and a single hard turn reported 149.7 degrees where the robot had
actually turned 124.1. Two turns in and the trees already marked in the costmap
sit on top of the robot, so the planner refuses its own start pose and the
coordinates of the next aisle point into a trunk.

`ekf` runs the fault injector and the robot_localization pair underneath, so
map comes from GNSS and the IMU and stops drifting. Odometry still supplies the
odom->base_link link, which is exactly the split the frames are meant to have:
odom smooth but drifting, map absolute but corrected.

The simulator odometry argument is deliberately not called odom_topic: an
included launch file inherits the parent's configurations, and a name shared
with navigation.launch.py would silently repoint Nav2 at the raw best effort
topic it cannot subscribe to.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _odom_only(namespace, sim_odom, sim_time):
    """map welded to odom, and the raw simulator odometry made usable."""
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
    ]


def _ekf(namespace, profile, heading_deg, sim_time):
    """GNSS and IMU corrected map, with odom->base_link off the wheels.

    The fault injector is in the healthy path here rather than only in fault
    runs: it already owns the republishing every channel needs, from best
    effort to reliable, and it broadcasts odom->base_link from the wheel
    channel. That makes odom_tf redundant, and leaves one place to later break
    a sensor on purpose without rewiring anything.

    The filter is given a starting heading. Nothing in this stack measures an
    absolute one: the EKF takes yaw rate from the IMU and position from GNSS,
    so heading is only observable once the machine is moving and GNSS fixes can
    be compared against the direction it believes it is driving. Left at zero,
    a robot parked at the mouth of a row that runs south is told it is facing
    east, and Nav2 plans and steers 90 degrees out until the error drives
    itself out. The rows of this orchard all face the same way, so that heading
    is known before the machine is switched on.
    """
    channels = os.path.join(
        get_package_share_directory('kraken_faults'), 'config', 'channels.yaml')
    localisation = os.path.join(
        get_package_share_directory('kraken_localisation'), 'launch', 'localisation.launch.py')

    # x, y, z, roll, pitch, yaw, then velocities and accelerations.
    initial_state = [0.0] * 15
    initial_state[5] = math.radians(float(heading_deg))

    return [
        GroupAction([
            PushRosNamespace(namespace),
            Node(
                package='kraken_faults', executable='fault_injector', name='fault_injector',
                output='screen', parameters=[channels, sim_time],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localisation),
                launch_arguments={
                    'profile': profile,
                    'use_sim_time': 'true',
                    'frame_prefix': namespace + '/',
                    'initial_yaw': str(initial_state[5]),
                }.items(),
            ),
        ]),
    ]


def _nodes(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    sim_odom = LaunchConfiguration('sim_odom').perform(context)
    localisation = LaunchConfiguration('localisation').perform(context)
    profile = LaunchConfiguration('profile').perform(context)
    sim_time = {'use_sim_time': True}

    navigation = os.path.join(
        get_package_share_directory('kraken_nav'), 'launch', 'navigation.launch.py')

    if localisation == 'ekf':
        frames = _ekf(namespace, profile,
                      LaunchConfiguration('row_heading_deg').perform(context), sim_time)
    else:
        frames = _odom_only(namespace, sim_odom, sim_time)

    return frames + [
        Node(
            package='kraken_sim', executable='ackermann_bridge', name='ackermann_bridge',
            namespace=namespace, output='screen', parameters=[sim_time],
        ),
        # Nav2 comes up last, and not at the same moment as everything else.
        # Its lifecycle manager configures each server as soon as it starts,
        # and if the server's transition service has not been discovered by
        # then the manager does not wait: it reports a failed transition and
        # abandons the whole bringup, leaving a stack that looks alive but
        # aborts every goal instantly. Adding the localisation nodes made that
        # race a regular occurrence, so the discovery traffic is allowed to
        # settle first.
        TimerAction(period=8.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation),
                launch_arguments={
                    'use_sim_time': 'true',
                    'namespace': namespace,
                    'frame_prefix': namespace + '/',
                }.items(),
            ),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='kraken1'),
        DeclareLaunchArgument('sim_odom', default_value='wheel/odom',
                              description='O3DE odometry to broadcast as tf and republish'),
        DeclareLaunchArgument('localisation', default_value='ekf',
                              choices=['static', 'ekf'],
                              description='where the map frame comes from'),
        DeclareLaunchArgument('profile', default_value='robust',
                              choices=['naive', 'robust', 'radar'],
                              description='EKF tuning, when localisation is ekf'),
        DeclareLaunchArgument('row_heading_deg', default_value='-90.0',
                              description='which way the rows run in the map frame, '
                                          'used to start the filter off facing the right way'),
        OpaqueFunction(function=_nodes),
    ])
