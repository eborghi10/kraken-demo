# SPDX-License-Identifier: BSD-3-Clause
"""Nav2 servers layered on top of an already-running localisation stack.

This launch file assumes someone else supplies /clock, the map->odom->base_link
transform chain and odometry/filtered. It starts no simulator and no filter,
so it can sit under either the headless sim or O3DE unchanged.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SERVERS = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']

# Frames that belong to one robot. `map` is shared and stays as it is.
ROBOT_FRAMES = ('odom', 'base_link')


def _namespaced_params(source, namespace, prefix):
    """Re-root nav2.yaml and prefix its robot frames.

    Parameter files are matched by fully qualified node name, so under a
    namespace an unmodified file applies to nothing at all. Frames are rewritten
    by value rather than by key because `global_frame` means the odom frame in
    the local costmap and the map frame in the global one.
    """
    if not namespace and not prefix:
        return source

    with open(source, 'r') as handle:
        params = yaml.safe_load(handle)

    def walk(node):
        for key, value in node.items():
            if isinstance(value, dict):
                walk(value)
            elif key.endswith('frame') and value in ROBOT_FRAMES:
                node[key] = prefix + value

    walk(params)
    if namespace:
        params = {namespace: params}

    handle = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.safe_dump(params, handle)
    handle.close()
    return handle.name


def _nodes(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    prefix = LaunchConfiguration('frame_prefix').perform(context)
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    params = _namespaced_params(
        os.path.join(get_package_share_directory('kraken_nav'), 'config', 'nav2.yaml'),
        namespace, prefix)

    return [
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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('frame_prefix', default_value='',
                              description="TF frame prefix, e.g. 'kraken1/'"),
        OpaqueFunction(function=_nodes),
    ])
