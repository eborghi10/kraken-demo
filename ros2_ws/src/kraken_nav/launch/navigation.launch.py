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

# The two filter servers come first: a costmap that is told to run a keepout
# filter will wait for the mask before it will activate.
SERVERS = ['filter_mask_server', 'costmap_filter_info_server',
           'controller_server', 'planner_server', 'behavior_server', 'bt_navigator']

# Frames that belong to one robot. `map` is shared and stays as it is.
ROBOT_FRAMES = ('odom', 'base_link')


def _namespaced_params(source, namespace, prefix):
    """Re-root nav2.yaml, prefix its robot frames and absolutise its topics.

    Parameter files are matched by fully qualified node name, so under a
    namespace an unmodified file applies to nothing at all. Frames are rewritten
    by value rather than by key because `global_frame` means the odom frame in
    the local costmap and the map frame in the global one. Observation topics
    are made absolute because a costmap is its own node one level below the
    robot: left relative, `pc` resolves to `<ns>/local_costmap/pc` and the layer
    silently sees nothing.
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
            elif key in ('topic', 'filter_info_topic') and isinstance(value, str) \
                    and not value.startswith('/'):
                node[key] = '/'.join(('', namespace, value)) if namespace else '/' + value

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

    # Passed as their own dict because the values are absolute paths only known
    # at launch time. Both trees drop nav2's Spin recovery, which this robot
    # cannot perform; bt_navigator loads both, so both have to be replaced.
    trees = os.path.join(get_package_share_directory('kraken_nav'), 'behavior_trees')
    behavior_trees = {
        'default_nav_to_pose_bt_xml': os.path.join(
            trees, 'navigate_to_pose_ackermann.xml'),
        'default_nav_through_poses_bt_xml': os.path.join(
            trees, 'navigate_through_poses_ackermann.xml'),
    }

    # The parameter file is keyed by fully qualified node name, so the servers
    # have to be launched into the namespace its keys were written for.
    remappings = [('odometry/filtered', LaunchConfiguration('odom_topic').perform(context))]

    # A keepout mask is served exactly like a map, and a second little server
    # tells the costmap filters how to read it. Both topics have to be absolute:
    # the filter subscribes to the mask under whatever name the info message
    # carries, and it does so from the costmap node, which sits one level below
    # the robot. Left relative it looks for <ns>/global_costmap/... and waits
    # for a publisher that will never appear.
    mask = os.path.join(
        get_package_share_directory('kraken_nav'), 'maps', 'orchard_keepout.yaml')
    mask_topic = '/'.join(('', namespace, 'keepout_filter_mask')) if namespace \
        else '/keepout_filter_mask'
    filters = [
        Node(package='nav2_map_server', executable='map_server', name='filter_mask_server',
             namespace=namespace, output='screen',
             parameters=[use_sim_time,
                         {'yaml_filename': mask, 'topic_name': mask_topic}]),
        Node(package='nav2_map_server', executable='costmap_filter_info_server',
             name='costmap_filter_info_server', namespace=namespace, output='screen',
             parameters=[use_sim_time,
                         {'type': 0, 'filter_info_topic': 'costmap_filter_info',
                          'mask_topic': mask_topic,
                          'base': 0.0, 'multiplier': 1.0}]),
    ]

    return filters + [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             namespace=namespace, output='screen', parameters=[params, use_sim_time],
             remappings=remappings),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             namespace=namespace, output='screen', parameters=[params, use_sim_time]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             namespace=namespace, output='screen', parameters=[params, use_sim_time]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             namespace=namespace, output='screen',
             parameters=[params, use_sim_time, behavior_trees], remappings=remappings),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', namespace=namespace, output='screen',
            parameters=[use_sim_time, {'autostart': True, 'node_names': SERVERS}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('frame_prefix', default_value='',
                              description="TF frame prefix, e.g. 'kraken1/'"),
        DeclareLaunchArgument('odom_topic', default_value='odometry/filtered',
                              description='Odometry the controller tracks'),
        OpaqueFunction(function=_nodes),
    ])
