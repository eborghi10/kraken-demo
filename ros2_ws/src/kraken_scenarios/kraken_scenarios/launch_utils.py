# SPDX-License-Identifier: BSD-3-Clause
"""Shared composition of a scenario run.

The interactive launch file and the automated tests must bring up byte-for-byte
the same stack, otherwise a green test says nothing about what you see when you
run the demo by hand. Both go through here.
"""
import json
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def scenario_path(name):
    return os.path.join(
        get_package_share_directory('kraken_scenarios'), 'scenarios', name + '.yaml')


def load_scenario(name):
    with open(scenario_path(name), 'r') as handle:
        return yaml.safe_load(handle)


def scenario_actions(name, report='', simulator='headless', seed=None, namespace=''):
    """Return (background actions, scenario runner action) for one scenario.

    `simulator` picks who owns /clock and the sensor topics. 'headless' starts
    kraken_sim; 'o3de' starts nothing and expects an already-running O3DE
    launcher to satisfy the same topic contract (see Project/README.md).

    `seed` overrides the scenario's own seed, which is how the sweep tool walks
    a scenario across noise realisations.

    `namespace` puts the whole stack under one robot, which is what O3DE needs:
    it publishes `/<robot>/imu/data` and tags its TF frames `<robot>/base_link`.
    Every topic in this stack is relative so that it can follow. TF stays global,
    because there is only ever one tree.
    """
    if simulator not in ('headless', 'o3de'):
        raise ValueError("simulator must be 'headless' or 'o3de', got %r" % (simulator,))

    scenario = load_scenario(name)
    seed = scenario.get('seed', 0) if seed is None else int(seed)
    sim_time = {'use_sim_time': True}
    prefix = namespace + '/' if namespace else ''

    channels = os.path.join(
        get_package_share_directory('kraken_faults'), 'config', 'channels.yaml')
    localisation = os.path.join(
        get_package_share_directory('kraken_localisation'), 'launch', 'localisation.launch.py')

    background = []
    if simulator == 'headless':
        background += [
            # The simulator owns /clock, so it is the one node that must not consume it.
            Node(
                package='kraken_sim', executable='headless_sim', name='headless_sim',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'seed': seed,
                    'real_time_factor': float(scenario.get('real_time_factor', 3.0)),
                    # Flat ground unless the scenario says otherwise, so adding
                    # terrain cannot move the numbers of scenarios without it.
                    'terrain': json.dumps(scenario['terrain']) if 'terrain' in scenario else '',
                    'world_frame': prefix + 'world',
                    'odom_frame': prefix + 'odom',
                    'base_frame': prefix + 'base_link',
                    'imu_frame': prefix + 'imu_link',
                }],
            ),
            # O3DE publishes its own sensor frames; the headless sim reports an
            # IMU frame it never puts in TF.
            Node(
                package='tf2_ros', executable='static_transform_publisher', name='base_to_imu',
                arguments=['--frame-id', prefix + 'base_link',
                           '--child-frame-id', prefix + 'imu_link'],
                parameters=[sim_time],
            ),
        ]

    background += [
        Node(
            package='kraken_faults', executable='fault_injector', name='fault_injector',
            output='screen', parameters=[channels, dict(sim_time, seed=seed)],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localisation),
            launch_arguments={
                'profile': scenario.get('profile', 'robust'),
                'use_sim_time': 'true',
                'frame_prefix': prefix,
            }.items(),
        ),
        Node(
            package='kraken_scenarios', executable='scorer', name='scorer',
            output='screen', parameters=[sim_time],
        ),
    ]

    if scenario.get('navigation', False):
        navigation = os.path.join(
            get_package_share_directory('kraken_nav'), 'launch', 'navigation.launch.py')
        background.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation),
                launch_arguments={
                    'use_sim_time': 'true',
                    'namespace': namespace,
                    'frame_prefix': prefix,
                }.items(),
            )
        )

    runner = Node(
        package='kraken_scenarios', executable='scenario_runner', name='scenario_runner',
        output='screen',
        parameters=[dict(sim_time, scenario=scenario_path(name), report=report)],
    )

    if namespace:
        return ([GroupAction([PushRosNamespace(namespace), *background])],
                GroupAction([PushRosNamespace(namespace), runner]))
    return background, runner
