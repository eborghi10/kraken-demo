# SPDX-License-Identifier: BSD-3-Clause
"""Shared composition of a scenario run.

The interactive launch file and the automated tests must bring up byte-for-byte
the same stack, otherwise a green test says nothing about what you see when you
run the demo by hand. Both go through here.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def scenario_path(name):
    return os.path.join(
        get_package_share_directory('kraken_scenarios'), 'scenarios', name + '.yaml')


def load_scenario(name):
    with open(scenario_path(name), 'r') as handle:
        return yaml.safe_load(handle)


def scenario_actions(name, report=''):
    """Return (background actions, scenario runner action) for one scenario."""
    scenario = load_scenario(name)
    seed = scenario.get('seed', 0)
    sim_time = {'use_sim_time': True}

    channels = os.path.join(
        get_package_share_directory('kraken_faults'), 'config', 'channels.yaml')
    localisation = os.path.join(
        get_package_share_directory('kraken_localisation'), 'launch', 'localisation.launch.py')

    background = [
        # The simulator owns /clock, so it is the one node that must not consume it.
        Node(
            package='kraken_sim', executable='headless_sim', name='headless_sim',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'seed': seed,
                'real_time_factor': float(scenario.get('real_time_factor', 3.0)),
            }],
        ),
        Node(
            package='kraken_faults', executable='fault_injector', name='fault_injector',
            output='screen', parameters=[channels, dict(sim_time, seed=seed)],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localisation),
            launch_arguments={
                'profile': scenario.get('profile', 'robust'),
                'use_sim_time': 'true',
            }.items(),
        ),
        Node(
            package='kraken_scenarios', executable='scorer', name='scorer',
            output='screen', parameters=[sim_time],
        ),
    ]

    runner = Node(
        package='kraken_scenarios', executable='scenario_runner', name='scenario_runner',
        output='screen',
        parameters=[dict(sim_time, scenario=scenario_path(name), report=report)],
    )
    return background, runner
