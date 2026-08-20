# SPDX-License-Identifier: BSD-3-Clause
"""Run one scenario by hand:

    ros2 launch kraken_scenarios scenario.launch.py scenario:=total_gnss_dropout
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from kraken_scenarios.launch_utils import scenario_actions


def _actions(context):
    name = LaunchConfiguration('scenario').perform(context)
    report = LaunchConfiguration('report').perform(context)
    simulator = LaunchConfiguration('simulator').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    background, runner = scenario_actions(name, report, simulator, seed or None, namespace)
    return background + [runner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='total_gnss_dropout'),
        DeclareLaunchArgument('report', default_value='',
                              description='write a JSON result here when finished'),
        DeclareLaunchArgument('simulator', default_value='headless',
                              description="'headless' starts kraken_sim; 'o3de' expects "
                                          'the O3DE launcher to be running already'),
        DeclareLaunchArgument('seed', default_value='',
                              description="override the scenario's noise seed"),
        DeclareLaunchArgument('namespace', default_value='',
                              description='robot to run under, e.g. kraken1 for O3DE'),
        OpaqueFunction(function=_actions),
    ])
