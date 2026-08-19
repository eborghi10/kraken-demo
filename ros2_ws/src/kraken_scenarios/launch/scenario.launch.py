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
    background, runner = scenario_actions(name, report)
    return background + [runner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='total_gnss_dropout'),
        DeclareLaunchArgument('report', default_value='',
                              description='write a JSON result here when finished'),
        OpaqueFunction(function=_actions),
    ])
