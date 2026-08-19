# SPDX-License-Identifier: BSD-3-Clause
"""Launch tests: bring up the whole stack, run a scenario, judge the result.

One launch per scenario, the same way rostest worked in ROS 1. The scenario
files own the thresholds so that adding a failure mode does not mean editing
this file.
"""
import json
import os
import unittest

import launch_testing
import launch_testing.asserts
import pytest
from launch import LaunchDescription

from kraken_scenarios.launch_utils import load_scenario, scenario_actions

SCENARIOS = [
    'total_gnss_dropout',
    'gnss_dropout_naive',
    'gnss_degraded',
    'gnss_spoof',
    'imu_dropout',
    'wheel_slip',
]

REPORT_DIR = os.environ.get('KRAKEN_REPORT_DIR', '/tmp/kraken_reports')

# Every expectation key maps to a field of the report and a comparison.
#
# `max_*` bounds are checked against the running worst case, `max_final_*`
# against the value at the end of the run. The distinction matters: losing a
# sensor mid-turn produces a transient that the filter then recovers from, and
# for some faults that transient is acceptable while a permanent offset is not.
# Scenarios say which of the two they care about.
CHECKS = {
    'max_position_error': ('worst_position_error', lambda got, want: got < want, 'below'),
    'max_heading_error_deg': ('worst_heading_error_deg', lambda got, want: got < want, 'below'),
    'max_final_position_error': ('position_error', lambda got, want: got < want, 'below'),
    'max_final_heading_error_deg': (
        'heading_error_deg', lambda got, want: abs(got) < want, 'below'),
    'min_position_error': ('worst_position_error', lambda got, want: got > want, 'above'),
    'min_path_length': ('path_length', lambda got, want: got > want, 'above'),
    'min_path_rotation_deg': ('path_rotation_deg', lambda got, want: got > want, 'above'),
}


def report_path(scenario):
    return os.path.join(REPORT_DIR, scenario + '.json')


@pytest.mark.launch_test
@launch_testing.parametrize('scenario', SCENARIOS)
def generate_test_description(scenario):
    destination = report_path(scenario)
    os.makedirs(REPORT_DIR, exist_ok=True)
    if os.path.exists(destination):
        os.remove(destination)

    background, runner = scenario_actions(scenario, destination)
    return (
        LaunchDescription(background + [runner, launch_testing.actions.ReadyToTest()]),
        {'runner': runner},
    )


class TestScenarioCompletes(unittest.TestCase):

    def test_runner_exits(self, proc_info, runner):
        proc_info.assertWaitForShutdown(process=runner, timeout=300)


@launch_testing.post_shutdown_test()
class TestScenarioResult(unittest.TestCase):

    def test_runner_succeeded(self, proc_info, runner):
        launch_testing.asserts.assertExitCodes(proc_info, process=runner)

    def test_meets_expectations(self, scenario):
        destination = report_path(scenario)
        self.assertTrue(os.path.exists(destination),
                        'scenario %s wrote no report' % scenario)
        with open(destination, 'r') as handle:
            report = json.load(handle)

        expectations = load_scenario(scenario)['expect']
        for key, want in expectations.items():
            field, ok, direction = CHECKS[key]
            got = report[field]
            self.assertTrue(
                ok(got, want),
                '%s: %s was %.3f, expected %s %.3f\nfull report: %s'
                % (scenario, field, got, direction, want,
                   json.dumps(report, indent=2, sort_keys=True)))
