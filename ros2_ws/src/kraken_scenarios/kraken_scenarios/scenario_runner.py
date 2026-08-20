#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Drives a scenario file: moves the robot, injects faults, writes a report.

Scenarios are data, not code, so adding a new failure mode to the suite means
adding a YAML file. The runner exits non-zero if it could not complete the
scenario; whether the *result* is acceptable is decided by the test that reads
the report, because "acceptable" differs per filter profile.
"""
import json
import math
import os
import sys
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

from kraken_interfaces.msg import FaultSpec, LocalisationScore
from kraken_interfaces.srv import SetFault
from kraken_scenarios.scorer import pose_of, relative

MODES = {
    'healthy': FaultSpec.MODE_HEALTHY,
    'dropout': FaultSpec.MODE_DROPOUT,
    'degrade': FaultSpec.MODE_DEGRADE,
    'bias_ramp': FaultSpec.MODE_BIAS_RAMP,
    'slip': FaultSpec.MODE_SLIP,
}


class ScenarioRunner(Node):

    def __init__(self):
        super().__init__('scenario_runner')

        self.declare_parameter('scenario', '')
        self.declare_parameter('report', '')
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('startup_timeout_s', 120.0)

        path = self.get_parameter('scenario').value
        if not path:
            raise ValueError('the scenario parameter is required')
        with open(path, 'r') as handle:
            self.scenario = yaml.safe_load(handle)

        self._score = None
        self._truth = None
        self._estimate = None
        self._reference = None
        self._navigation = None
        self.create_subscription(LocalisationScore, 'scorer/score', self._on_score, 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self._on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odometry/filtered', self._on_estimate, 10)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._set_fault = self.create_client(SetFault, 'fault_injector/set_fault')
        self._mark_reference = self.create_client(Trigger, 'scorer/mark_reference')
        self._navigate = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _on_score(self, msg):
        self._score = msg

    def _on_truth(self, msg):
        self._truth = pose_of(msg)

    def _on_estimate(self, msg):
        self._estimate = pose_of(msg)

    def _call(self, client, request, what):
        if not client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('service %s never appeared' % client.srv_name)
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            self._rate.sleep()
        response = future.result()
        if not response.success:
            raise RuntimeError('%s failed: %s' % (what, response.message))
        self.get_logger().info('%s: %s' % (what, response.message))

    def _wait_for_clock(self):
        # Under simulated time the clock reads zero until the first /clock
        # message lands. The headless sim starts at zero too, so this never
        # mattered; O3DE has usually been up for minutes by the time we attach,
        # and a deadline computed against zero is already blown when the real
        # time arrives. Wall sleep, because a rate on a stopped clock is one.
        if not self.get_parameter('use_sim_time').value:
            return
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            time.sleep(0.05)

    def _wait_for_stack(self):
        # The EKF needs a datum, a first fix and a first odometry message before
        # it publishes anything, so nothing before this point is meaningful.
        timeout = self.get_parameter('startup_timeout_s').value
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout
        idle = Twist()
        while rclpy.ok() and self._score is None:
            if self.get_clock().now().nanoseconds * 1e-9 > deadline:
                raise RuntimeError('localisation stack produced no estimate in %.0f s' % timeout)
            self._cmd_pub.publish(idle)
            self._rate.sleep()
        self.get_logger().info('localisation stack is up')

    def _drive(self, seconds, linear, angular):
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        end = self.get_clock().now().nanoseconds * 1e-9 + float(seconds)
        while rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 < end:
            self._cmd_pub.publish(command)
            self._rate.sleep()

    def _inject(self, phase):
        spec = FaultSpec()
        spec.channel = phase['channel']
        mode = phase['mode']
        if mode not in MODES:
            raise ValueError('unknown fault mode %r' % mode)
        spec.mode = MODES[mode]
        spec.noise_stddev = float(phase.get('noise_stddev', 0.0))
        spec.covariance_scale = float(phase.get('covariance_scale', 1.0))
        spec.bias_rate = float(phase.get('bias_rate', 0.0))
        spec.slip_ratio = float(phase.get('slip_ratio', 1.0))
        duration = float(phase.get('fault_duration', 0.0))
        spec.duration.sec = int(duration)
        spec.duration.nanosec = int((duration - int(duration)) * 1e9)
        self._call(self._set_fault, SetFault.Request(fault=spec),
                   'set_fault %s/%s' % (spec.channel, mode))

    def _seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _settle(self, future, deadline):
        while rclpy.ok() and not future.done():
            if self._seconds() > deadline:
                return None
            self._rate.sleep()
        return future.result()

    def _navigate_to(self, phase):
        """Drive to a goal with Nav2 and record both answers to "did it arrive".

        Nav2 decides it has arrived by looking at the filtered estimate. Whether
        the robot is actually there is a different question, and the gap between
        the two is the whole measurement.
        """
        if self._reference is None:
            raise ValueError('navigate_to needs an earlier mark_reference phase')
        if not self._navigate.wait_for_server(timeout_sec=60.0):
            raise RuntimeError('nav2 never advertised /navigate_to_pose')

        goal_pose = (float(phase['x']), float(phase['y']), float(phase.get('yaw', 0.0)))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = phase.get('frame', 'map')
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_pose[0]
        goal.pose.pose.position.y = goal_pose[1]
        goal.pose.pose.orientation.z = math.sin(0.5 * goal_pose[2])
        goal.pose.pose.orientation.w = math.cos(0.5 * goal_pose[2])

        deadline = self._seconds() + float(phase.get('timeout', 120.0))
        started = self._seconds()
        handle = self._settle(self._navigate.send_goal_async(goal), deadline)
        if handle is None:
            raise RuntimeError('nav2 never acknowledged the goal')
        if not handle.accepted:
            raise RuntimeError('nav2 rejected the goal')

        outcome = self._settle(handle.get_result_async(), deadline)
        if outcome is None:
            # A robot that never arrives is a result, not a crash. Cancel so the
            # controller stops driving before the next phase commands it.
            handle.cancel_goal_async()
            status = GoalStatus.STATUS_ABORTED
        else:
            status = outcome.status

        self._navigation = {
            'navigation_succeeded': 1.0 if status == GoalStatus.STATUS_SUCCEEDED else 0.0,
            'navigation_time_s': self._seconds() - started,
            'goal_error_estimated': math.hypot(self._estimate[0] - goal_pose[0],
                                               self._estimate[1] - goal_pose[1]),
            'goal_error_true': self._true_goal_error(goal_pose),
        }
        self.get_logger().info('navigate_to: %s' % json.dumps(self._navigation, sort_keys=True))

    def _true_goal_error(self, goal_pose):
        """How far the robot really is from the goal.

        The goal is in the map frame and ground truth is in the simulator's
        world frame, and those are offset by however far apart the two origins
        are. Expressing both relative to the marked pose pair cancels it, the
        same way the scorer cancels it for drift.
        """
        goal_in_reference = relative(self._reference[1], goal_pose)
        truth_in_reference = relative(self._reference[0], self._truth)
        return math.hypot(truth_in_reference[0] - goal_in_reference[0],
                          truth_in_reference[1] - goal_in_reference[1])

    def run(self):
        self._wait_for_clock()
        self._rate = self.create_rate(self.get_parameter('command_rate_hz').value)
        self._wait_for_stack()

        for phase in self.scenario['phases']:
            action = phase.get('action')
            label = phase.get('label', action or 'drive')
            self.get_logger().info('phase: %s' % label)
            if action == 'mark_reference':
                self._call(self._mark_reference, Trigger.Request(), 'mark_reference')
                # The scorer marks on its own latest pair; at 50 Hz the two are
                # at most a tick apart, which is centimetres on a metres-scale
                # measurement.
                self._reference = (self._truth, self._estimate)
            elif action == 'navigate_to':
                self._navigate_to(phase)
            elif action == 'set_fault':
                self._inject(phase)
            elif action is not None:
                raise ValueError('unknown action %r' % action)
            if 'duration' in phase:
                self._drive(phase['duration'], phase.get('linear', 0.0),
                            phase.get('angular', 0.0))

        self._cmd_pub.publish(Twist())
        if self._score is None:
            raise RuntimeError('no score was ever published')
        return self._report()

    def _report(self):
        score = self._score
        report = {
            'scenario': self.scenario['name'],
            'profile': self.scenario.get('profile', 'robust'),
            'position_error': score.position_error,
            'worst_position_error': score.worst_position_error,
            'heading_error_deg': math.degrees(score.heading_error),
            'worst_heading_error_deg': math.degrees(score.worst_heading_error),
            'path_length': score.path_length,
            'path_rotation_deg': math.degrees(score.path_rotation),
        }
        if self._navigation is not None:
            report.update(self._navigation)
        destination = self.get_parameter('report').value
        if destination:
            directory = os.path.dirname(destination)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(destination, 'w') as handle:
                json.dump(report, handle, indent=2, sort_keys=True)
        self.get_logger().info(json.dumps(report, indent=2, sort_keys=True))
        return report


def main():
    rclpy.init()
    node = ScenarioRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    status = 0
    try:
        node.run()
    except Exception as error:  # noqa: BLE001 - surfaced as a non-zero exit code
        node.get_logger().error(str(error))
        status = 1
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(status)


if __name__ == '__main__':
    main()
