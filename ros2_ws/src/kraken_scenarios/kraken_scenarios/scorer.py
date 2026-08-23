#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Measures how far the filtered estimate has drifted from ground truth.

Errors are expressed in the frame of the pose pair captured when the reference
was marked. The map frame and the simulator's world frame are never exactly
aligned -- navsat_transform picks its origin from a datum, the simulator picks
its own -- and that constant offset is not a localisation error. Comparing
relative to a marked instant cancels it, so what is left is genuinely the drift
accumulated since the fault was injected.
"""
import functools
import math

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from std_srvs.srv import Trigger

from kraken_interfaces.msg import LocalisationScore

ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_of(msg):
    return (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_of(msg))


def relative(reference, current):
    """Pose of `current` expressed in the frame of `reference`."""
    dx = current[0] - reference[0]
    dy = current[1] - reference[1]
    cos_r, sin_r = math.cos(-reference[2]), math.sin(-reference[2])
    return (cos_r * dx - sin_r * dy,
            sin_r * dx + cos_r * dy,
            wrap(current[2] - reference[2]))


def segment_distance(point, start, end):
    """Distance from `point` to the segment `start`-`end`."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    squared = dx * dx + dy * dy
    if squared == 0.0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    along = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / squared
    along = min(1.0, max(0.0, along))
    return math.hypot(point[0] - start[0] - along * dx,
                      point[1] - start[1] - along * dy)


class Scorer(Node):

    def __init__(self):
        super().__init__('scorer')

        self.declare_parameter('truth_topic', 'ground_truth/odom')
        self.declare_parameter('estimate_topic', 'odometry/filtered')
        self.declare_parameter('plan_topic', 'plan')
        self.declare_parameter('publish_rate_hz', 10.0)
        # The behaviours the navigator is allowed to fall back on, as named in
        # nav2.yaml. Each one is an action server, so its status topic says both
        # how often it ran and for how long.
        self.declare_parameter('recovery_actions', ['backup', 'drive_on_heading', 'wait'])
        # How far a plan's endpoint has to move before it counts as a new goal
        # rather than a replan of the current one.
        self.declare_parameter('goal_tolerance_m', 0.5)

        self._truth = None
        self._estimate = None
        self._reference = None
        self._previous_truth = None
        self._path_length = 0.0
        self._path_rotation = 0.0
        self._worst_position = 0.0
        self._worst_heading = 0.0
        self._plan = None
        self._plan_goal = None
        self._cross_track = 0.0
        self._worst_cross_track = 0.0
        self._cross_track_sum = 0.0
        self._cross_track_samples = 0
        self._open_recoveries = {}
        self._recovery_count = 0
        self._recovery_time = 0.0
        self._reference_time = 0.0

        # The truth comes from whichever simulator is running, and O3DE
        # publishes best effort; the estimate comes from the EKF, which is
        # reliable and worth keeping that way.
        self.create_subscription(
            Odometry, self.get_parameter('truth_topic').value, self._on_truth,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('estimate_topic').value, self._on_estimate, 10)
        self.create_subscription(
            Path, self.get_parameter('plan_topic').value, self._on_plan, 10)
        for action in self.get_parameter('recovery_actions').value:
            self.create_subscription(
                GoalStatusArray, action + '/_action/status',
                functools.partial(self._on_recovery_status, action),
                qos_profile_action_status_default)
        self._score_pub = self.create_publisher(LocalisationScore, '~/score', 10)
        self.create_service(Trigger, '~/mark_reference', self._on_mark_reference)
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self._publish)

    def _seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_truth(self, msg):
        self._truth = pose_of(msg)
        if self._reference is not None:
            if self._previous_truth is not None:
                self._path_length += math.hypot(self._truth[0] - self._previous_truth[0],
                                                self._truth[1] - self._previous_truth[1])
                self._path_rotation += abs(wrap(self._truth[2] - self._previous_truth[2]))
            position_error, heading_error = self._errors()
            self._worst_position = max(self._worst_position, position_error)
            self._worst_heading = max(self._worst_heading, abs(heading_error))
            if self._plan is not None:
                self._cross_track = self._distance_to_plan()
                self._worst_cross_track = max(self._worst_cross_track, self._cross_track)
                self._cross_track_sum += self._cross_track
                self._cross_track_samples += 1
        self._previous_truth = self._truth

    def _on_estimate(self, msg):
        self._estimate = pose_of(msg)

    def _on_plan(self, msg):
        """Keep the first plan laid down for each goal, in the reference frame.

        Plans arrive in the map frame, which is the estimate's frame, so they
        are converted once here rather than on every truth sample.
        """
        if self._reference is None or len(msg.poses) < 2:
            return
        goal = msg.poses[-1].pose.position
        if self._plan is not None and math.hypot(goal.x - self._plan_goal[0],
                                                 goal.y - self._plan_goal[1]) <= \
                self.get_parameter('goal_tolerance_m').value:
            return
        self._plan_goal = (goal.x, goal.y)
        self._plan = [relative(self._reference[1],
                               (pose.pose.position.x, pose.pose.position.y, 0.0))[:2]
                      for pose in msg.poses]

    def _on_recovery_status(self, action, msg):
        """Count and time the navigator's fallbacks.

        A finished goal drops out of the status list once its result expires, so
        anything the server stops reporting is treated as finished rather than
        waiting for a terminal status that may never be seen.
        """
        now = self._seconds()
        active = set()
        for status in msg.status_list:
            key = (action, bytes(status.goal_info.goal_id.uuid))
            if status.status in ACTIVE:
                active.add(key)
                if key not in self._open_recoveries:
                    self._open_recoveries[key] = now
                    if self._reference is not None:
                        self._recovery_count += 1
            else:
                self._close_recovery(key, now)
        for key in [k for k in self._open_recoveries
                    if k[0] == action and k not in active]:
            self._close_recovery(key, now)

    def _close_recovery(self, key, now):
        started = self._open_recoveries.pop(key, None)
        if started is not None and self._reference is not None:
            self._recovery_time += max(0.0, now - max(started, self._reference_time))

    def _on_mark_reference(self, request, response):
        del request
        if self._truth is None or self._estimate is None:
            response.success = False
            response.message = 'still waiting for ground truth or estimate'
            return response
        self._reference = (self._truth, self._estimate)
        self._reference_time = self._seconds()
        self._previous_truth = self._truth
        self._path_length = 0.0
        self._path_rotation = 0.0
        self._worst_position = 0.0
        self._worst_heading = 0.0
        self._plan = None
        self._cross_track = 0.0
        self._worst_cross_track = 0.0
        self._cross_track_sum = 0.0
        self._cross_track_samples = 0
        self._recovery_count = 0
        self._recovery_time = 0.0
        response.success = True
        response.message = 'reference marked'
        return response

    def _distance_to_plan(self):
        truth = relative(self._reference[0], self._truth)
        return min(segment_distance(truth[:2], start, end)
                   for start, end in zip(self._plan, self._plan[1:]))

    def _errors(self):
        truth = relative(self._reference[0], self._truth)
        estimate = relative(self._reference[1], self._estimate)
        return (math.hypot(estimate[0] - truth[0], estimate[1] - truth[1]),
                wrap(estimate[2] - truth[2]))

    def _publish(self):
        if self._truth is None or self._estimate is None:
            return
        msg = LocalisationScore()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        if self._reference is not None:
            msg.position_error, msg.heading_error = self._errors()
            msg.worst_position_error = self._worst_position
            msg.worst_heading_error = self._worst_heading
            msg.path_length = self._path_length
            msg.path_rotation = self._path_rotation
            msg.cross_track_error = self._cross_track
            msg.worst_cross_track_error = self._worst_cross_track
            if self._cross_track_samples:
                msg.mean_cross_track_error = self._cross_track_sum / self._cross_track_samples
            msg.recovery_count = self._recovery_count
            # A recovery still running at the end of the run has not been closed
            # out yet, and its time is exactly the time that matters most.
            msg.recovery_time = self._recovery_time + sum(
                max(0.0, self._seconds() - max(started, self._reference_time))
                for started in self._open_recoveries.values())
            msg.elapsed_time = self._seconds() - self._reference_time
        self._score_pub.publish(msg)


def main():
    rclpy.init()
    node = Scorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
