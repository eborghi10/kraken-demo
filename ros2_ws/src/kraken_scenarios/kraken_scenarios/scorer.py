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
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

from kraken_interfaces.msg import LocalisationScore


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


class Scorer(Node):

    def __init__(self):
        super().__init__('scorer')

        self.declare_parameter('truth_topic', '/ground_truth/odom')
        self.declare_parameter('estimate_topic', '/odometry/filtered')
        self.declare_parameter('publish_rate_hz', 10.0)

        self._truth = None
        self._estimate = None
        self._reference = None
        self._previous_truth = None
        self._path_length = 0.0
        self._path_rotation = 0.0
        self._worst_position = 0.0
        self._worst_heading = 0.0

        self.create_subscription(
            Odometry, self.get_parameter('truth_topic').value, self._on_truth, 10)
        self.create_subscription(
            Odometry, self.get_parameter('estimate_topic').value, self._on_estimate, 10)
        self._score_pub = self.create_publisher(LocalisationScore, '~/score', 10)
        self.create_service(Trigger, '~/mark_reference', self._on_mark_reference)
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self._publish)

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
        self._previous_truth = self._truth

    def _on_estimate(self, msg):
        self._estimate = pose_of(msg)

    def _on_mark_reference(self, request, response):
        del request
        if self._truth is None or self._estimate is None:
            response.success = False
            response.message = 'still waiting for ground truth or estimate'
            return response
        self._reference = (self._truth, self._estimate)
        self._previous_truth = self._truth
        self._path_length = 0.0
        self._path_rotation = 0.0
        self._worst_position = 0.0
        self._worst_heading = 0.0
        response.success = True
        response.message = 'reference marked'
        return response

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
